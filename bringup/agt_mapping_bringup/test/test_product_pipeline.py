"""Product plugin selection tests run without ROS, Qt, a bag, or robot hardware."""
from argparse import Namespace
from pathlib import Path

import pytest
import yaml

from agt_mapping_bringup.product_pipeline import (
    PROCESSORS, SENSOR_INPUTS, ProductPipelineError,
    apply_to_cli, capability_report, load_product_pipeline,
)


def _write(tmp_path, **changes):
    config = {
        'schema_version': 1,
        'sensor': {'id': 'mid360_custom', 'lidar_topic': 'auto', 'imu_topic': 'auto'},
        'frontend': {'backend_id': 'fast_livo2_lio'},
        'processors': {'occupancy_review': 'manual', 'relocalization_study': 'manual'},
    }
    config.update(changes)
    path = tmp_path / 'product.yaml'
    path.write_text(yaml.safe_dump(config), encoding='utf-8')
    return path


def test_mid360_default_decouples_input_and_processing(tmp_path):
    plan = load_product_pipeline(_write(tmp_path))
    assert plan.sensor_id == 'mid360_custom'
    assert plan.frontend_id == 'fast_livo2_lio'
    assert set(plan.processors) == {'occupancy_review', 'relocalization_study'}
    assert plan.as_dict()['optional_processors'][0]['execution'] == 'manual'


def test_capability_listing_does_not_advertise_all_backends_as_runnable():
    report = capability_report()
    assert report['no_automatic_fallback']
    assert report['backend_authority'].endswith('resolve_backend')
    assert set(SENSOR_INPUTS) == {'mid360_custom'}
    assert {'dynamic_cleanup_3d', 'terrain_traversability'} <= set(PROCESSORS)
    by_id = {p['id']: p for p in report['frontends']}
    assert by_id['fast_livo2_lio']['session_status'] == 'verified'
    assert by_id['point_lio']['session_status'] == 'registered_not_verified'


@pytest.mark.parametrize('frontend', ['point_lio', 'lio_sam_noloop', 'fast_lio2_legacy'])
def test_unverified_frontend_fails_closed(tmp_path, frontend):
    with pytest.raises(ProductPipelineError, match='lacks a verified operator'):
        load_product_pipeline(_write(tmp_path, frontend={'backend_id': frontend}))


@pytest.mark.parametrize('bad_sensor', ['other_lidar', None, ['mid360_custom']])
def test_unknown_sensor_fails_closed(tmp_path, bad_sensor):
    with pytest.raises(ProductPipelineError, match='no verified input adapter'):
        load_product_pipeline(_write(tmp_path, sensor={'id': bad_sensor}))


@pytest.mark.parametrize('stage', ['dynamic_cleanup_3d', 'terrain_traversability', 'route_authoring'])
def test_no_fake_auto_processing_or_fake_ready(tmp_path, stage):
    with pytest.raises(ProductPipelineError, match='no accepted manual'):
        load_product_pipeline(_write(tmp_path, processors={stage: 'manual'}))


def test_unknown_stage_or_mode_rejected(tmp_path):
    with pytest.raises(ProductPipelineError, match='unknown keys'):
        load_product_pipeline(_write(tmp_path, processors={'shell': 'manual'}))
    with pytest.raises(ProductPipelineError, match='only disabled/manual'):
        load_product_pipeline(_write(tmp_path, processors={'occupancy_review': 'auto'}))


def test_sensor_topics_are_validated_before_launch(tmp_path):
    with pytest.raises(ProductPipelineError, match='absolute ROS topic'):
        load_product_pipeline(_write(
            tmp_path, sensor={'id': 'mid360_custom', 'lidar_topic': 'relative',
                              'imu_topic': 'auto'}))
    with pytest.raises(ProductPipelineError, match='must differ'):
        load_product_pipeline(_write(
            tmp_path, sensor={'id': 'mid360_custom', 'lidar_topic': '/same',
                              'imu_topic': '/same'}))


def test_cli_selection_accepts_default_and_detects_conflicts(tmp_path):
    plan = load_product_pipeline(_write(
        tmp_path, sensor={'id': 'mid360_custom', 'lidar_topic': '/agt/sensors/lidar/custom',
                          'imu_topic': '/agt/sensors/imu/data'}))
    args = Namespace(backend=None, lidar_topic='auto', imu_topic='auto', live=False)
    apply_to_cli(args, plan)
    assert args.backend == 'fast_livo2_lio'
    assert args.lidar_topic == '/agt/sensors/lidar/custom'
    args.backend = 'point_lio'
    with pytest.raises(ProductPipelineError, match='conflicts'):
        apply_to_cli(args, plan)


def test_map_store_only_resolves_reference_without_creating_it(tmp_path):
    plan = load_product_pipeline(_write(tmp_path, map_store={'root': 'maps'}))
    assert plan.map_store == str(tmp_path / 'maps')
    assert not (tmp_path / 'maps').exists()


def test_schema_and_unknown_fields_fail_explicitly(tmp_path):
    with pytest.raises(ProductPipelineError, match='schema_version'):
        load_product_pipeline(_write(tmp_path, schema_version=2))
    with pytest.raises(ProductPipelineError, match='unknown keys'):
        load_product_pipeline(_write(tmp_path, unwanted='value'))
