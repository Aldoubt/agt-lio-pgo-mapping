import pytest
from shapely.geometry import box

from agt_coverage_authoring.metrics import coverage_area_metrics, work_polylines
from agt_coverage_authoring.models import PathPoint


def test_union_overlap_and_empty_coverage_preserve_target():
    target=box(0,0,10,4)
    report=coverage_area_metrics(target,[[(0,1),(10,1)],[(0,2),(10,2)]],2.)
    assert report["target_area_m2"]==40.
    assert report["covered_area_m2"]==30.
    assert report["overlap_area_m2"]==10.
    assert report["missed_area_m2"]==10.
    empty=coverage_area_metrics(target,[],2.)
    assert empty["coverage_rate"]==0 and empty["missed_area_m2"]==40.


def test_connections_never_count_as_work():
    points=[PathPoint(0,0,0,kind="SWATH",segment_id="work_a"),
            PathPoint(2,0,0,kind="SWATH",segment_id="work_a"),
            PathPoint(2,0,0,kind="CONNECTION",segment_id="turn"),
            PathPoint(8,0,0,kind="CONNECTION",segment_id="turn"),
            PathPoint(8,0,0,kind="SWATH",segment_id="work_b"),
            PathPoint(10,0,0,kind="SWATH",segment_id="work_b")]
    swaths=work_polylines(points)
    assert swaths==[[(0,0),(2,0)],[(8,0),(10,0)]]
    report=coverage_area_metrics(box(0,-.5,10,.5),swaths,1.)
    assert report["coverage_rate"]==pytest.approx(.4)


def test_zero_length_work_geometry_is_invalid():
    with pytest.raises(ValueError):
        coverage_area_metrics(box(0,0,1,1),[[(.5,.5),(.5,.5)]],.5)
