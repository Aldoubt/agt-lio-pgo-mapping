from pathlib import Path

import pytest

from agt_map_localization_benchmark.relocalization_mvp import (
    _ambiguity_status, algorithm_stale_reasons, native_dependency_fingerprints,
    supports_candidate_trace,
)


def test_trace_capability_is_detected_from_existing_native_usage(tmp_path: Path):
    binary = tmp_path / 'trace_capable'
    binary.write_text('#!/bin/sh\necho "Usage: localizer [--trace-candidates-json PATH]"\n', encoding='utf-8')
    binary.chmod(0o755)
    assert supports_candidate_trace(binary)


def test_non_trace_native_is_detected_without_running_analysis(tmp_path: Path):
    binary = tmp_path / 'baseline'
    binary.write_text('#!/bin/sh\necho \'{"success":false,"message":"unknown argument: --help"}\'\n',
                      encoding='utf-8')
    binary.chmod(0o755)
    assert not supports_candidate_trace(binary)


def _record():
    return {
        'algorithm': {
            'binary_sha256': {'candidate_localizer': 'a' * 64},
            'config': {'global_settings': {'candidate_top_k': 4}},
            'code_provenance': {'head': 'commit', 'source_state_sha256': 'b' * 64},
        },
    }


def test_current_algorithm_revision_passes_staleness_gate():
    assert algorithm_stale_reasons(
        _record(), current_binary_sha256={'candidate_localizer': 'a' * 64},
        current_code_provenance={'head': 'commit', 'source_state_sha256': 'b' * 64},
        current_global_settings={'candidate_top_k': 4}) == []


@pytest.mark.parametrize(('change', 'reason'), [
    ('binary', 'native algorithm binary revision changed'),
    ('config', 'native GLOBAL analysis configuration changed'),
    ('source', 'mapping analysis source revision changed'),
])
def test_binary_config_or_source_change_marks_evidence_stale(change: str, reason: str):
    record = _record()
    binaries = {'candidate_localizer': 'a' * 64}
    provenance = {'head': 'commit', 'source_state_sha256': 'b' * 64}
    settings = {'candidate_top_k': 4}
    if change == 'binary': binaries['candidate_localizer'] = 'c' * 64
    elif change == 'config': settings['candidate_top_k'] = 10
    else: provenance['source_state_sha256'] = 'd' * 64
    assert reason in algorithm_stale_reasons(record, current_binary_sha256=binaries,
                                             current_code_provenance=provenance,
                                             current_global_settings=settings)


def test_native_dependency_fingerprints_use_workspace_source_root(tmp_path, monkeypatch):
    from agt_map_localization_benchmark import relocalization_mvp

    workspace = tmp_path / 'ros2_ws'
    repository = workspace / 'src' / 'agt_mapping_framework'
    install = workspace / 'install'
    captured = {}
    monkeypatch.setattr(relocalization_mvp, 'dependency_fingerprints',
                        lambda ros_install, src_root: captured.update(
                            ros_install=ros_install, src_root=src_root) or {'fixture': {}})

    result = native_dependency_fingerprints(install, repository)

    assert result == {'fixture': {}}
    assert captured['ros_install'] == install.resolve()
    assert captured['src_root'] == workspace / 'src'


def test_ambiguity_is_known_only_with_two_ranked_candidates_scored():
    assert _ambiguity_status([
        {'descriptor_sector_similarity': 0.8, 'bbs_score': None},
        {'descriptor_sector_similarity': None, 'bbs_score': None},
    ]) == 'NO_DATA'
    assert _ambiguity_status([
        {'descriptor_sector_similarity': 0.8, 'bbs_score': None},
        {'descriptor_sector_similarity': None, 'bbs_score': 0.7},
    ]) == 'KNOWN'
    assert _ambiguity_status([
        {'descriptor_sector_similarity': True, 'bbs_score': None},
        {'descriptor_sector_similarity': float('nan'), 'bbs_score': None},
    ]) == 'NO_DATA'
