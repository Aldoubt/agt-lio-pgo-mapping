from pathlib import Path
import pytest
from agt_mapping_bringup.cli import parser, _live_plan
from agt_mapping_bringup.preflight import PreflightError


def test_live_cli_exposes_external_calibration():
    args = parser().parse_args(
        ["--live", "--fastlio-config", "/mounted/calibration.yaml"]
    )
    assert args.fastlio_config == "/mounted/calibration.yaml"


def test_both_live_launches_declare_calibration():
    root = Path(__file__).resolve().parents[1]
    for name in ["mapping_live_mid360.launch.py", "mapping_live_yhs_mid360.launch.py"]:
        assert (
            "DeclareLaunchArgument('fastlio_config'"
            in (root / "launch" / name).read_text()
        )
    source = (root / "agt_mapping_bringup/live_launch.py").read_text()
    assert "'calibration_file': text(lio_config)" in source
