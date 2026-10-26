from dataclasses import replace
import math

import numpy as np
import pytest
from shapely.geometry import box
from shapely.ops import unary_union

from agt_coverage_authoring.backends import ReferenceBackend, RosBackend
from agt_coverage_authoring.models import CoverageError, GridMap, PlanningContext, Pose, RobotProfile, WorkRow
from agt_coverage_authoring.planner import plan_coverage


def context(**changes):
    robot = RobotProfile("test", "sha256:"+"a"*64, .6, 1.5,
                         ((.3,.2),(.3,-.2),(-.3,-.2),(-.3,.2)))
    base = PlanningContext(GridMap(np.zeros((200,200),dtype=np.int8),.1,(0.,0.,0.)),
        robot, box(0,0,20,20), [WorkRow("row_1",((5.,5.),(10.,5.)))],
        box(5,4.5,10,5.5), [], Pose(5,5,0), Pose(10,5,0), backend="reference",
        sample_step=.1, planning_timeout=30.)
    return replace(base,**changes)


def test_single_row_real_reference_backend_preserves_endpoints():
    result = plan_coverage(context())
    assert result.complete
    assert result.report["metrics"]["covered_rows"] == 1
    assert result.report["metrics"]["work_length_m"] == pytest.approx(5.)
    assert (result.points[0].x,result.points[-1].x)==(5.,10.)
    assert result.report["validation"]["geometric_valid"]
    assert result.report["validation"]["dynamic_status"] == "UNKNOWN"
    assert not result.report["eligible_for_execution"]


def test_forward_priority_and_return_to_entry_real_curves():
    c=context(rows=[WorkRow("a",((5.,5.),(10.,5.))),WorkRow("b",((5.,8.),(10.,8.)))],end=Pose(5,5,0))
    result=plan_coverage(c)
    assert result.complete
    assert result.report["metrics"]["covered_rows"]==2
    assert result.report["metrics"]["gear_changes"]==0
    assert all(point.direction=="F" for point in result.points)
    assert (result.points[-1].x,result.points[-1].y)==(5,5)
    assert all(not candidate["allow_reverse"] for candidate in result.report["candidates"])


def test_dead_end_requires_reverse_and_keeps_full_route():
    c=context(boundary=box(2,2.5,7,3.5),rows=[WorkRow("dead_end",((3.,3.),(6.,3.)))],
              start=Pose(3,3,0),end=Pose(3,3,0))
    result=plan_coverage(c)
    assert result.complete
    assert result.report["metrics"]["reverse_length_m"]==pytest.approx(3.)
    assert "REVERSE_UNSUPPORTED" in result.report["runtime_compatibility"]["blocked_by"]
    cusps=[(a,b) for a,b in zip(result.points,result.points[1:]) if a.direction!=b.direction]
    assert cusps and all(a.v_ref==b.v_ref==0 and a.event_ref==b.event_ref=="gear_change" for a,b in cusps)
    forward_only=plan_coverage(replace(c,allow_reverse=False))
    assert not forward_only.complete
    assert forward_only.report["metrics"]["target_rows"]==1


def test_blocked_row_remains_in_denominator():
    c=context(rows=[WorkRow("clear",((5.,5.),(10.,5.))),WorkRow("blocked",((5.,8.),(10.,8.)))],
              keepouts=[box(7,7.5,8,8.5)])
    result=plan_coverage(c)
    assert not result.complete
    assert result.report["metrics"]["target_rows"]==2
    assert result.report["metrics"]["covered_rows"]==1
    assert result.report["metrics"]["row_coverage_rate"]==.5
    assert any(item["row_id"]=="blocked" for item in result.missing)


def test_area_coverage_uses_operation_width_not_vehicle_width():
    c=context(mode="area",operation_width=1.,work_area=box(5,4.5,10,5.5))
    result=plan_coverage(c)
    assert result.complete
    assert result.report["metrics"]["area"]["coverage_rate"]==pytest.approx(1.)
    assert result.report["metrics"]["area"]["covered_area_m2"]==pytest.approx(5.)
    assert result.provenance["swath_algorithm"].endswith("not Fields2Cover)")


def test_area_swaths_follow_confirmed_rotated_aisle_before_generation():
    c=context(mode="area",operation_width=1.,work_area=box(4.5,5,5.5,10),
              rows=[WorkRow("north_row",((5.,5.),(5.,10.)))],
              start=Pose(5,5,math.pi/2),end=Pose(5,10,math.pi/2))
    result=plan_coverage(c)
    assert result.complete
    assert result.report["metrics"]["target_rows"]==1
    assert result.report["metrics"]["area"]["coverage_rate"]==pytest.approx(1.)
    assert all(abs(math.cos(p.yaw))<1e-8 for p in result.points if p.kind=="SWATH")
    assert result.provenance["resolved_heading_rad"]==pytest.approx(math.pi/2)
    assert result.report["heading_source"]=="confirmed_row"
    assert c.heading is None  # Derived planning context never rewrites input.


def test_explicit_heading_overrides_confirmed_row_direction():
    c=context(mode="area",operation_width=1.,heading=0.,
              rows=[WorkRow("north_row",((5.,5.),(5.,10.)))])
    result=plan_coverage(c)
    assert result.complete
    assert result.report["heading_source"]=="explicit"
    assert result.provenance["resolved_heading_rad"]==0.
    assert all(abs(math.sin(p.yaw))<1e-8 for p in result.points if p.kind=="SWATH")


def test_area_without_rows_uses_long_axis_and_reports_reference_limit():
    c=context(mode="area",operation_width=1.,work_area=box(4.5,5,5.5,10),rows=[],
              start=Pose(5,5,math.pi/2),end=Pose(5,10,math.pi/2))
    c.robot=replace(c.robot,rear_axle_to_base=(.3,0.))
    result=plan_coverage(c)
    assert result.complete
    assert result.report["heading_source"]=="work_area_long_axis"
    assert result.report["resolved_heading_rad"]==pytest.approx(math.pi/2)
    assert "REFERENCE_POINT_UNSUPPORTED" in result.report["runtime_compatibility"]["blocked_by"]


def test_cancellation_and_deadline_are_not_partial_success():
    with pytest.raises(CoverageError,match="canceled") as error:
        plan_coverage(context(cancelled=lambda:True))
    assert error.value.code=="CANCELED"
    with pytest.raises(CoverageError) as error:
        plan_coverage(context(planning_timeout=-1))
    assert error.value.code=="PLANNING_TIMEOUT"


def test_sharp_polyline_corner_is_not_silently_cut():
    c=context(rows=[WorkRow("corner",((5.,5.),(7.,5.),(7.,7.)))])
    result=plan_coverage(c)
    assert not result.complete and not result.points
    assert result.report["metrics"]["target_rows"]==1


def test_ros_never_silently_falls_back_and_requires_private_domain(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID","0")
    with pytest.raises(CoverageError) as error:
        RosBackend(context(backend="ros",backend_options={"ros_domain_id":0}))
    assert error.value.code=="PRIVATE_DOMAIN_REQUIRED"


def test_ros_rejects_rotated_costmap_before_connecting():
    c=context()
    c.grid.origin=(0.,0.,.2)
    with pytest.raises(CoverageError) as error:
        RosBackend(c)
    assert error.value.code=="UNSUPPORTED_MAP_YAW"


def test_reference_swaths_cover_each_disconnected_component():
    c=context(mode="area",operation_width=1.,work_area=unary_union([
        box(5,4.6,10,5.4),box(5,7.1,10,7.9)]))
    rows=ReferenceBackend().generate_swaths(c)
    assert len(rows)==2
    assert rows[0].centerline[0][1]==pytest.approx(5.)
    assert rows[1].centerline[0][1]==pytest.approx(7.5)


def test_integer_swath_count_is_stable_under_floating_point_roundoff():
    c=context(mode="area",operation_width=.8,work_area=unary_union([
        box(5,4.6,10,5.4),box(5,7.1,10,7.9)]))
    assert len(ReferenceBackend().generate_swaths(c))==2


def test_confirmed_headlands_allow_full_vehicle_at_work_endpoints():
    c=context(rows=[WorkRow("row",((5.,5.),(10.,5.)),corridor=box(5,4.5,10,5.5))])
    blocked=plan_coverage(c)
    assert not blocked.complete
    accepted=plan_coverage(replace(c,headlands=[box(4,4,5.1,6),box(9.9,4,11,6)]))
    assert accepted.complete


def test_partial_area_does_not_shrink_target_to_valid_swaths():
    c=context(mode="area",operation_width=1.,work_area=box(5,4.5,10,6.5),
              keepouts=[box(6,5.8,8,6.2)])
    result=plan_coverage(c)
    assert not result.complete
    area=result.report["metrics"]["area"]
    assert area["target_area_m2"]==10.
    assert area["coverage_rate"]<1
    assert any(item["code"]=="UNCOVERED_WORK_AREA" for item in result.missing)
