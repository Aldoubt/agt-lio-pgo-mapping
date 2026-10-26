"""Deterministic whole-task coverage authoring with explicit incomplete results.

The v2 SWATH/CONNECTION separation is preserved. Work geometry is authoritative
input; only connections are delegated to a planner. A failed connection causes
other complete route orders to be evaluated rather than altered work geometry.
"""
from __future__ import annotations

from dataclasses import replace
import math
import time

from shapely.geometry import LineString
from shapely.ops import unary_union

from .backends import make_backend
from .metrics import coverage_area_metrics, trajectory_metrics, work_polylines
from .models import CoverageError, PathPoint, PlanResult, Pose, WorkRow


def _angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def _pose(point):
    return Pose(point.x, point.y, point.yaw)


def _same(a, b):
    return math.hypot(a.x-b.x, a.y-b.y) < 1e-8 and abs(_angle(a.yaw-b.yaw)) < 1e-8


def _resolve_heading(context):
    """Resolve the work axis before asking a backend to generate area swaths."""
    if context.heading is not None:
        heading, source = float(context.heading), "explicit"
    elif context.rows:
        coordinates = context.rows[0].centerline
        if len(coordinates) < 2 or math.dist(coordinates[0], coordinates[-1]) <= 1e-9:
            raise CoverageError("INVALID_WORK_DIRECTION", "First confirmed row must have distinct endpoints")
        heading = math.atan2(coordinates[-1][1]-coordinates[0][1],
                             coordinates[-1][0]-coordinates[0][0])
        source = "confirmed_row"
    else:
        target = context.work_area
        if target is None or target.is_empty or not target.is_valid or target.area <= 0:
            raise CoverageError("INVALID_WORK_AREA", "Resolving work direction requires confirmed rows or a valid work area")
        corners = list(target.minimum_rotated_rectangle.exterior.coords)
        edges = [(math.dist(a,b), math.atan2(b[1]-a[1],b[0]-a[0]) % math.pi)
                 for a,b in zip(corners,corners[1:])]
        longest = max(length for length,_ in edges)
        heading = min(angle for length,angle in edges
                      if math.isclose(length,longest,rel_tol=1e-9,abs_tol=1e-9))
        source = "work_area_long_axis"
    if not math.isfinite(heading):
        raise CoverageError("INVALID_WORK_DIRECTION", "Work direction must be finite")
    heading %= math.pi
    if math.isclose(heading,math.pi,abs_tol=1e-12):
        heading = 0.
    return heading,source


def sample_row(row: WorkRow, context, against=False) -> list[PathPoint]:
    """Sample confirmed geometry, never replace curved rows by endpoint chords."""
    coordinates = list(reversed(row.centerline)) if against else list(row.centerline)
    if len(coordinates) < 2 or LineString(coordinates).length <= 1e-8:
        raise CoverageError("ZERO_LENGTH_SWATH", f"Row {row.id} must have positive length")
    step = min(context.sample_step, context.grid.resolution / 2)
    output = []
    for start, end in zip(coordinates, coordinates[1:]):
        context.check_cancelled()
        length = math.dist(start, end)
        if length <= 1e-9:
            continue
        count = math.ceil(length / step)
        if count + len(output) > 50000:
            raise CoverageError("RESOURCE_LIMIT", f"Row {row.id} has too many samples")
        yaw = math.atan2(end[1]-start[1], end[0]-start[0])
        # A sharp corner retains both headings at the vertex so the Ackermann
        # validator rejects it rather than silently cutting through the crop.
        if output and abs(_angle(output[-1].yaw-yaw)) < 1e-8:
            output.pop()
        for index in range(count+1):
            ratio = index/count
            output.append(PathPoint(start[0]+ratio*(end[0]-start[0]), start[1]+ratio*(end[1]-start[1]), yaw,
                direction="F", v_ref=context.robot.max_forward_speed,
                segment_id="work_"+row.id, kind="SWATH", semantic_ref=row.id))
    return output


def _append(route, part, segment_id=None):
    if not part:
        return
    if len(route)+len(part)>250000:
        raise CoverageError("RESOURCE_LIMIT","Joined route exceeds 250000 points")
    copied = [replace(p, segment_id=segment_id or p.segment_id) for p in part]
    if route:
        if not _same(route[-1], copied[0]):
            raise CoverageError("DISCONNECTED_SEGMENT", "Connection failed to preserve exact endpoint pose")
        if route[-1].direction != copied[0].direction:
            route[-1].v_ref = copied[0].v_ref = 0.
            route[-1].event_ref = copied[0].event_ref = "gear_change"
        # Keep coincident boundary poses: outgoing kind and direction then
        # remain unambiguous without dropping either work endpoint.
    route.extend(copied)


def _orders(rows, heading):
    nx, ny = -math.sin(heading), math.cos(heading)
    ordered = sorted(rows, key=lambda r: (
        sum(x*nx+y*ny for x,y in r.centerline)/len(r.centerline), r.id))
    sequences = [ordered, ordered[::2]+ordered[1::2]]
    sequences += [list(reversed(order)) for order in list(sequences)]
    seen = set()
    for sequence in sequences:
        for initial_against in (False, True):
            candidate = tuple((row.id, bool(index % 2) != initial_against) for index,row in enumerate(sequence))
            if candidate not in seen:
                seen.add(candidate)
                yield candidate


def plan_coverage(context, backend=None, progress=None) -> PlanResult:
    """Plan without publishing ROS paths, modifying maps, or enabling motion.

    Partial output retains every requested target in the denominator. A partial
    route is advisory; it cannot be relabelled as a READY whole task.
    """
    from .grid import footprint_at
    from .validation import validate_path
    progress = progress or (lambda event: None)
    original_cancelled = context.cancelled
    deadline = time.monotonic() + context.planning_timeout
    def cancelled():
        if original_cancelled():
            return True
        if time.monotonic() >= deadline:
            raise CoverageError("PLANNING_TIMEOUT", "Whole authoring job exceeded planning_timeout")
        return False
    context = replace(context, cancelled=cancelled)
    context.check_cancelled()
    if context.mode not in ("rows", "area"):
        raise CoverageError("UNSUPPORTED_MODE", f"Unknown coverage mode {context.mode!r}")
    heading,heading_source = _resolve_heading(context)
    # The request snapshot remains unchanged; the resolved value is derived
    # provenance, shared by swath generation and route-order comparison.
    context = replace(context,heading=heading)
    owned = backend is None
    backend = backend or make_backend(context)
    try:
        progress({"stage": "swaths", "detail": "Preparing confirmed work geometry"})
        rows = list(context.rows) if context.mode == "rows" else list(backend.generate_swaths(context))
        if not rows or len(rows)>1000 or len({r.id for r in rows})!=len(rows):
            raise CoverageError("INVALID_WORK_ROWS", "Need 1..1000 uniquely identified work rows")
        variants = {}
        row_errors = {}
        for index,row in enumerate(rows):
            context.check_cancelled()
            variants[row.id] = {}
            for against, direction in ((False,"along"),(True,"against")):
                if direction not in row.allowed_directions:
                    continue
                try:
                    points = sample_row(row,context,against)
                    allowed_corridor = (row.corridor.union(unary_union(getattr(context,"headlands",[])))
                                        if row.corridor is not None else None)
                    if allowed_corridor is not None and any(not allowed_corridor.covers(footprint_at(p,context.robot).buffer(context.clearance_margin)) for p in points):
                        raise CoverageError("AISLE_TOO_NARROW", f"Full footprint leaves confirmed corridor {row.id}")
                    validation = validate_path(points=points,context=context,check_dynamic=False)
                    if validation["valid"]:
                        variants[row.id][against] = points
                    else:
                        row_errors[row.id] = validation["issues"][0]["code"] if validation["issues"] else "INVALID_WORK_GEOMETRY"
                except CoverageError as error:
                    if error.code in ("CANCELED","PLANNING_TIMEOUT"):
                        raise
                    row_errors[row.id] = error.code
            progress({"stage":"validate_work", "current":index+1,"total":len(rows)})
        orders = list(_orders(rows,heading))
        cache = {}
        def connection(start,end,allow_reverse):
            context.check_cancelled()
            if _same(start,end):
                return []
            key = (start,end,allow_reverse)
            if key not in cache:
                try:
                    points = backend.connect(start,end,context,allow_reverse=allow_reverse)
                    if not points or not _same(points[0],start) or not _same(points[-1],end):
                        raise CoverageError("ENDPOINT_MISMATCH","Backend connection endpoints differ from request")
                    report = validate_path(points=points,context=context,check_dynamic=False)
                    if not report["valid"]:
                        raise CoverageError("INVALID_CONNECTION", ",".join(report.get("error_codes",[])))
                    if not allow_reverse and any(p.direction=="R" for p in points):
                        raise CoverageError("REVERSE_NOT_ALLOWED","Backend returned reverse motion during forward pass")
                    cache[key] = points
                except CoverageError as error:
                    if error.code in ("CANCELED","PLANNING_TIMEOUT","BACKEND_UNAVAILABLE","BACKEND_PROFILE_MISMATCH","PRIVATE_DOMAIN_REQUIRED"):
                        raise
                    cache[key] = error
            if isinstance(cache[key],CoverageError):
                raise cache[key]
            return cache[key]
        candidates = []
        # Evaluate all forward orders before allowing any reversing solution.
        for allow_reverse in ([False,True] if context.allow_reverse else [False]):
            for order_index,order in enumerate(orders):
                context.check_cancelled()
                route, visited, missing = [],[],[]
                current = context.start
                for row_id, preferred_against in order:
                    options = variants[row_id]
                    if not options:
                        missing.append({"row_id":row_id,"code":row_errors.get(row_id,"DIRECTION_NOT_ALLOWED")})
                        continue
                    choices = sorted(options,key=lambda against: against!=preferred_against)
                    failure = None
                    for against in choices:
                        work = options[against]
                        try:
                            edge = connection(current,_pose(work[0]),allow_reverse)
                            _append(route,edge,f"connection_{len(visited):04d}")
                            _append(route,work)
                            current = _pose(work[-1])
                            visited.append(row_id)
                            failure = None
                            break
                        except CoverageError as error:
                            if error.code in ("CANCELED","PLANNING_TIMEOUT","BACKEND_UNAVAILABLE","BACKEND_PROFILE_MISMATCH"):
                                raise
                            failure = error
                    if failure:
                        missing.append({"row_id":row_id,"code":failure.code,"detail":str(failure)})
                returned = False
                if visited:
                    try:
                        _append(route,connection(current,context.end,allow_reverse),"return_connection")
                        returned = True
                    except CoverageError as error:
                        if error.code in ("CANCELED","PLANNING_TIMEOUT","BACKEND_UNAVAILABLE","BACKEND_PROFILE_MISMATCH"):
                            raise
                        missing.append({"row_id":None,"code":error.code,"detail":"Return/exit connection: "+str(error)})
                metrics = trajectory_metrics(route,context.robot)
                metrics.update({"target_rows":len(rows),"covered_rows":len(visited),"row_coverage_rate":len(visited)/len(rows)})
                if context.operation_width is not None and context.work_area is not None:
                    metrics["area"] = coverage_area_metrics(context.work_area,work_polylines(route),context.operation_width)
                else:
                    metrics["area"] = None
                complete = len(visited)==len(rows) and returned
                if context.mode == "area" and metrics["area"]["missed_area_m2"] > 1e-6:
                    complete = False
                    missing.append({"row_id":None,"code":"UNCOVERED_WORK_AREA",
                                    "missed_area_m2":metrics["area"]["missed_area_m2"]})
                joined_validation = validate_path(points=route,context=context)
                if not joined_validation["valid"]:
                    complete=False
                    missing.append({"row_id":None,"code":"JOINED_ROUTE_INVALID",
                                    "issues":joined_validation["issues"][:5]})
                candidate = {"points":route,"visited":visited,"missing":missing,"metrics":metrics,"complete":complete,
                             "order_index":order_index,"allow_reverse":allow_reverse,"returned":returned,
                             "_validation":joined_validation}
                candidates.append(candidate)
                progress({"stage":"connections","current":order_index+1,"total":len(orders),"allow_reverse":allow_reverse})
            if any(c["complete"] for c in candidates):
                break
        def rank(candidate):
            m = candidate["metrics"]
            coverage = m["area"]["covered_area_m2"] if context.mode == "area" else m["covered_rows"]
            return (not candidate["_validation"]["valid"],not candidate["complete"],-coverage,not candidate["returned"],m["gear_changes"],m["reverse_length_m"],m["cruise_time_lower_bound_s"],m["connection_length_m"],candidate["order_index"])
        chosen = min(candidates,key=rank)
        points = chosen["points"]
        validation = chosen["_validation"]
        if not validation["valid"]:
            chosen["complete"] = False
        for index,point in enumerate(points):
            point.curvature = validation["per_point_curvature"][index]
            point.clearance = validation["per_point_clearance"][index]
        blockers = []
        if any(p.direction=="R" for p in points):
            blockers.append("REVERSE_UNSUPPORTED")
        if len({round(p.v_ref,9) for p in points})>1:
            blockers.append("VARIABLE_SPEED_UNSUPPORTED")
        if not chosen["complete"]:
            blockers.append("INCOMPLETE_TASK")
        if validation["dynamic_status"] != "VALID":
            blockers.append("DYNAMIC_CONSTRAINTS_UNVERIFIED" if validation["dynamic_status"]=="UNKNOWN" else "DYNAMIC_CONSTRAINTS_INVALID")
        if any(abs(value)>1e-12 for value in context.robot.rear_axle_to_base):
            blockers.append("REFERENCE_POINT_UNSUPPORTED")
        summaries = [{k:v for k,v in candidate.items() if k not in ("points","_validation")} for candidate in candidates]
        report = {
            "status":"COMPLETE" if chosen["complete"] else "PARTIAL" if chosen["visited"] and validation["valid"] else "FAILED",
            "mode":context.mode,"metrics":chosen["metrics"],"visited_rows":chosen["visited"],
            "validation":validation,"candidates":summaries,"eligible_for_execution":False,
            "trajectory_reference":"rear_axle_center",
            "runtime_compatibility":{"compatible":not blockers,"blocked_by":blockers},
            "denominator_policy":"all_requested_rows_and_original_work_area",
            "area_completion_tolerance_m2":1e-6,
            "resolved_heading_rad":heading,"heading_source":heading_source,
            "search_scope":"bounded_order_candidates; failure is not a proof of global unreachability",
        }
        provenance = dict(getattr(backend,"provenance",{}))
        provenance["input_files"] = dict(context.input_files)
        provenance.update({"resolved_heading_rad":heading,"heading_source":heading_source})
        return PlanResult(points,chosen["complete"],report,chosen["missing"],provenance)
    finally:
        if owned:
            backend.close()
