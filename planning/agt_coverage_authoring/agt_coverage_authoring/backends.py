"""Explicit planning backends; importing this module never imports or starts ROS.

The production adapter consumes the pinned OpenNav humble-v2 swath action and
Nav2 Smac Hybrid-A*. The reference adapter uses rsplan 1.0.10 curve enumeration;
it does NOT search around obstacles and is never an automatic ROS fallback.
"""
from __future__ import annotations

from dataclasses import replace
import importlib.metadata
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time

from shapely import affinity
from shapely.geometry import LineString

from .models import CoverageError, PathPoint, Pose, WorkRow


def _angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def _lines(geometry):
    if geometry.is_empty:
        return []
    if geometry.geom_type == "LineString":
        return [geometry]
    return [line for part in getattr(geometry, "geoms", ()) for line in _lines(part)]


def _canonical_swaths(rows, heading):
    """Stable geometry ordering, independent of server return ordering."""
    dx,dy=math.cos(heading),math.sin(heading)
    normalized=[]
    for row in rows:
        coordinates=row.centerline
        if (coordinates[-1][0]-coordinates[0][0])*dx+(coordinates[-1][1]-coordinates[0][1])*dy<0:
            coordinates=tuple(reversed(coordinates))
        normalized.append(replace(row,centerline=coordinates))
    normalized.sort(key=lambda row:(
        sum(-x*dy+y*dx for x,y in row.centerline)/len(row.centerline),row.centerline))
    return [replace(row,id=f"swath_{index:04d}") for index,row in enumerate(normalized)]


class ReferenceBackend:
    """Bounded analytic candidates, explicitly selected for offline comparison."""
    provenance = {
        "backend": "reference", "connection_algorithm": "rsplan 1.0.10 Reeds-Shepp candidates",
        "swath_algorithm": "Shapely parallel clipping (reference, not Fields2Cover)",
        "obstacle_search": False, "eligible_for_execution": False,
    }

    def __init__(self):
        try:
            version = importlib.metadata.version("rsplan")
        except importlib.metadata.PackageNotFoundError as exc:
            raise CoverageError("BACKEND_UNAVAILABLE", "Install the reference extra: rsplan==1.0.10") from exc
        if version != "1.0.10":
            raise CoverageError("BACKEND_VERSION_MISMATCH", "Reference adapter requires rsplan==1.0.10")

    def generate_swaths(self, context):
        width = context.operation_width
        if width is None or not math.isfinite(width) or width <= 0:
            raise CoverageError("INVALID_OPERATION_WIDTH", "Area coverage requires a positive operation width")
        target = context.work_area
        if target is None or target.is_empty or not target.is_valid or target.area <= 0:
            raise CoverageError("INVALID_WORK_AREA", "Area coverage requires a valid work polygon")
        heading = context.heading if context.heading is not None else 0.
        rows = []
        polygons = list(target.geoms) if target.geom_type == "MultiPolygon" else [target]
        for polygon in sorted(polygons,key=lambda part:part.bounds):
            local = affinity.rotate(polygon, -heading, origin=(0, 0), use_radians=True)
            minx, miny, maxx, maxy = local.bounds
            ratio=(maxy-miny)/width
            nearest=round(ratio)
            count=max(1,nearest if math.isclose(ratio,nearest,rel_tol=1e-10,abs_tol=1e-10) else math.ceil(ratio))
            if count + len(rows) > 10000:
                raise CoverageError("RESOURCE_LIMIT", "Work area would generate more than 10000 swath levels")
            for index in range(count):
                context.check_cancelled()
                y = miny + (index + .5) * (maxy - miny) / count
                parts = _lines(LineString([(minx - width, y), (maxx + width, y)]).intersection(local))
                for part in sorted(parts, key=lambda line: line.bounds):
                    world = affinity.rotate(part, heading, origin=(0, 0), use_radians=True)
                    if world.length > 1e-8:
                        rows.append(WorkRow(f"swath_{len(rows):04d}", tuple(world.coords)))
        if not rows:
            raise CoverageError("NO_SWATHS", "No positive-length swaths generated")
        return _canonical_swaths(rows,heading)

    def connect(self, start, end, context, allow_reverse=False):
        from rsplan.planner import _solve_path
        from .validation import validate_path
        deadline = time.monotonic() + context.connection_timeout
        step = min(context.sample_step, context.grid.resolution / 2)
        if math.hypot(end.x-start.x, end.y-start.y) < 1e-9 and abs(_angle(end.yaw-start.yaw)) < 1e-9:
            return []
        try:
            # Private enumerator intentionally version pinned: public path() discards
            # other curves before occupancy/forward-only checks can run.
            candidates = _solve_path((start.x, start.y, start.yaw), (end.x, end.y, end.yaw),
                                     context.robot.min_turning_radius, step)
        except (ValueError, ZeroDivisionError, FloatingPointError) as exc:
            raise CoverageError("SEARCH_EXHAUSTED", "Reference curve enumeration failed") from exc
        candidates = [path for path in candidates if allow_reverse or
                      all(segment.direction == 1 or abs(segment.length) < 1e-10 for segment in path.segments)]
        candidates.sort(key=lambda path: (
            sum(a.direction != b.direction for a, b in zip(path.segments, path.segments[1:])),
            sum(abs(s.length) for s in path.segments if s.direction == -1), path.total_length))
        for candidate in candidates:
            context.check_cancelled()
            if time.monotonic() >= deadline:
                raise CoverageError("CONNECTION_TIMEOUT", "Reference candidate validation timed out")
            if candidate.total_length / step > 50000:
                continue
            points = []
            pose = (start.x, start.y, start.yaw)
            for segment in candidate.segments:
                if abs(segment.length) < 1e-10:
                    continue
                samples = segment.calc_waypoints(pose, step, False)
                segment_points = [PathPoint(float(p[0]), float(p[1]), _angle(float(p[2])),
                    direction="F" if segment.direction == 1 else "R",
                    v_ref=context.robot.max_forward_speed if segment.direction == 1 else context.robot.max_reverse_speed,
                    curvature=float(p[3])) for p in samples]
                if points and segment_points:
                    if points[-1].direction != segment_points[0].direction:
                        points[-1].v_ref = segment_points[0].v_ref = 0.
                        points[-1].event_ref = segment_points[0].event_ref = "gear_change"
                    else:
                        points.pop()  # Shared position now belongs to the outgoing interval.
                points.extend(segment_points)
                if points:
                    pose = (points[-1].x, points[-1].y, points[-1].yaw)
            if len(points) < 2:
                continue
            # Do not hide solver endpoint errors by making a long snapped chord.
            if math.hypot(points[-1].x-end.x, points[-1].y-end.y) > 1e-5 or abs(_angle(points[-1].yaw-end.yaw)) > 1e-5:
                continue
            points[0] = replace(points[0], x=start.x, y=start.y, yaw=start.yaw)
            points[-1] = replace(points[-1], x=end.x, y=end.y, yaw=end.yaw)
            report = validate_path(points=points, context=context, check_dynamic=False)
            if report["valid"]:
                return points
        raise CoverageError("SEARCH_EXHAUSTED", "No collision-free analytic candidate; obstacle-search reachability remains unknown")

    def close(self):
        pass


class RosBackend:
    """Client of separately launched private offline planning servers only."""
    provenance = {
        "backend": "ros", "connection_algorithm": "Nav2 SmacPlannerHybrid",
        "swath_algorithm": "OpenNav/Fields2Cover",
        "required_opennav_commit": "f413d0da7b4e52249b9abdb9d1fec7cef0238449",
        "required_fields2cover_commit": "3613525c241538fa9fd9df3e1209ae8184627958",
        "installed_revision_verified": False,
        "eligible_for_execution": False,
    }

    def __init__(self, context):
        options = context.backend_options
        if abs(_angle(context.grid.origin[2])) > 1e-9:
            raise CoverageError("UNSUPPORTED_MAP_YAW", "Smac's axis-aligned costmap requires a zero-yaw planning raster; core/reference mode preserves rotated maps")
        domain = options.get("ros_domain_id")
        if isinstance(domain, bool) or not isinstance(domain, int) or not 0 <= domain <= 232:
            raise CoverageError("PRIVATE_DOMAIN_REQUIRED", "backend_options.ros_domain_id must select an explicit offline ROS domain")
        if domain == int(os.environ.get("ROS_DOMAIN_ID", "0")):
            raise CoverageError("PRIVATE_DOMAIN_REQUIRED", "Offline backend domain must differ from the caller's live ROS_DOMAIN_ID")
        namespace = options.get("namespace", "/agt_coverage_offline")
        if not isinstance(namespace,str) or not re.fullmatch(r"/[A-Za-z_][A-Za-z0-9_/]*",namespace):
            raise CoverageError("PRIVATE_NAMESPACE_REQUIRED", "Use a non-root absolute offline namespace")
        namespace=namespace.rstrip("/")
        if type(options.get("auto_launch",True)) is not bool:
            raise CoverageError("BACKEND_CONFIGURATION","auto_launch must be a boolean")
        for key in ("forward_planner","reverse_planner"):
            if key in options and (not isinstance(options[key],str) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*",options[key])):
                raise CoverageError("BACKEND_CONFIGURATION",f"Invalid {key} identifier")
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.node import Node
            from rclpy.action import ActionClient
            from nav2_msgs.action import ComputePathToPose
            from opennav_coverage_msgs.action import ComputeCoveragePath
        except ImportError as exc:
            raise CoverageError("BACKEND_UNAVAILABLE", "Source ROS 2 Humble and the pinned OpenNav/Smac workspace; no reference fallback was used") from exc
        self.rclpy = rclpy
        self.context = Context()
        self.context.init(args=[], domain_id=domain)
        self.node = Node("coverage_authoring_client", context=self.context, namespace=namespace)
        self.nav_action = ComputePathToPose
        self.coverage_action = ComputeCoveragePath
        self.namespace = namespace
        self.path_client = ActionClient(self.node, ComputePathToPose, "compute_path_to_pose")
        self.swath_client = ActionClient(self.node, ComputeCoveragePath, "compute_coverage_path")
        self.forward_id = options.get("forward_planner", "CoverageForwardHybrid")
        self.reverse_id = options.get("reverse_planner", "CoverageRepairHybrid")
        self._checked = set()
        self.process = None
        self.process_log = None
        if options.get("auto_launch", True):
            try:
                planning_map = options.get("planning_map")
                if not planning_map or not Path(planning_map).is_file() or not context.profile_path:
                    raise CoverageError("BACKEND_CONFIGURATION", "Owned ROS backend requires derived planning_map and canonical profile_path")
                if self.path_client.wait_for_server(timeout_sec=.2) or self.swath_client.wait_for_server(timeout_sec=.2):
                    raise CoverageError("BACKEND_ALREADY_RUNNING", "Offline namespace is already occupied; choose another domain or explicitly use auto_launch=false")
                self.process_log = tempfile.TemporaryFile(mode="w+b")
                env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
                self.process = subprocess.Popen([
                    "ros2","launch","agt_coverage_authoring","offline_backend.launch.py",
                    f"planning_map:={Path(planning_map).resolve()}",
                    f"platform_profile:={context.profile_path}",
                    f"operation_width:={context.operation_width or 1.0}",
                    f"namespace:={namespace}",
                    f"connection_timeout:={context.connection_timeout}",
                    f"forward_planner:={self.forward_id}", f"reverse_planner:={self.reverse_id}",
                ],env=env,start_new_session=True,stdout=self.process_log,stderr=subprocess.STDOUT)
            except (OSError,CoverageError) as exc:
                self.close()
                if isinstance(exc,CoverageError):
                    raise
                raise CoverageError("BACKEND_UNAVAILABLE", f"Cannot launch private offline servers: {exc}") from exc

    def _alive(self):
        if self.process is not None and self.process.poll() is not None:
            self.process_log.seek(0,2)
            length = self.process_log.tell()
            self.process_log.seek(max(0,length-2000))
            detail = self.process_log.read().decode("utf-8",errors="replace")
            raise CoverageError("BACKEND_PROCESS_FAILED", "Owned offline servers exited: "+detail)

    def _wait(self, future, context, deadline):
        while not future.done():
            context.check_cancelled()
            self._alive()
            if time.monotonic() >= deadline:
                raise CoverageError("CONNECTION_TIMEOUT", "Offline ROS request timed out")
            self.rclpy.spin_once(self.node, timeout_sec=min(.05, max(0., deadline-time.monotonic())))
        if future.exception() is not None:
            raise CoverageError("BACKEND_ERROR", str(future.exception()))
        return future.result()

    def _parameters(self, server, names, context):
        from rcl_interfaces.srv import GetParameters
        from lifecycle_msgs.srv import GetState
        # A lifecycle node advertises parameters before plugin declarations in
        # configure(); wait for ACTIVE rather than treating startup as mismatch.
        state_client = self.node.create_client(GetState, f"{self.namespace}/{server}/get_state")
        startup_deadline=time.monotonic()+max(30.,context.connection_timeout)
        try:
            while True:
                context.check_cancelled()
                self._alive()
                if time.monotonic()>=startup_deadline:
                    raise CoverageError("BACKEND_UNAVAILABLE",f"Offline {server} did not become ACTIVE")
                if not state_client.wait_for_service(timeout_sec=.1):
                    continue
                state=self._wait(state_client.call_async(GetState.Request()),context,startup_deadline)
                if state.current_state.id==3:
                    break
                self.rclpy.spin_once(self.node,timeout_sec=.1)
        finally:
            self.node.destroy_client(state_client)
        deadline = time.monotonic() + min(5., context.connection_timeout)
        client = self.node.create_client(GetParameters, f"{self.namespace}/{server}/get_parameters")
        try:
            while not client.wait_for_service(timeout_sec=.1):
                context.check_cancelled()
                self._alive()
                if time.monotonic() >= deadline:
                    raise CoverageError("BACKEND_UNAVAILABLE", f"Offline {server} parameter service unavailable")
            request = GetParameters.Request()
            request.names = names
            values = self._wait(client.call_async(request), context, deadline).values
            output = []
            for value in values:
                output.append(value.double_value if value.type == 3 else value.string_value if value.type == 4 else value.bool_value if value.type == 1 else None)
            return output
        finally:
            self.node.destroy_client(client)

    def _goal(self, client, goal, context):
        deadline = time.monotonic() + context.connection_timeout
        while not client.wait_for_server(timeout_sec=.1):
            context.check_cancelled()
            self._alive()
            if time.monotonic() >= deadline:
                raise CoverageError("BACKEND_UNAVAILABLE", "Private offline action server unavailable")
        handle = None
        send_future = None
        try:
            send_future=client.send_goal_async(goal)
            handle = self._wait(send_future, context, deadline)
            if not handle.accepted:
                raise CoverageError("GOAL_REJECTED", "Offline planner rejected the request")
            result = self._wait(handle.get_result_async(), context, deadline)
            if result.status != 4:
                raise CoverageError("SEARCH_EXHAUSTED", f"Offline action finished with status {result.status}")
            return result.result
        except CoverageError:
            # Cancellation may race goal acceptance; collect the handle within
            # a bounded grace period so an accepted child is also canceled.
            if handle is None and send_future is not None:
                accept_deadline=time.monotonic()+1.
                while not send_future.done() and time.monotonic()<accept_deadline:
                    self.rclpy.spin_once(self.node,timeout_sec=.05)
                if send_future.done() and send_future.exception() is None:
                    handle=send_future.result()
            if handle is not None and handle.accepted:
                cancel = handle.cancel_goal_async()
                cancel_deadline = time.monotonic() + 1.
                while not cancel.done() and time.monotonic() < cancel_deadline:
                    self.rclpy.spin_once(self.node, timeout_sec=.05)
            raise

    def generate_swaths(self, context):
        from opennav_coverage_msgs.msg import Coordinate, Coordinates
        width = context.operation_width
        if width is None or width <= 0:
            raise CoverageError("INVALID_OPERATION_WIDTH", "Area coverage needs operation_width")
        actual = self._parameters("coverage_server", ["operation_width"], context)[0]
        if actual is None or not math.isclose(actual, width, abs_tol=1e-8):
            raise CoverageError("BACKEND_PROFILE_MISMATCH", "Configure private Coverage Server with this operation_width before lifecycle configure")
        target = context.work_area
        polygons = list(target.geoms) if target.geom_type == "MultiPolygon" else [target]
        rows = []
        for polygon in polygons:
            goal = self.coverage_action.Goal()
            goal.frame_id = "map"
            goal.generate_headland = False
            goal.generate_route = False
            goal.generate_path = False
            goal.use_gml_file = False
            goal.swath_mode.mode = "SET_ANGLE"
            goal.swath_mode.objective = "LENGTH"
            goal.swath_mode.best_angle = float(context.heading or 0.)
            for ring in [polygon.exterior, *polygon.interiors]:
                coordinates = Coordinates()
                for x, y in ring.coords:
                    point = Coordinate()
                    point.axis1, point.axis2 = float(x), float(y)
                    coordinates.coordinates.append(point)
                goal.polygons.append(coordinates)
            result = self._goal(self.swath_client, goal, context)
            if result.error_code != 0:
                raise CoverageError("SWATH_GENERATION_FAILED", f"OpenNav error {result.error_code}")
            if result.coverage_path.header.frame_id != "map":
                raise CoverageError("FRAME_MISMATCH", "OpenNav swaths must use map frame")
            for swath in result.coverage_path.swaths:
                coords = ((float(swath.start.x), float(swath.start.y)), (float(swath.end.x), float(swath.end.y)))
                if any(not math.isfinite(v) for xy in coords for v in xy) or math.dist(*coords) <= 1e-8:
                    raise CoverageError("ZERO_LENGTH_SWATH", "OpenNav returned invalid swath geometry")
                if not target.buffer(1e-5).covers(LineString(coords)):
                    raise CoverageError("SWATH_OUTSIDE_TARGET", "OpenNav returned a swath outside the requested work area")
                rows.append(WorkRow(f"swath_{len(rows):04d}", coords))
        if not rows:
            raise CoverageError("NO_SWATHS", "OpenNav returned no work swaths")
        return _canonical_swaths(rows,float(context.heading or 0.))

    def connect(self, start, end, context, allow_reverse=False):
        from geometry_msgs.msg import PoseStamped
        planner_id = self.reverse_id if allow_reverse else self.forward_id
        if planner_id not in self._checked:
            radius, model, plugin, unknown = self._parameters("planner_server", [
                planner_id+".minimum_turning_radius", planner_id+".motion_model_for_search",
                planner_id+".plugin", planner_id+".allow_unknown"], context)
            if radius is None or not math.isclose(radius, context.robot.min_turning_radius, abs_tol=1e-8) or model != ("REEDS_SHEPP" if allow_reverse else "DUBIN") or "SmacPlannerHybrid" not in str(plugin) or unknown is not False:
                raise CoverageError("BACKEND_PROFILE_MISMATCH", "Private Smac planner model/radius/unknown policy differs from the selected robot")
            self._checked.add(planner_id)
        goal = self.nav_action.Goal()
        def stamped(pose):
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.pose.position.x, msg.pose.position.y = float(pose.x), float(pose.y)
            msg.pose.orientation.z, msg.pose.orientation.w = math.sin(pose.yaw/2), math.cos(pose.yaw/2)
            return msg
        goal.start, goal.goal = stamped(start), stamped(end)
        goal.use_start = True
        goal.planner_id = planner_id
        result = self._goal(self.path_client, goal, context)
        if result.path.header.frame_id != "map" or len(result.path.poses) < 2:
            raise CoverageError("INVALID_CONNECTION", "Smac returned an empty path or wrong frame")
        points = []
        for stamped_pose in result.path.poses:
            p, q = stamped_pose.pose.position, stamped_pose.pose.orientation
            if stamped_pose.header.frame_id not in ("", "map") or not all(math.isfinite(x) for x in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)) or abs(q.x)>1e-6 or abs(q.y)>1e-6 or abs(q.z*q.z+q.w*q.w-1)>1e-4:
                raise CoverageError("INVALID_CONNECTION", "Smac path pose is not finite planar map geometry")
            points.append(PathPoint(float(p.x), float(p.y), _angle(2*math.atan2(q.z,q.w))))
        # Smac Path carries body yaw but no gear field. Derive gear only for
        # nonzero edges, then independently validate no-slip and curvature.
        directions = []
        for a,b in zip(points, points[1:]):
            dx,dy = b.x-a.x,b.y-a.y
            if math.hypot(dx,dy) < 1e-9:
                raise CoverageError("INVALID_CONNECTION", "Smac returned a zero-length interval")
            heading = a.yaw + _angle(b.yaw-a.yaw)/2
            directions.append("F" if dx*math.cos(heading)+dy*math.sin(heading) >= 0 else "R")
        if not allow_reverse and "R" in directions:
            raise CoverageError("REVERSE_NOT_ALLOWED", "Forward planner returned reverse motion")
        directions.append(directions[-1])
        output = []
        for i,(point,direction) in enumerate(zip(points,directions)):
            point.direction = direction
            point.v_ref = context.robot.max_reverse_speed if direction == "R" else context.robot.max_forward_speed
            if i and directions[i-1] != direction:
                output.append(replace(point, direction=directions[i-1], v_ref=0., event_ref="gear_change"))
                point.v_ref, point.event_ref = 0., "gear_change"
            output.append(point)
        for actual, expected in ((output[0], start),(output[-1],end)):
            if math.hypot(actual.x-expected.x,actual.y-expected.y)>1e-4 or abs(_angle(actual.yaw-expected.yaw))>1e-4:
                raise CoverageError("ENDPOINT_MISMATCH", "Smac connection does not preserve exact required endpoint poses")
            # Only this already checked numerical tolerance may be snapped;
            # the common validator then rechecks the modified endpoint edges.
            actual.x,actual.y,actual.yaw=expected.x,expected.y,expected.yaw
        return output

    def close(self):
        if self.process is not None and self.process.poll() is None:
            try:
                os.killpg(self.process.pid,signal.SIGINT)
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid,signal.SIGTERM)
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid,signal.SIGKILL)
                    self.process.wait(timeout=2)
            except ProcessLookupError:
                pass
        if self.process_log is not None:
            self.process_log.close()
        self.path_client.destroy()
        self.swath_client.destroy()
        self.node.destroy_node()
        self.context.shutdown()


def make_backend(context):
    if context.backend == "reference":
        return ReferenceBackend()
    if context.backend == "ros":
        return RosBackend(context)
    raise CoverageError("UNSUPPORTED_BACKEND", f"Unknown backend {context.backend!r}")
