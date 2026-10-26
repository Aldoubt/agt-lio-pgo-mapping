"""Readiness and verified-export helpers for the FAST-LIVO2 source workflow."""
from pathlib import Path
import time

from .session_state import mark_session


def wait_until(check, spin, timeout, description, *, now=time.monotonic):
    deadline = now() + timeout
    while not check():
        remaining = deadline - now()
        if remaining <= 0:
            raise TimeoutError(f'Timed out waiting for {description} ({timeout:g}s)')
        spin(min(0.1, remaining))


def _spin(rclpy, node, seconds):
    if not rclpy.ok():
        raise InterruptedError('ROS context stopped')
    rclpy.spin_once(node, timeout_sec=seconds)


def _run(stage, action, args=None):
    import rclpy
    from rclpy.node import Node
    rclpy.init(args=args)
    node = Node('mapping_' + stage)
    output = Path(node.declare_parameter('output_dir', '').value)
    code = 1
    try:
        action(rclpy, node, output)
        code = 0
    except (KeyboardInterrupt, InterruptedError):
        mark_session(output, 'cancelled', f'{stage} interrupted; no success claimed')
        code = 130
    except Exception as exc:
        node.get_logger().error(str(exc))
        mark_session(output, 'failed', f'{stage}: {exc}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


def _require_frontend_mode(node):
    mode = node.declare_parameter('reference_mode', 'frontend').value
    if mode != 'frontend':
        raise ValueError('only reference_mode=frontend is supported; PGO and FAST-LIO2 paths are disabled')


def _wait_ready(rclpy, node, output):
    from std_srvs.srv import Trigger
    timeout = float(node.declare_parameter('startup_timeout', 45.0).value)
    lidar = node.declare_parameter('lidar_topic', '/agt/sensors/lidar/custom').value
    imu = node.declare_parameter('imu_topic', '/agt/sensors/imu/data').value
    _require_frontend_mode(node)
    require_publishers = bool(node.declare_parameter('require_publishers', False).value)
    client = node.create_client(Trigger, '/mapping/frontend/export_artifact')
    missing = []

    def ready():
        missing.clear()
        if not client.service_is_ready():
            missing.append('/mapping/frontend/export_artifact')
        if require_publishers:
            for topic in (lidar, imu):
                if not node.get_publishers_info_by_topic(topic):
                    missing.append(f'publisher on {topic} (sensor stream is not live)')
        required = {
            lidar: {'fastlivo_mapping'},
            imu: {'fastlivo_mapping'},
            '/aft_mapped_to_init': {'mapping_frontend_adapter'},
            '/cloud_registered_lidar': {'mapping_frontend_adapter'},
            '/mapping/frontend/odometry': {'frontend_map_exporter'},
            '/mapping/frontend/cloud': {'frontend_map_exporter'},
        }
        for topic, consumers in required.items():
            present = {info.node_name for info in node.get_subscriptions_info_by_topic(topic)}
            if not consumers.issubset(present):
                missing.append(f'{topic} -> {sorted(consumers - present)}')
        return not missing

    node.get_logger().info(f'[1/4] Waiting up to {timeout:g}s for FAST-LIVO2 LIO-only pipeline')
    try:
        wait_until(ready, lambda dt: _spin(rclpy, node, dt), timeout, 'mapping readiness')
    except TimeoutError as exc:
        raise TimeoutError(f'{exc}; missing: {", ".join(missing)}') from exc
    mark_session(output, 'ready', 'Frontend export service and all required sensor/map consumers discovered')
    node.get_logger().info('[1/4] Pipeline ready; ' + (
        'live sensor streams are being consumed' if require_publishers else 'rosbag playback may start'))


def _export(rclpy, node, output):
    from diagnostic_msgs.msg import DiagnosticArray
    from nav_msgs.msg import Odometry
    from std_srvs.srv import Trigger
    _require_frontend_mode(node)
    from agt_mapping_artifacts.frontend_package import verify_frontend_map_package

    package_root = output / 'map_package'
    timeout = float(node.declare_parameter('export_timeout', 180.0).value)
    drain = float(node.declare_parameter('drain_seconds', 3.0).value)
    deadline = time.monotonic() + timeout
    activity = [time.monotonic()]
    failure = []
    client = node.create_client(Trigger, '/mapping/frontend/export_artifact')
    node.create_subscription(Odometry, '/mapping/frontend/odometry',
                             lambda _: activity.__setitem__(0, time.monotonic()), 50)

    def frontend_status(message):
        for status in message.status:
            if status.name != 'mapping/frontend/exporter':
                continue
            values = {item.key: item.value for item in status.values}
            if values.get('artifact_state') == 'failed':
                failure.append(status.message or 'frontend exporter reported failure')

    node.create_subscription(DiagnosticArray, '/mapping/frontend/status', frontend_status, 10)

    def remaining():
        value = deadline - time.monotonic()
        if value <= 0:
            raise TimeoutError('frontend map export/verification deadline exceeded')
        return value

    spin = lambda dt: _spin(rclpy, node, dt)
    mark_session(output, 'draining', f'Waiting for {drain:g}s frontend quiet window')
    node.get_logger().info('[3/4] Playback/recording finished; waiting for frontend quiet window')
    wait_until(lambda: time.monotonic() - activity[0] >= drain, spin, remaining(), 'frontend quiet window')
    wait_until(client.service_is_ready, spin, remaining(), 'frontend export service')
    mark_session(output, 'exporting', 'Requesting FAST-LIVO2 same-session paired patch/pose package')
    future = client.call_async(Trigger.Request())
    wait_until(future.done, spin, remaining(), 'frontend export acknowledgement')
    response = future.result()
    if response is None or not response.success:
        raise RuntimeError(f'Frontend map exporter rejected export: {getattr(response, "message", "no response")}')
    node.get_logger().info('[3/4] Export accepted; this is NOT yet a verified map package')
    mark_session(output, 'verifying', 'Export accepted; validating paired map, keyframes, poses, and checksums')
    last_error = 'map_package/checksums.sha256 not produced yet'
    while time.monotonic() < deadline:
        if failure:
            raise RuntimeError(f'Frontend map export failed: {failure[-1]}')
        if (package_root / 'checksums.sha256').is_file():
            try:
                result = verify_frontend_map_package(package_root)
            except Exception as exc:
                last_error = str(exc)
            else:
                mark_session(
                    output, 'completed', 'FAST-LIVO2 LIO-only same-session source package verified',
                    artifact_path=str(package_root), artifact_verified=True,
                    backend_id=result['backend_id'], reference_type='FRONTEND_SAME_SESSION_REFERENCE',
                    keyframe_count=result['keyframes'], map_point_count=result['map_points'],
                    map_frame=result['map_frame'])
                node.get_logger().info(f'[4/4] Source map package VERIFIED: {package_root}; {result}')
                return
        spin(min(0.5, max(0.0, deadline - time.monotonic())))
    raise TimeoutError(f'No verified frontend map package within {timeout:g}s: {last_error}')


def wait_ready_main(args=None):
    return _run('wait_ready', _wait_ready, args)


def export_main(args=None):
    return _run('export_verified', _export, args)
