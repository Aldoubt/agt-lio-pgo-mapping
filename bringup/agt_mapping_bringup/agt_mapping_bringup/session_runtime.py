"""Short-lived readiness/export helpers. ROS imports stay out of pure tests."""
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


def _wait_ready(rclpy, node, output):
    from std_srvs.srv import Trigger
    timeout = float(node.declare_parameter('startup_timeout', 45.0).value)
    lidar = node.declare_parameter('lidar_topic', '/livox/lidar').value
    imu = node.declare_parameter('imu_topic', '/livox/imu').value
    client = node.create_client(Trigger, '/mapping/backend/export_artifact')
    missing = []

    def ready():
        missing.clear()
        if not client.service_is_ready():
            missing.append('/mapping/backend/export_artifact')
        services = dict(node.get_service_names_and_types())
        if 'interface/srv/SaveMaps' not in services.get('/pgo/save_maps', []):
            missing.append('/pgo/save_maps')
        # Confirm the consumer chain exists BEFORE any recorded samples are published.
        required = {
            lidar: {'mid360_adapter_node'},
            imu: {'lio_node'},
            '/mapping/sensor/livox': {'lio_node'},
            '/mapping/frontend/cloud': {'pgo_node'},
            '/mapping/frontend/odometry': {'pgo_node', 'pgo_backend_node'},
            '/mapping/backend/status': {'mapping_artifact_exporter'},
        }
        for topic, consumers in required.items():
            present = {info.node_name for info in node.get_subscriptions_info_by_topic(topic)}
            if not consumers.issubset(present):
                missing.append(f'{topic} -> {sorted(consumers - present)}')
        return not missing

    node.get_logger().info(f'[1/4] Waiting up to {timeout:g}s for the mapping pipeline')
    try:
        wait_until(ready, lambda dt: _spin(rclpy, node, dt), timeout, 'mapping readiness')
    except TimeoutError as exc:
        raise TimeoutError(f'{exc}; missing: {", ".join(missing)}') from exc
    mark_session(output, 'ready', 'Required services and sensor/backend consumers discovered')
    node.get_logger().info('[1/4] Pipeline ready; rosbag playback may start')


def _export(rclpy, node, output):
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from agt_mapping_artifacts.validation import ArtifactValidationError, verify_artifact

    timeout = float(node.declare_parameter('export_timeout', 180.0).value)
    drain = float(node.declare_parameter('drain_seconds', 3.0).value)
    deadline = time.monotonic() + timeout
    activity = [time.monotonic()]
    failure = []
    client = node.create_client(Trigger, '/mapping/backend/export_artifact')
    node.create_subscription(Odometry, '/mapping/frontend/odometry',
                             lambda _: activity.__setitem__(0, time.monotonic()), 50)

    def backend_status(message):
        if message.data in {'pgo_export_failed', 'pgo_export_empty',
                            'pgo_save_service_unavailable', 'frontend_stale_artifact_export_rejected'}:
            failure.append(message.data)

    node.create_subscription(String, '/mapping/backend/status', backend_status, 10)

    def remaining():
        value = deadline - time.monotonic()
        if value <= 0:
            raise TimeoutError('PGO export/verification deadline exceeded')
        return value

    spin = lambda dt: _spin(rclpy, node, dt)
    mark_session(output, 'draining', f'Waiting for {drain:g}s frontend quiet window')
    node.get_logger().info('[3/4] Playback finished; waiting for frontend quiet window')
    wait_until(lambda: time.monotonic() - activity[0] >= drain, spin, remaining(), 'frontend quiet window')
    wait_until(client.service_is_ready, spin, remaining(), 'export service')
    mark_session(output, 'exporting', 'Requesting optimized PGO export')
    future = client.call_async(Trigger.Request())
    wait_until(future.done, spin, remaining(), 'export acknowledgement')
    response = future.result()
    if response is None or not response.success:
        raise RuntimeError(f'PGO rejected export: {getattr(response, "message", "no response")}')
    node.get_logger().info('[3/4] Export accepted; this is NOT yet a saved/verified map')
    mark_session(output, 'verifying', 'Export accepted; waiting for complete artifact and checksums')
    last_error = 'map_package not produced yet'
    while time.monotonic() < deadline:
        if failure:
            raise RuntimeError(f'Backend export failed: {failure[-1]}')
        if (output / 'map_package' / 'checksums.sha256').is_file():
            try:
                root = verify_artifact(output, deadline=deadline)
            except ArtifactValidationError as exc:
                last_error = str(exc)
            else:
                mark_session(output, 'completed', 'Optimized PGO artifact verified',
                             artifact_path=str(root), artifact_verified=True)
                node.get_logger().info(f'[4/4] Artifact VERIFIED: {root}')
                return
        spin(min(0.5, max(0.0, deadline - time.monotonic())))
    raise TimeoutError(f'No verified map within {timeout:g}s: {last_error}')


def wait_ready_main(args=None):
    return _run('wait_ready', _wait_ready, args)


def export_main(args=None):
    return _run('export_verified', _export, args)
