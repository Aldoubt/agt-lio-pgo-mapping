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
    reference_mode = node.declare_parameter('reference_mode', 'pgo').value
    if reference_mode not in ('pgo', 'fastlio'):
        raise ValueError('reference_mode must be pgo or fastlio')
    require_publishers = bool(node.declare_parameter('require_publishers', False).value)
    client = node.create_client(Trigger, '/mapping/backend/export_artifact')
    missing = []

    def ready():
        missing.clear()
        if not client.service_is_ready():
            missing.append('/mapping/backend/export_artifact')
        if require_publishers:
            # Live mode: the driver must already publish both raw streams,
            # otherwise FAST-LIO2 would wait forever with no visible error.
            for topic in (lidar, imu):
                if not node.get_publishers_info_by_topic(topic):
                    missing.append(f'publisher on {topic} (livox_ros_driver2 not up or sensor unreachable)')
        services = dict(node.get_service_names_and_types())
        if reference_mode == 'pgo' and 'interface/srv/SaveMaps' not in services.get('/pgo/save_maps', []):
            missing.append('/pgo/save_maps')
        # Confirm the consumer chain exists BEFORE any recorded samples are published.
        required = {
            lidar: {'mid360_adapter_node'},
            imu: {'lio_node'},
            '/mapping/sensor/livox': {'lio_node'},
        }
        if reference_mode == 'fastlio':
            required['/mapping/frontend/cloud'] = {'fastlio_reference_exporter'}
            required['/mapping/frontend/odometry'] = {'fastlio_reference_exporter'}
        else:
            required['/mapping/frontend/cloud'] = {'pgo_node'}
            required['/mapping/frontend/odometry'] = {'pgo_node', 'pgo_backend_node'}
            required['/mapping/backend/status'] = {'mapping_artifact_exporter'}
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
    node.get_logger().info('[1/4] Pipeline ready; ' + (
        'live sensor streams are being consumed' if require_publishers else 'rosbag playback may start'))


def _export(rclpy, node, output):
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    reference_mode = node.declare_parameter('reference_mode', 'pgo').value
    if reference_mode not in ('pgo', 'fastlio'):
        raise ValueError('reference_mode must be pgo or fastlio')
    if reference_mode == 'pgo':
        from agt_mapping_artifacts.validation import ArtifactValidationError, verify_artifact
        verify_package = lambda deadline: verify_artifact(output, deadline=deadline)
        package_root = output / 'map_package'
    else:
        from .fastlio_reference import verify_fastlio_reference_package
        verify_package = lambda _deadline: verify_fastlio_reference_package(output / 'fastlio_reference_package')
        package_root = output / 'fastlio_reference_package'

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
                            'pgo_save_service_unavailable', 'frontend_stale_artifact_export_rejected',
                            'fastlio_reference_capture_failed', 'fastlio_reference_export_no_poses',
                            'fastlio_reference_export_failed'}:
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
    export_message = ('Requesting consistent FAST-LIO2-only reference export (PGO disabled)'
                      if reference_mode == 'fastlio' else 'Requesting optimized PGO export')
    mark_session(output, 'exporting', export_message)
    future = client.call_async(Trigger.Request())
    wait_until(future.done, spin, remaining(), 'export acknowledgement')
    response = future.result()
    if response is None or not response.success:
        backend_name = 'FAST-LIO2 reference exporter' if reference_mode == 'fastlio' else 'PGO'
        raise RuntimeError(f'{backend_name} rejected export: {getattr(response, "message", "no response")}')
    node.get_logger().info('[3/4] Export accepted; this is NOT yet a saved/verified map')
    mark_session(output, 'verifying', 'Export accepted; waiting for complete artifact and checksums')
    last_error = f'{package_root.name} not produced yet'
    while time.monotonic() < deadline:
        if failure:
            raise RuntimeError(f'Backend export failed: {failure[-1]}')
        if (package_root / 'checksums.sha256').is_file():
            try:
                result = verify_package(deadline)
            except Exception as exc:
                last_error = str(exc)
            else:
                root = package_root
                completed_message = ('FAST-LIO2 same-session reference verified' if reference_mode == 'fastlio'
                                     else 'Optimized PGO artifact verified')
                details = {'artifact_path': str(root), 'artifact_verified': True}
                if reference_mode == 'fastlio':
                    details.update(reference_type=result['reference_type'],
                                   keyframe_count=result['keyframes'], map_point_count=result['map_points'])
                mark_session(output, 'completed', completed_message, **details)
                node.get_logger().info(f'[4/4] Artifact VERIFIED: {root}; {result}')
                return
        spin(min(0.5, max(0.0, deadline - time.monotonic())))
    raise TimeoutError(f'No verified map within {timeout:g}s: {last_error}')


def wait_ready_main(args=None):
    return _run('wait_ready', _wait_ready, args)


def export_main(args=None):
    return _run('export_verified', _export, args)
