"""Live-session finish control: a Trigger service plus an optional duration.

Exits 0 when the operator requests the end of recording (service call, stop
file, or --duration elapsed). The launch composition reacts by stopping the
raw rosbag recorder and then running the verified export, so the live path
finishes through exactly the same finalizer as an offline replay.
"""
from pathlib import Path
import time

from .session_state import mark_session

STOP_FILE_NAME = 'STOP_MAPPING'
FINISH_SERVICE = '/mapping/session/finish'


def _supervise(rclpy, node, output):
    from std_srvs.srv import Trigger
    from sensor_msgs.msg import Imu

    duration = float(node.declare_parameter('duration_seconds', 0.0).value)
    stall_timeout = float(node.declare_parameter('sensor_stall_seconds', 5.0).value)
    imu_topic = node.declare_parameter('imu_topic', '/livox/imu').value
    finish = []
    last_imu = [time.monotonic()]
    start = time.monotonic()

    def on_finish(_request, response):
        finish.append('service')
        response.success = True
        response.message = 'Finishing live mapping session; raw bag will be closed and exported'
        return response

    node.create_service(Trigger, FINISH_SERVICE, on_finish)
    node.create_subscription(Imu, imu_topic, lambda _: last_imu.__setitem__(0, time.monotonic()), 50)
    stop_file = Path(output) / STOP_FILE_NAME
    mark_session(output, 'recording',
                 f'Live MID360 mapping; finish with: ros2 service call {FINISH_SERVICE} std_srvs/srv/Trigger "{{}}" '
                 f'or touch {stop_file}')
    node.get_logger().info(f'[2/4] Live mapping. Finish: ros2 service call {FINISH_SERVICE} '
                           f'std_srvs/srv/Trigger "{{}}"  (or touch {stop_file}). Ctrl+C cancels WITHOUT export.')
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        now = time.monotonic()
        if finish:
            reason = 'operator service call'
            break
        if stop_file.exists():
            reason = 'stop file'
            break
        if duration > 0 and now - start >= duration:
            reason = f'{duration:g}s duration elapsed'
            break
        if stall_timeout > 0 and now - last_imu[0] > stall_timeout:
            raise RuntimeError(f'No IMU samples on {imu_topic} for {stall_timeout:g}s: sensor stalled or '
                               'network dropped; live session aborted without export')
    else:
        raise InterruptedError('ROS context stopped')
    mark_session(output, 'finishing', f'Live capture finished ({reason}); closing raw bag')
    node.get_logger().info(f'[2/4] Finish requested ({reason}); closing raw bag and exporting')
    return 0


def supervisor_main(args=None):
    from .session_runtime import _run
    return _run('live_supervisor', _supervise, args)
