"""One MID360 mapping sensor owner for YHS; no robot hardware launch or base TF."""


def sensor_parameters(source, frame_id):
    """Same CustomMsg mode as mapping_custom; source was statically validated."""
    return {
        'xfer_format': 1,
        'multi_topic': 0,
        'data_src': 0,
        'publish_freq': float(source.publish_freq),
        'output_data_type': 0,
        'frame_id': str(frame_id),
        'lvx_file_path': '',
        'user_config_path': str(source.path),
        'cmdline_input_bd_code': 'livox0000000001',
    }


def make_sensor_node(source, frame_id):
    """Load launch types only when a supervised live session is actually started."""
    from launch_ros.actions import Node
    from launch_ros.parameter_descriptions import ParameterValue

    params = sensor_parameters(source, frame_id)
    params['frame_id'] = ParameterValue(params['frame_id'], value_type=str)
    params['user_config_path'] = ParameterValue(params['user_config_path'], value_type=str)
    return Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
                name='livox_lidar_publisher', output='screen', parameters=[params])
