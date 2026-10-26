"""Private planner-only process group. No TF, localization, controller or base."""
import json
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _servers(context):
    from agt_coverage_authoring.io import load_robot_profile
    def value(name):
        return LaunchConfiguration(name).perform(context)
    profile = load_robot_profile(value("platform_profile"))
    width, timeout = float(value("operation_width")), float(value("connection_timeout"))
    if not math.isfinite(width) or width <= 0 or not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("operation_width and connection_timeout must be finite and positive")
    namespace = value("namespace").strip("/")
    if not namespace:
        raise ValueError("Offline planner namespace cannot be root")
    forward, reverse = value("forward_planner"), value("reverse_planner")
    if forward == reverse:
        raise ValueError("Forward and reversible planners must have distinct IDs")
    # Collision geometry in the actual planning reference frame, without a
    # second implicit margin. Final authoring validation adds explicit margin.
    dx,dy = profile.rear_axle_to_base
    footprint = json.dumps([[x+dx,y+dy] for x,y in profile.footprint])
    planner = {"use_sim_time":False,"planner_plugins":[forward,reverse],"expected_planner_frequency":.1}
    for name,model in ((forward,"DUBIN"),(reverse,"REEDS_SHEPP")):
        planner.update({
            name+".plugin":"nav2_smac_planner/SmacPlannerHybrid",
            name+".motion_model_for_search":model,
            name+".minimum_turning_radius":profile.min_turning_radius,
            name+".allow_unknown":False,
            name+".tolerance":0.0,
            name+".downsample_costmap":False,
            name+".angle_quantization_bins":72,
            name+".max_iterations":1000000,
            name+".max_planning_time":timeout,
            name+".analytic_expansion_max_length":max(6.,4.*profile.min_turning_radius),
            name+".reverse_penalty":2., name+".change_penalty":.2,
            name+".smooth_path":False,
        })
    # Costmap parameters belong to the planner's child node. A parameter dict
    # scoped to '/**' is supported by rcl YAML; use RewrittenYaml-free explicit
    # temporary file so the exact namespace is unambiguous.
    import os
    import tempfile
    import yaml
    costmap = {
        "use_sim_time":False,"global_frame":"map","robot_base_frame":"map",
        "update_frequency":1.,"publish_frequency":1.,"rolling_window":False,
        "track_unknown_space":True,"footprint":footprint,"footprint_padding":0.,
        "plugins":["static_layer"],"always_send_full_costmap":True,
        "static_layer.plugin":"nav2_costmap_2d::StaticLayer",
        "static_layer.map_topic":"/"+namespace+"/map",
        "static_layer.map_subscribe_transient_local":True,
    }
    descriptor,path = tempfile.mkstemp(prefix="agt_coverage_costmap_",suffix=".yaml")
    with os.fdopen(descriptor,"w",encoding="utf-8") as stream:
        yaml.safe_dump({"/"+namespace+"/global_costmap/global_costmap":{"ros__parameters":costmap}},stream)
    # launch owns this temporary parameter file; cleanup on process exit also
    # covers exceptions without altering any map or user configuration.
    import atexit
    atexit.register(lambda: os.path.exists(path) and os.unlink(path))
    return [
        Node(package="nav2_map_server",executable="map_server",name="map_server",namespace=namespace,
             parameters=[{"yaml_filename":value("planning_map"),"use_sim_time":False}],output="screen"),
        Node(package="nav2_planner",executable="planner_server",name="planner_server",namespace=namespace,
             parameters=[planner,path],output="screen"),
        Node(package="opennav_coverage",executable="opennav_coverage",name="coverage_server",namespace=namespace,
             parameters=[{"use_sim_time":False,"coordinates_in_cartesian_frame":True,
                          "robot_width":max(y for _,y in profile.footprint)-min(y for _,y in profile.footprint),
                          "operation_width":width,"min_turning_radius":profile.min_turning_radius}],output="screen"),
        Node(package="nav2_lifecycle_manager",executable="lifecycle_manager",name="offline_lifecycle_manager",namespace=namespace,
             parameters=[{"use_sim_time":False,"autostart":True,
                          "node_names":["map_server","planner_server","coverage_server"]}],output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("planning_map"),DeclareLaunchArgument("platform_profile"),
        DeclareLaunchArgument("operation_width",default_value="1.0"),
        DeclareLaunchArgument("namespace",default_value="agt_coverage_offline"),
        DeclareLaunchArgument("connection_timeout",default_value="15.0"),
        DeclareLaunchArgument("forward_planner",default_value="CoverageForwardHybrid"),
        DeclareLaunchArgument("reverse_planner",default_value="CoverageRepairHybrid"),
        OpaqueFunction(function=_servers),
    ])
