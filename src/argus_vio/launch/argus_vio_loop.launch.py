"""Launch the VINS-Fusion stereo-inertial estimator + loop_fusion pose-graph.

Same estimator as argus_vio.launch.py, but ALSO starts the loop_fusion node so
loop closures correct accumulated drift on revisits (long-path / Scenario C).

Wiring: vins_node publishes its keyframe stream on GLOBAL names; loop_fusion
subscribes to /vins_estimator/* names, so loop_fusion's subs are remapped onto
what vins actually publishes:

  loop_fusion sub                    <- vins publishes
  /vins_estimator/odometry           <- /argus/vio/odom_optimized (vins /odometry remap)
  /vins_estimator/keyframe_pose      <- /keyframe_pose
  /vins_estimator/keyframe_point     <- /keyframe_point
  /vins_estimator/extrinsic          <- /extrinsic
  /vins_estimator/margin_cloud       <- /argus/vio/margin_cloud
  (image)                            <- /argus/cam0/image_raw (config image0_topic)

loop_fusion outputs, remapped onto the ARGUS schema:
  odometry_rect    -> /argus/vio/odom_loop      (loop-CORRECTED odometry; eval this)
  pose_graph_path  -> /argus/vio/loop_closures  (corrected pose-graph path, schema topic)
  base_path        -> /argus/vio/base_path      (un-corrected path, for before/after)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("argus_vio")
    default_config = os.path.join(share, "config", "argus_stereo_imu_config.yaml")

    config = LaunchConfiguration("config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    vins_remaps = [
        ("/imu_propagate", "/argus/vio/odom"),
        ("/odometry", "/argus/vio/odom_optimized"),
        ("/path", "/argus/vio/path"),
        ("/camera_pose", "/argus/vio/camera_pose"),
        ("/point_cloud", "/argus/vio/point_cloud"),
        ("/margin_cloud", "/argus/vio/margin_cloud"),
        ("/key_poses", "/argus/vio/key_poses"),
        ("/image_track", "/argus/vio/image_track"),
    ]

    loop_remaps = [
        ("/vins_estimator/odometry", "/argus/vio/odom_optimized"),
        ("/vins_estimator/keyframe_pose", "/keyframe_pose"),
        ("/vins_estimator/keyframe_point", "/keyframe_point"),
        ("/vins_estimator/extrinsic", "/extrinsic"),
        ("/vins_estimator/margin_cloud", "/argus/vio/margin_cloud"),
        ("odometry_rect", "/argus/vio/odom_loop"),
        ("pose_graph_path", "/argus/vio/loop_closures"),
        ("base_path", "/argus/vio/base_path"),
    ]

    return LaunchDescription(
        [
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="vins",
                executable="vins_node",
                name="vins_estimator",
                output="screen",
                arguments=[config],
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=vins_remaps,
            ),
            Node(
                package="loop_fusion",
                executable="loop_fusion_node",
                name="loop_fusion",
                output="screen",
                arguments=[config],
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=loop_remaps,
            ),
        ]
    )
