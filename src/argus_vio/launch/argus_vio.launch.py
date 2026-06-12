"""Launch the VINS-Fusion stereo-inertial estimator against ARGUS topics.

Starts the upstream ``vins_node`` (third_party/VINS-Fusion-ROS2) with the ARGUS
config and remaps its outputs onto the frozen ``/argus/vio/*`` schema:

  /odometry      -> /argus/vio/odom   (nav_msgs/Odometry, world frame)
  /path          -> /argus/vio/path   (nav_msgs/Path,     world frame)

The estimator's *input* topics (IMU + stereo) are driven by the config file
(imu_topic / image0_topic / image1_topic), not by remaps.
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

    return LaunchDescription(
        [
            # Pin DDS to Cyclone (Fast-DDS has known WSL issues; matches D1 stack).
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Path to the VINS-Fusion config YAML.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use /clock from the simulator (always true for ARGUS).",
            ),
            Node(
                package="vins",
                executable="vins_node",
                name="vins_estimator",
                output="screen",
                # vins_node reads the config path as its single positional arg.
                arguments=[config],
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=[
                    # /argus/vio/odom = high-rate (IMU-rate ~100 Hz) real-time
                    # odometry -> satisfies the 30 Hz schema; the image-rate
                    # optimized estimate is image-rate-limited (~13 Hz on the
                    # iGPU) so it goes on a secondary topic for drift analysis.
                    ("/imu_propagate", "/argus/vio/odom"),
                    ("/odometry", "/argus/vio/odom_optimized"),
                    ("/path", "/argus/vio/path"),
                    ("/camera_pose", "/argus/vio/camera_pose"),
                    ("/point_cloud", "/argus/vio/point_cloud"),
                    ("/margin_cloud", "/argus/vio/margin_cloud"),
                    ("/key_poses", "/argus/vio/key_poses"),
                    ("/image_track", "/argus/vio/image_track"),
                ],
            ),
        ]
    )
