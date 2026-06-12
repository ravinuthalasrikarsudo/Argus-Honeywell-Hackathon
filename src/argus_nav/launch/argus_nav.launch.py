"""Launch the ARGUS navigation pillar: dense stereo depth + reactive avoidance.

Brings up the ``argus_nav`` nodes side by side on top of a running sim/stack:

  stereo_depth     -- /argus/cam{0,1}/image_raw -> /argus/depth/{image,points}
  reactive_avoider -- /argus/depth/points + /argus/lidar/points (+ VIO pose)
                      -> /argus/cmd_vel  (potential-field planner)
  occupancy_mapper -- /argus/depth/points (+ VIO pose) -> /argus/map/points
                      (log-odds temporal fusion: clean, dynamic terrain map)

Consumes only public contract topics (+ the additive /argus/lidar/* sensor)
and publishes the additive /argus/depth/*, /argus/map/* and /argus/nav/* topics
(see docs/CONTRACT.md sec 8). Nothing here remaps or changes a frozen interface.

Args:
  use_sim_time  (true)  -- follow /clock from the simulator (always true here).
  decimation    (2)     -- SGBM input downscale; 2 keeps RTF in the iGPU budget.
  goal_x/y/z            -- world ENU goal the avoider drives toward.
  max_speed     (0.8)   -- m/s, the project flight envelope.
  pose_type     (odometry) -- 'odometry' (VIO, GPS-free) or 'pose' (PoseStamped).
  pose_topic    (/argus/vio/odom) -- pose source; pair with pose_type.
  use_lidar     (true)  -- fuse the 3D LiDAR cloud into the avoider obstacle field.
  enable_depth / enable_avoider / enable_mapper (true) -- toggle nodes for ablations.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    decimation = LaunchConfiguration("decimation")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    goal_z = LaunchConfiguration("goal_z")
    max_speed = LaunchConfiguration("max_speed")
    pose_type = LaunchConfiguration("pose_type")
    pose_topic = LaunchConfiguration("pose_topic")
    map_pose_type = LaunchConfiguration("map_pose_type")
    map_pose_topic = LaunchConfiguration("map_pose_topic")
    use_lidar = LaunchConfiguration("use_lidar")
    enable_depth = LaunchConfiguration("enable_depth")
    enable_avoider = LaunchConfiguration("enable_avoider")
    enable_mapper = LaunchConfiguration("enable_mapper")

    return LaunchDescription(
        [
            # Pin DDS to Cyclone to match the rest of the ARGUS stack (deviation #7).
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use /clock from the simulator (always true for ARGUS).",
            ),
            DeclareLaunchArgument(
                "decimation",
                default_value="2",
                description="SGBM input downscale factor (1280x720 -> 640x360 at 2).",
            ),
            DeclareLaunchArgument("goal_x", default_value="28.0"),
            DeclareLaunchArgument("goal_y", default_value="0.0"),
            DeclareLaunchArgument("goal_z", default_value="1.2"),
            DeclareLaunchArgument(
                "max_speed",
                default_value="0.8",
                description="Planner speed cap (m/s); the project flight envelope.",
            ),
            DeclareLaunchArgument(
                "pose_type",
                default_value="odometry",
                description="Pose source type for avoider+mapper: 'odometry' (VIO, "
                            "GPS-free, default) or 'pose' (PoseStamped, e.g. ground truth).",
            ),
            DeclareLaunchArgument(
                "pose_topic",
                default_value="/argus/vio/odom",
                description="Pose topic; pair with pose_type. Use /argus/ground_truth/pose "
                            "+ pose_type:=pose for an A/B baseline.",
            ),
            # Pose source for the MAP/trajectory only -- decoupled from the avoider so
            # the map can be rendered from a non-drifting source (ground truth) while
            # flight control stays GPS-free on VIO. Default mirrors pose_type/pose_topic
            # (= avoider source) so nothing changes unless these are set explicitly.
            DeclareLaunchArgument("map_pose_type", default_value=pose_type),
            DeclareLaunchArgument("map_pose_topic", default_value=pose_topic),
            DeclareLaunchArgument("use_lidar", default_value="true"),
            DeclareLaunchArgument("enable_depth", default_value="true"),
            DeclareLaunchArgument("enable_avoider", default_value="true"),
            DeclareLaunchArgument("enable_mapper", default_value="true"),
            Node(
                package="argus_nav",
                executable="stereo_depth",
                name="stereo_depth",
                output="screen",
                condition=IfCondition(enable_depth),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "decimation": decimation,
                    }
                ],
            ),
            Node(
                package="argus_nav",
                executable="reactive_avoider",
                name="reactive_avoider",
                output="screen",
                condition=IfCondition(enable_avoider),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "goal_x": goal_x,
                        "goal_y": goal_y,
                        "goal_z": goal_z,
                        "max_speed": max_speed,
                        "pose_type": pose_type,
                        "pose_topic": pose_topic,
                        "use_lidar": use_lidar,
                    }
                ],
            ),
            Node(
                package="argus_nav",
                executable="occupancy_mapper",
                name="occupancy_mapper",
                output="screen",
                condition=IfCondition(enable_mapper),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "pose_type": map_pose_type,
                        "pose_topic": map_pose_topic,
                    }
                ],
            ),
        ]
    )
