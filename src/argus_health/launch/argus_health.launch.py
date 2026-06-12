"""Launch the ARGUS VIO health monitor (Pillar 3).

Runs the ``health_monitor`` node alongside a live VINS-Fusion stack. It consumes
the ``/argus/vio/*`` topics and publishes ``/argus/vio/health`` plus the recovery
signal. Pure rclpy node -> runs under the colcon-selected system interpreter
(unlike the SuperPoint node, no venv/onnxruntime needed).

Args:
  use_sim_time    (true)  -- follow /clock from the simulator (always true here).
  enable_recovery (true)  -- arm the LOST-triggered hold/flag recovery (C3); set
                             false for the C1 baseline ablation cell.
  recovery_hold_cmd (true)-- while recovering, publish a zero-velocity /argus/cmd_vel
                             hold so the drone stops dead-reckoning in the dark.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_recovery = LaunchConfiguration("enable_recovery")
    recovery_hold_cmd = LaunchConfiguration("recovery_hold_cmd")

    return LaunchDescription(
        [
            # Pin Cyclone to match the rest of the ARGUS stack (deviation #7).
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("enable_recovery", default_value="true"),
            DeclareLaunchArgument("recovery_hold_cmd", default_value="true"),
            Node(
                package="argus_health",
                executable="health_monitor",
                name="argus_health_monitor",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "enable_recovery": enable_recovery,
                        "recovery_hold_cmd": recovery_hold_cmd,
                    }
                ],
            ),
        ]
    )
