#!/usr/bin/env python3
"""ARGUS top-level bringup launch.

Co-starts the full simulation stack as one shot:

  1. Gazebo Harmonic running the ``warehouse_corridor`` world.
  2. The kinematic stereo/IMU ``argus_drone`` spawned at the frozen
     start pose (1.5, 0, 1.0).
  3. ``ros_gz_bridge parameter_bridge`` driven by ``argus_bridge.yaml``.
  4. The ``camera_info_patch`` node that fixes the right-camera
     projection term (deviation #3).

Both the bridge AND the patch are started here on purpose: without the patch the
right CameraInfo keeps P[3] = 0 and downstream stereo is wrong (see the frozen
contract). All ROS nodes run with ``use_sim_time`` so they follow the bridged
``/clock``. The RMW is pinned to CycloneDDS to match the rest of the workspace.

The world resolves by filename and the drone model by path through the
``argus_sim`` GZ_SIM_RESOURCE_PATH hook, so this launch only works once
``install/setup.bash`` has been sourced.

Args:
  world          world name (file ``<world>.sdf`` on GZ_SIM_RESOURCE_PATH).
  headless       true => ``gz sim -s`` (server only, sensors still render
                 offscreen on ogre2); false => GUI.
  use_sim_time   propagate sim time to every ROS node (default true).
  spawn_delay    seconds to wait for the gz server before spawning the drone.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Frozen drone start pose (matches the contract; spawned, not baked into world).
SPAWN_X = '1.5'
SPAWN_Y = '0.0'
SPAWN_Z = '1.0'


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() in ('true', '1')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1')
    spawn_delay = float(LaunchConfiguration('spawn_delay').perform(context))

    sim_share = get_package_share_directory('argus_sim')
    bringup_share = get_package_share_directory('argus_bringup')

    drone_model = os.path.join(sim_share, 'models', 'argus_drone', 'model.sdf')
    bridge_config = os.path.join(bringup_share, 'config', 'argus_bridge.yaml')

    # `-r` runs unpaused; `-s` is server-only (ogre2 sensors still render
    # offscreen). The world file is found on GZ_SIM_RESOURCE_PATH by name.
    gz_args = '{server}-r -v 4 {world}.sdf'.format(
        server='-s ' if headless else '', world=world)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': gz_args,
            'gz_version': '8',
            # Tearing down gz tears down the whole launch.
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # Spawn the drone via ros_gz_sim `create`. The model is pure primitives, so
    # loading the SDF file directly is self-contained (no model:// resolution).
    # Delayed so the gz server's /world/<world>/create service is up first.
    spawn_drone = TimerAction(
        period=spawn_delay,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_argus_drone',
            output='screen',
            arguments=[
                '-world', world,
                '-file', drone_model,
                '-name', 'argus_drone',
                '-x', SPAWN_X, '-y', SPAWN_Y, '-z', SPAWN_Z,
                '-allow_renaming', 'false',
            ],
        )],
    )

    # ROS <-> gz bridge. use_sim_time keeps it on the sim clock.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='argus_bridge',
        output='screen',
        parameters=[
            {'config_file': bridge_config},
            {'use_sim_time': use_sim_time},
        ],
    )

    # Right-camera baseline patch (deviation #3): republishes /argus/cam1/camera_info
    # with P[3] = -fx*baseline. Contract defaults live in the node itself.
    camera_info_patch = Node(
        package='argus_bringup',
        executable='camera_info_patch',
        name='camera_info_patch',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return [gz_sim, spawn_drone, bridge, camera_info_patch]


def generate_launch_description():
    return LaunchDescription([
        # Pin the middleware to match the workspace (Cyclone). gz-transport is
        # separate and unaffected; this only governs the ROS side.
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),

        DeclareLaunchArgument(
            'world', default_value='warehouse_corridor',
            description='World name; file <world>.sdf must be on GZ_SIM_RESOURCE_PATH.'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true => gz sim -s (server only); false => GUI.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Run all ROS nodes on the bridged /clock.'),
        DeclareLaunchArgument(
            'spawn_delay', default_value='4.0',
            description='Seconds to wait for the gz server before spawning the drone.'),

        OpaqueFunction(function=launch_setup),
    ])
