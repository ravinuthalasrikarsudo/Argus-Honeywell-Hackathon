"""ARGUS :: superpoint.launch.py

Launch the standalone SuperPoint front-end via the GPU venv interpreter
(onnxruntime-gpu). A plain Node action would use the colcon-selected system
python3 (no onnxruntime), so we ExecuteProcess the venv python on the installed
node module instead. Prefer `bash scripts/run_superpoint.sh` for interactive use;
this launch file exists for composition with the rest of the stack.
"""
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    home = os.path.expanduser('~')
    venv_py = os.path.join(home, '.venvs', 'argus-sp', 'bin', 'python')
    ws = os.path.join(home, 'argus')

    image_topic = LaunchConfiguration('image_topic')
    model_path = LaunchConfiguration('model_path')

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/argus/cam0/image_raw'),
        DeclareLaunchArgument(
            'model_path',
            default_value=os.path.join(ws, 'models', 'superpoint', 'superpoint_1024.onnx')),
        ExecuteProcess(
            cmd=[venv_py, '-m', 'argus_superpoint.superpoint_node', '--ros-args',
                 '-p', 'use_sim_time:=true',
                 '-p', ['image_topic:=', image_topic],
                 '-p', ['model_path:=', model_path]],
            additional_env={
                'PYTHONPATH': os.path.join(ws, 'src', 'argus_superpoint')
                + os.pathsep + os.environ.get('PYTHONPATH', ''),
            },
            output='screen',
        ),
    ])
