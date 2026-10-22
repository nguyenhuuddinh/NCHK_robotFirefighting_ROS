"""Launch observe-only YOLO and fire-alignment preview on the laptop."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_MODEL_PATH = (
    '/media/huudinh/New Volume/esp32/'
    'Fire_smoke_person_detection/best_final.pt'
)
DEFAULT_PYTHON_EXECUTABLE = (
    '/home/huudinh/.venvs/fire_robot_yolo/bin/python'
)


def _validate_paths(context):
    """Fail before node startup when config or model paths are invalid."""
    model_path = LaunchConfiguration('model_path').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    python_executable = LaunchConfiguration(
        'python_executable').perform(context)
    if not os.path.isfile(model_path):
        raise RuntimeError(f'YOLO model does not exist: {model_path}')
    if not os.path.isfile(params_file):
        raise RuntimeError(f'YOLO params file does not exist: {params_file}')
    if not os.path.isfile(python_executable) or \
            not os.access(python_executable, os.X_OK):
        raise RuntimeError(
            f'YOLO Python is missing or not executable: {python_executable}')
    # Import dependencies once in YoloObserver._load_model, where failures are
    # logged before any valid detection is published. A separate heavy import
    # here doubles initialization work and cannot guarantee CUDA/model readiness.
    return []


def generate_launch_description():
    """Create the standalone laptop perception launch description."""
    package_dir = get_package_share_directory('fire_robot_perception')
    default_params = os.path.join(package_dir, 'config', 'yolo_params.yaml')

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=DEFAULT_MODEL_PATH,
        description='Absolute path to the trusted Ultralytics .pt checkpoint',
    )
    python_executable_arg = DeclareLaunchArgument(
        'python_executable',
        default_value=DEFAULT_PYTHON_EXECUTABLE,
        description='Python interpreter containing torch and ultralytics',
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Absolute path to YOLO observer parameters',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time; false for the physical robot',
    )

    # The ament console-script wrapper is generated with /usr/bin/python3 on
    # this ROS 2 Humble install. Activating a venv does not change that
    # shebang. Run the module explicitly so inference always uses the GPU venv.
    observer = ExecuteProcess(
        cmd=[
            LaunchConfiguration('python_executable'),
            '-m',
            'fire_robot_perception.yolo_observer',
            '--ros-args',
            '-r',
            '__node:=yolo_observer',
            '--params-file',
            LaunchConfiguration('params_file'),
            '-p',
            ['model_path:=', LaunchConfiguration('model_path')],
            '-p',
            ['use_sim_time:=', LaunchConfiguration('use_sim_time')],
        ],
        output='screen',
    )

    alignment_observer = Node(
        package='fire_robot_perception',
        executable='fire_alignment_observer',
        name='fire_alignment_observer',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        model_path_arg,
        python_executable_arg,
        params_file_arg,
        use_sim_time_arg,
        OpaqueFunction(function=_validate_paths),
        observer,
        alignment_observer,
    ])
