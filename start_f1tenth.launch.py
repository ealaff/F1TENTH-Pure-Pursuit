from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    workspace = Path.home() / 'f1tenth_ws'

    # F1TENTH simulator
    simulator_launch = (
        Path(get_package_share_directory('f1tenth_gym_ros'))
        / 'launch'
        / 'gym_bridge_launch.py'
    )

    # RViz configuration
    rviz_config = (
        workspace / 'src' / 'f1tenth_gym_ros'
        / 'config' / 'rviz' / 'gym_bridge.rviz'
    )

    # Pure Pursuit controller
    controller_script = workspace / 'pure_pursuit_f1tenth.py'
    python_executable = workspace / '.venv' / 'bin' / 'python3'

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(simulator_launch)),
        launch_arguments={
            'num_agent': '1',
            'open_foxglove': 'false'
        }.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', str(rviz_config)],
        output='screen'
    )

    controller = ExecuteProcess(
        cmd=[
            str(python_executable),
            str(controller_script)
        ],
        output='screen'
    )

    return LaunchDescription([
        simulator,
        rviz,
        TimerAction(
            period=3.0,
            actions=[controller]
        )
    ])