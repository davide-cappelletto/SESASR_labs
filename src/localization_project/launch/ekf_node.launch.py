from launch.launch_description import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    ExecuteProcess,
    RegisterEventHandler,
    DeclareLaunchArgument,
    EmitEvent,
    TimerAction,
    IncludeLaunchDescription
)
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.events import Shutdown
from launch.conditions import IfCondition

from launch.substitutions import (
    ThisLaunchFileDir,
    PathJoinSubstitution,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare("localization_project"),
                                   "config",
                                   "ekf_node.yaml"])
    ekf_node = Node(
        package="localization_project",
        executable="EKF_node",
        output="screen",
        parameters=[config],
    )

    timer_shutdown = TimerAction(
        period=10.0,
        actions=[
            EmitEvent(event=Shutdown(reason="Timer expired")),
        ],
    )

    rosbag_output_dir = DeclareLaunchArgument(
        'rosbag_output_dir',
        default_value='localization_project/rosbags',
        description='Directory where the rosbag should be saved'
    )

    rosbag_record_cmd = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-o", "data",
             "/diff_drive_controller/odom", "/ground_truth", "/ekf"],
        cwd=PathJoinSubstitution(
            ['/home/davide/sesasr_labs/src', LaunchConfiguration('rosbag_output_dir')]),
        output="screen",
    )

    on_shutdown_handler = RegisterEventHandler(event_handler=OnProcessExit(
        target_action=ekf_node,
        on_exit=[EmitEvent(event=Shutdown(reason="Timer expired"))]),
    )
    

    return LaunchDescription([ekf_node, rosbag_output_dir, rosbag_record_cmd, on_shutdown_handler, timer_shutdown])
