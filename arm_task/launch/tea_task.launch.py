"""
Launch tea task pipeline: IK pipeline + tea task node.

The tea_task_node subscribes to /target_grasp_array from YOLO (running
on RDK X5) and orchestrates arm pick-and-place sequences.

Usage:
    ros2 launch arm_task tea_task.launch.py
    ros2 launch arm_task tea_task.launch.py move_time_ms:=3000
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    move_time_ms = LaunchConfiguration('move_time_ms', default='1500')

    return LaunchDescription([
        DeclareLaunchArgument(
            'move_time_ms', default_value='1500',
            description='Servo move time in ms (default 1500)'),
        DeclareLaunchArgument(
            'preferred_pitch', default_value='-50.0',
            description='Preferred pitch angle (deg)'),

        # Serial bridge (IK mode)
        Node(
            package='arm_base',
            executable='arm_base',
            name='arm_base',
            output='screen',
            parameters=[{
                'ik_mode': True,
                'move_time_ms': move_time_ms,
            }],
        ),

        # IK solver
        Node(
            package='arm_test',
            executable='ik_node.py',
            name='ik_solver',
            output='screen',
            parameters=[{'preferred_pitch': -50.0}],
        ),

        # Tea task orchestrator
        Node(
            package='arm_task',
            executable='tea_task_node',
            name='tea_task_node',
            output='screen',
        ),
    ])
