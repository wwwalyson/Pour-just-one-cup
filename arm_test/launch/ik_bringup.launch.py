"""
IK pipeline launch: arm_base (serial) + ik_node (ROS2-side IK).

Bypasses STM32's closed-source LeArm.lib — sends absolute servo
positions via CMD_MULT_SERVO_MOVE, eliminating coordinate drift.
Pitch angle is auto-selected from the valid range for each target.

Usage:
    ros2 launch arm_test ik_bringup.launch.py target_x:=15.0
    ros2 launch arm_test ik_bringup.launch.py preferred_pitch:=-90.0  # top-down grasp
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    preferred_pitch = LaunchConfiguration('preferred_pitch', default='-50.0')
    target_x = LaunchConfiguration('target_x', default='15.0')
    target_y = LaunchConfiguration('target_y', default='0.0')
    target_z = LaunchConfiguration('target_z', default='2.0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'preferred_pitch', default_value='-50.0',
            description='Preferred pitch angle (deg). -50=natural, -90=top-down'),
        DeclareLaunchArgument('target_x', default_value='15.0'),
        DeclareLaunchArgument('target_y', default_value='0.0'),
        DeclareLaunchArgument('target_z', default_value='2.0'),

        # Serial communication bridge (IK mode: only /joint_target, no old paths)
        Node(
            package='arm_base',
            executable='arm_base',
            name='arm_base',
            output='screen',
            parameters=[{
                'ik_mode': True,
                'move_time_ms': 1500,
            }],
        ),

        # IK solver: /arm_cmd → auto-pitch IK → /joint_target
        Node(
            package='arm_test',
            executable='ik_node.py',
            name='ik_solver',
            output='screen',
            parameters=[{'preferred_pitch': preferred_pitch}],
        ),

        # Absolute coordinate publisher (ik_mode bypasses STM32 accumulation)
        Node(
            package='arm_cmd',
            executable='arm_cmd',
            name='arm_cmd',
            output='screen',
            parameters=[{
                'target_x': target_x,
                'target_y': target_y,
                'target_z': target_z,
                'ik_mode': True,
            }],
        ),
    ])
