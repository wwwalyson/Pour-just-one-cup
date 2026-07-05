from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():


    return LaunchDescription([
        DeclareLaunchArgument('usart_port_name', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baud_rate', default_value='9600'),
        DeclareLaunchArgument('gripper_servo_id', default_value='1'),
        DeclareLaunchArgument('target_x', default_value='15.0'),
        DeclareLaunchArgument('target_y', default_value='0.0'),
        DeclareLaunchArgument('target_z', default_value='2.0'),

        Node(
            package='arm_base',
            executable='arm_base',
            name='arm_base',
            output='screen',
            parameters=[{
                'usart_port_name': LaunchConfiguration('usart_port_name'),
                'serial_baud_rate': LaunchConfiguration('serial_baud_rate'),
                'gripper_servo_id': LaunchConfiguration('gripper_servo_id'),
            }],
        ),

        Node(
            package='arm_cmd',
            executable='arm_cmd',
            name='arm_cmd',
            output='screen',
            parameters=[{
                'target_x': LaunchConfiguration('target_x'),
                'target_y': LaunchConfiguration('target_y'),
                'target_z': LaunchConfiguration('target_z'),
            }],
        ),
    ])
