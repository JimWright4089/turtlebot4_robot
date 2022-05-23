from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
]


def generate_launch_description():

    rplidar_standard_stf = Node(
            name='rplidar_standard_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['-0.04816', '0', '0.14', '1.5707', '0.0', '0.0', 'base_link', 'laser'],
            condition=LaunchConfigurationEquals('model', 'standard')
        )

    rplidar_lite_stf = Node(
            name='rplidar_lite_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0.07', '1.5707', '0.0', '0.0', 'base_link', 'laser'],
            condition=LaunchConfigurationEquals('model', 'lite')
        )

    rplidar_node = Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',        
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 256000,  # A3
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Sensitivity',
            }],
        )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(rplidar_lite_stf)
    ld.add_action(rplidar_standard_stf)
    ld.add_action(rplidar_node)

    return ld
