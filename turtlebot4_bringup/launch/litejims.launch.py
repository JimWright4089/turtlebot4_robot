#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    pkg_turtlebot4_bringup = get_package_share_directory('turtlebot4_bringup')
    pkg_turtlebot4_diagnostics = get_package_share_directory('turtlebot4_diagnostics')

    param_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=PathJoinSubstitution(
            [pkg_turtlebot4_bringup, 'config', 'turtlebot4.yaml']),
        description='Turtlebot4 Robot param file'
    )

    turtlebot4_param_yaml_file = LaunchConfiguration('param_file')

    turtlebot4_robot_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_bringup, 'launch', 'robot.launch.py'])
    rplidar_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_bringup, 'launch', 'rplidarjims.launch.py'])
    oakd_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_bringup, 'launch', 'oakd.launch.py'])

    lite_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([turtlebot4_robot_launch_file]),
        launch_arguments=[('model', 'lite'),
                          ('param_file', turtlebot4_param_yaml_file)])
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rplidar_launch_file]))

    ld = LaunchDescription()
    ld.add_action(param_file_cmd)
    ld.add_action(lite_launch)
    ld.add_action(rplidar_launch)
    return ld
