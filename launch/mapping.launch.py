#!/usr/bin/env python3
# robot_perception: SLAM 建图
# 自动启动仿真 + slam_toolbox + RViz

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_robot_perception = get_package_share_directory('robot_perception')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    default_world = os.path.join(
        pkg_robot_perception, 'worlds', 'colored_world.world')
    default_slam_params = os.path.join(
        pkg_slam_toolbox, 'config', 'mapper_params_online_async.yaml')
    default_rviz_config = os.path.join(
        pkg_slam_toolbox, 'config', 'slam_toolbox_default.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=default_world)
    slam_params_file = LaunchConfiguration(
        'slam_params_file', default=default_slam_params)

    simulation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_perception, 'launch',
                         'simulation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
        }.items()
    )

    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch',
                         'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items()
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='要加载的 world 文件完整路径'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='是否使用仿真时钟'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_slam_params,
            description='slam_toolbox 参数文件完整路径'),

        simulation_cmd,
        slam_cmd,
        rviz_cmd,
    ])
