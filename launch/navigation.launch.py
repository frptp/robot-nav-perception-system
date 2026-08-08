#!/usr/bin/env python3
# robot_perception: Nav2 自主导航
# 自动启动仿真 + Nav2(定位+规划控制) + RViz

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL')
    if TURTLEBOT3_MODEL is None:
        raise RuntimeError(
            '环境变量 TURTLEBOT3_MODEL 未设置，请先执行: export TURTLEBOT3_MODEL=waffle')

    ROS_DISTRO = os.environ.get('ROS_DISTRO')
    pkg_robot_perception = get_package_share_directory('robot_perception')
    pkg_turtlebot3_navigation2 = get_package_share_directory(
        'turtlebot3_navigation2')

    default_world = os.path.join(
        pkg_robot_perception, 'worlds', 'colored_world.world')
    default_map = os.path.join(
        pkg_robot_perception, 'maps', 'my_map_colored.yaml')

    if ROS_DISTRO == 'humble':
        default_params = os.path.join(
            pkg_turtlebot3_navigation2, 'param', 'humble',
            TURTLEBOT3_MODEL + '.yaml')
    else:
        default_params = os.path.join(
            pkg_turtlebot3_navigation2, 'param',
            TURTLEBOT3_MODEL + '.yaml')

    default_rviz_config = os.path.join(
        pkg_turtlebot3_navigation2, 'rviz', 'tb3_navigation2.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=default_world)
    map_file = LaunchConfiguration('map', default=default_map)
    params_file = LaunchConfiguration(
        'params_file', default=default_params)

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

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
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
            'map',
            default_value=default_map,
            description='Nav2 定位使用的地图 yaml 完整路径'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Nav2 参数文件完整路径'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='是否使用仿真时钟'),

        simulation_cmd,
        nav2_cmd,
        rviz_cmd,
    ])
