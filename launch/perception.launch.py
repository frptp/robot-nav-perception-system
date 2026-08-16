#!/usr/bin/env python3
# robot_perception: 感知-导航联动闭环
# 自动启动仿真 + Nav2 + 颜色检测 + 目标搜寻导航

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

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=default_world)
    map_file = LaunchConfiguration('map', default=default_map)
    params_file = LaunchConfiguration('params_file', default=default_params)
    target_mode = LaunchConfiguration('target_mode', default='color')
    model_path = LaunchConfiguration('model_path', default='/tmp/yolov8n.pt')

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_perception, 'launch',
                         'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            'map': map_file,
            'params_file': params_file,
        }.items()
    )

    color_detector_cmd = Node(
        package='robot_perception',
        executable='color_detector',
        name='color_detector',
        output='screen'
    )

    yolo_detector_cmd = Node(
        package='robot_perception',
        executable='yolo_detector',
        name='yolo_detector',
        parameters=[{'model_path': model_path}],
        output='screen'
    )

    target_navigator_cmd = Node(
        package='robot_perception',
        executable='target_navigator',
        name='target_navigator',
        parameters=[{'target_mode': target_mode}],
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
        DeclareLaunchArgument(
            'target_mode',
            default_value='color',
            description='目标搜寻模式: color(颜色) 或 person(YOLO追人)'),
        DeclareLaunchArgument(
            'model_path',
            default_value='/tmp/yolov8n.pt',
            description='YOLOv8 模型权重文件路径（target_mode=person 时需要）'),

        navigation_cmd,
        color_detector_cmd,
        yolo_detector_cmd,
        target_navigator_cmd,
    ])
