#!/usr/bin/env python3
# robot_perception: 彩色仿真场景启动文件
# 启动 Gazebo(彩色世界) + TurtleBot3 机器人

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL')
    if TURTLEBOT3_MODEL is None:
        raise RuntimeError(
            '环境变量 TURTLEBOT3_MODEL 未设置，请先执行: export TURTLEBOT3_MODEL=waffle')

    pkg_robot_perception = get_package_share_directory('robot_perception')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    default_world = os.path.join(
        pkg_robot_perception, 'worlds', 'colored_world.world')

    # 让 gazebo 能找到 turtlebot3 的模型文件（mesh），并保留用户已有的模型路径
    gazebo_model_path = os.path.join(pkg_turtlebot3_gazebo, 'models')
    gazebo_model_path = gazebo_model_path + ':' + os.path.join(
        pkg_robot_perception, 'models')
    if os.environ.get('GAZEBO_MODEL_PATH'):
        gazebo_model_path = gazebo_model_path + ':' + os.environ['GAZEBO_MODEL_PATH']

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=default_world)
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch',
                         'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch',
                         'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
        }.items()
    )

    return LaunchDescription([
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH',
                               value=gazebo_model_path),

        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='要加载的 world 文件完整路径'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='是否使用仿真时钟'),
        DeclareLaunchArgument(
            'x_pose',
            default_value='-2.0',
            description='机器人出生点 x 坐标'),
        DeclareLaunchArgument(
            'y_pose',
            default_value='-0.5',
            description='机器人出生点 y 坐标'),

        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
    ])
