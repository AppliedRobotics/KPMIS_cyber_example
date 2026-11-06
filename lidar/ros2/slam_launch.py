"""Лидар YDLIDAR X3 Pro + одометрия rf2o + slam_toolbox + RViz2.

Запуск:   ros2 launch ~/ydlidar_ros2_ws/slam_launch.py
Без RViz: ros2 launch ~/ydlidar_ros2_ws/slam_launch.py rviz:=false
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

WS = os.path.expanduser('~/ydlidar_ros2_ws')
RVIZ_CFG = os.path.join(WS, 'slam.rviz')

def generate_launch_description():
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': os.path.join(WS, 'slam.yaml'),
                          'use_sim_time': 'false'}.items())

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        arguments=['-d', RVIZ_CFG] if os.path.exists(RVIZ_CFG) else [],
        additional_env={'DISPLAY': ':1',               # рабочий стол VNC
                        'LIBGL_ALWAYS_SOFTWARE': '1'},  # программная отрисовка 3D
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Запускать RViz2 (true/false)'),
        # драйвер лидара -> топик /scan
        Node(package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
             name='ydlidar_ros2_driver_node', output='screen',
             parameters=[os.path.join(WS, 'x3pro.yaml')]),
        # положение лидара на роботе: base_link -> laser_frame
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_laser',
             arguments=['--frame-id', 'base_link', '--child-frame-id', 'laser_frame']),
        # одометрия по сканам: odom -> base_link
        Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
             name='rf2o_laser_odometry', output='screen',
             parameters=[os.path.join(WS, 'rf2o.yaml')],
             arguments=['--ros-args', '--log-level', 'warn']),
        # SLAM: карта /map и map -> odom (через 3 с, когда лидар раскрутится)
        TimerAction(period=3.0, actions=[slam]),
        # RViz2 (через 5 с)
        TimerAction(period=5.0, actions=[rviz]),
    ])
