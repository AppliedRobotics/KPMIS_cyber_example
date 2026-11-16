"""Orange Pi Zero 3: лидар + одометрия rf2o + slam_toolbox
+ расстояния до препятствий + веб-страница + мост Foxglove/Lichtblick.

Поддерживаются два лидара (версия 3.1, 16.11.2026; C1 развёрнут на 180°):
  x3pro — YDLIDAR X3 Pro (драйвер ydlidar_ros2_driver, 115200 бод, до 8 м)
  c1    — SLAMTEC RPLIDAR C1 (драйвер sllidar_ros2, 460800 бод, до 12 м)
По умолчанию модель определяется сама (lidar:=auto): X3 Pro сразу после
открытия порта шлёт пакеты с заголовком AA 55 на 115200 бод, C1 молчит.

Запуск:  lidar_slam                     (или ros2 launch ~/ydlidar_ros2_ws/slam_launch.py)
Явно:    lidar_slam lidar:=c1           lidar_slam lidar:=x3pro
Без моста Foxglove/Lichtblick:  lidar_slam foxglove:=false
Модель можно закрепить файлом ~/ydlidar_ros2_ws/lidar_type (одно слово: x3pro или c1).

Просмотр: Lichtblick http://10.42.0.1:8081 (ws://10.42.0.1:8765) или http://10.42.0.1:8080
Файлы рядом: x3pro.yaml, rf2o.yaml, slam.yaml, slam_c1.yaml, lidar_markers.py, lidar_web.py
"""
import math
import os
import time
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

WS = os.path.expanduser('~/ydlidar_ros2_ws')
PORT = '/dev/ttyUSB0'


def detect_lidar(port=PORT, seconds=1.5):
    """'x3pro', если на 115200 бод приходят пакеты AA 55, иначе 'c1'."""
    try:
        import serial
        with serial.Serial(port, 115200, timeout=0.1) as s:
            s.reset_input_buffer()
            buf, t0 = b'', time.time()
            while time.time() - t0 < seconds:
                buf += s.read(256)
                if buf.count(b'\xaa\x55') >= 3:
                    return 'x3pro'
    except Exception as e:                      # нет порта — оставим X3 Pro, драйвер дождётся
        print(f'[slam_launch] не удалось открыть {port}: {e}')
        return 'x3pro'
    return 'c1'


def choose(context):
    lidar = LaunchConfiguration('lidar').perform(context).strip().lower()
    note = ''
    if lidar == 'auto':
        f = os.path.join(WS, 'lidar_type')
        if os.path.exists(f):
            lidar = open(f).read().strip().lower() or 'auto'
            note = ' (из файла lidar_type)'
    if lidar == 'auto':
        lidar = detect_lidar()
        note = ' (определён автоматически)'
    if lidar not in ('x3pro', 'c1'):
        lidar = 'x3pro'

    if lidar == 'c1':
        driver = Node(
            package='sllidar_ros2', executable='sllidar_node', name='sllidar_node', output='screen',
            parameters=[{'channel_type': 'serial', 'serial_port': PORT, 'serial_baudrate': 460800,
                         'frame_id': 'laser_frame', 'inverted': False, 'angle_compensate': True,
                         'scan_mode': 'Standard'}])
        slam_file, rf2o_freq, name = 'slam_c1.yaml', 10.0, 'SLAMTEC RPLIDAR C1'
        # у C1 ноль угла смотрит назад (проверено 16.11.2026: предмет спереди был в секторе 4)
        yaw = math.pi
    else:
        driver = Node(
            package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node', output='screen',
            parameters=[os.path.join(WS, 'x3pro.yaml')])
        slam_file, rf2o_freq, name = 'slam.yaml', 5.0, 'YDLIDAR X3 Pro'
        yaw = 0.0

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': os.path.join(WS, slam_file),
                          'use_sim_time': 'false'}.items())

    return [
        LogInfo(msg=f'Лидар: {name}{note}'),
        # положение лидара на роботе: base_link -> laser_frame (поворот yaw для C1)
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_laser',
             arguments=['--yaw', str(yaw), '--frame-id', 'base_link', '--child-frame-id', 'laser_frame']),
        # драйвер лидара -> топик /scan
        driver,
        # одометрия по сканам: odom -> base_link
        Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
             name='rf2o_laser_odometry', output='screen',
             parameters=[os.path.join(WS, 'rf2o.yaml'), {'freq': rf2o_freq}],
             arguments=['--ros-args', '--log-level', 'warn']),
        # SLAM: карта /map и map -> odom (через 3 с, когда лидар раскрутится)
        TimerAction(period=3.0, actions=[slam]),
        # расстояния до препятствий: маркеры /lidar_info и числа /lidar_sectors
        ExecuteProcess(cmd=['python3', os.path.join(WS, 'lidar_markers.py'), '--yaw', str(yaw)],
                       output='screen'),
        # веб-страница для любого браузера: http://10.42.0.1:8080
        ExecuteProcess(cmd=['python3', os.path.join(WS, 'lidar_web.py'), '--yaw', str(yaw)],
                       output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='auto',
                              description='Модель лидара: auto, x3pro или c1'),
        DeclareLaunchArgument('foxglove', default_value='true',
                              description='Запускать мост Foxglove/Lichtblick (true/false)'),
        # TF, драйвер, одометрия, SLAM и расстояния под выбранный лидар
        OpaqueFunction(function=choose),
        # мост для Foxglove / Lichtblick (порт 8765)
        Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
             parameters=[{'port': 8765}], output='log',
             condition=IfCondition(LaunchConfiguration('foxglove'))),
    ])
