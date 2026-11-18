"""Лидар (автовыбор: YDLIDAR X3 Pro или RPLIDAR C1) + одометрия rf2o + slam_toolbox + подписи + RViz2.

Запуск:   ros2 launch ~/ydlidar_ros2_ws/slam_launch.py              (лидар определяется сам)
          ros2 launch ~/ydlidar_ros2_ws/slam_launch.py lidar:=c1    (явно указать лидар: x3pro или c1)
Без RViz: ... rviz:=false
Поворот:  ... yaw:=0        (по умолчанию 180°: у обоих лидаров нулевой угол смотрит назад)
"""
import os, glob, time
import serial
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

WS = os.path.expanduser('~/ydlidar_ros2_ws')
RVIZ_CFG = os.path.join(WS, 'slam.rviz')
# поворот лидара: у X3 Pro и C1 нулевой угол смотрит назад -> по умолчанию 180°
YAW = PythonExpression(["3.1416 if '", LaunchConfiguration('yaw'), "' == 'auto' else ", LaunchConfiguration('yaw')])


def detect_lidar(ports):
    """Определяет модель лидара по ответу на порту. Возвращает (модель, порт) или (None, None)."""
    for port in ports:
        try:
            # YDLIDAR X3 Pro: сам передаёт пакеты с заголовком AA 55 на 115200
            with serial.Serial(port, 115200, timeout=0.2) as s:
                s.reset_input_buffer()
                buf, t0 = b'', time.time()
                while time.time() - t0 < 1.0:
                    buf += s.read(512)
                if buf.count(b'\xaa\x55') >= 3:
                    return 'x3pro', port
            # RPLIDAR C1: на запрос GET_INFO (A5 50) отвечает пакетом A5 5A на 460800
            with serial.Serial(port, 460800, timeout=0.5) as s:
                s.write(b'\xa5\x25')                 # STOP — на случай, если лидар сканирует
                time.sleep(0.05)
                s.reset_input_buffer()
                s.write(b'\xa5\x50')                 # GET_INFO
                if s.read(27)[:2] == b'\xa5\x5a':
                    return 'c1', port
        except (serial.SerialException, OSError):
            pass
    return None, None


def lidar_driver(context):
    lidar = LaunchConfiguration('lidar').perform(context)
    port = LaunchConfiguration('port').perform(context)
    if lidar == 'auto':
        ports = [port] if port != 'auto' else sorted(glob.glob('/dev/ttyUSB*'))
        lidar, port = detect_lidar(ports)
        if lidar is None:
            raise RuntimeError('Лидар не найден на портах %s. Проверьте подключение '
                               'или укажите явно: lidar:=x3pro / lidar:=c1' % ports)
        msg = 'Обнаружен лидар %s на порту %s' % ({'x3pro': 'YDLIDAR X3 Pro', 'c1': 'RPLIDAR C1'}[lidar], port)
    else:
        if port == 'auto':
            port = '/dev/ttyUSB0'
        msg = 'Лидар задан явно: %s, порт %s' % (lidar, port)

    if lidar == 'x3pro':
        node = Node(package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
                    name='ydlidar_ros2_driver_node', output='screen',
                    parameters=[os.path.join(WS, 'x3pro.yaml'), {'port': port}])
    elif lidar == 'c1':
        node = Node(package='sllidar_ros2', executable='sllidar_node',
                    name='sllidar_node', output='screen',
                    parameters=[{'channel_type': 'serial',
                                 'serial_port': port,
                                 'serial_baudrate': 460800,      # скорость порта C1
                                 'frame_id': 'laser_frame',
                                 'inverted': False,
                                 'angle_compensate': True,       # одинаковое число точек в скане (для SLAM)
                                 'scan_mode': 'Standard'}])
    else:
        raise RuntimeError('Неизвестный лидар "%s": допустимо auto, x3pro, c1' % lidar)
    return [LogInfo(msg=msg), node]


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
        DeclareLaunchArgument('lidar', default_value='auto',
                              description='Модель лидара: auto, x3pro или c1'),
        DeclareLaunchArgument('port', default_value='auto',
                              description='Последовательный порт лидара (auto — перебрать /dev/ttyUSB*)'),
        DeclareLaunchArgument('yaw', default_value='auto',
                              description='Поворот лидара относительно робота, рад'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Запускать RViz2 (true/false)'),

        # драйвер лидара -> /scan (модель определяется автоматически)
        OpaqueFunction(function=lidar_driver),

        # положение лидара на роботе: base_link -> laser_frame
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_laser',
             arguments=['--yaw', YAW, '--frame-id', 'base_link', '--child-frame-id', 'laser_frame']),
        # одометрия по сканам: odom -> base_link
        Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
             name='rf2o_laser_odometry', output='screen',
             parameters=[os.path.join(WS, 'rf2o.yaml')],
             arguments=['--ros-args', '--log-level', 'warn']),
        # SLAM: карта /map и map -> odom (через 3 с, когда лидар раскрутится)
        TimerAction(period=3.0, actions=[slam]),
        # подписи расстояний для RViz2 (топик /lidar_info)
        ExecuteProcess(cmd=['python3', os.path.join(WS, 'lidar_markers.py')], output='screen'),
        # RViz2 (через 5 с)
        TimerAction(period=5.0, actions=[rviz]),
    ])
