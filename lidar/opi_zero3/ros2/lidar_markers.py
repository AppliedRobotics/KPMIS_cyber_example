#!/usr/bin/env python3
"""Расстояния до препятствий по лидару.

Публикует:
  /lidar_info    (visualization_msgs/MarkerArray) — для Foxglove/RViz2:
                 красная линия и подпись до ближайшего препятствия,
                 белые подписи с минимумом в каждом из 8 секторов по 45°;
  /lidar_sectors (std_msgs/Float32MultiArray) — числа для программ и робота:
                 [ближайшее_м, угол_ближайшего_град, сектор0_м, ..., сектор7_м]
                 сектор 0 — вперёд (ось X лидара, ±22,5°), далее против часовой
                 стрелки через 45°; 0.0 — в секторе нет точек.

Посмотреть числа:  ros2 topic echo /lidar_sectors   (или команда lidar_dist)
Версия 2 (Zero 3, 16.11.2026): добавлен топик /lidar_sectors.
Версия 3: параметр --yaw — поворот лидара относительно робота (у C1 = pi);
           сектора и угол ближайшего считаются в системе робота.
"""
import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Float32MultiArray

SECTORS = 8          # секторы по 45°, сектор 0 — вперёд (ось X робота, base_link)
# поворот лидара относительно робота, рад (--yaw; у RPLIDAR C1 = pi)
YAW = float(sys.argv[sys.argv.index('--yaw') + 1]) if '--yaw' in sys.argv else 0.0
MIN_GAP = 0.45       # подписи ближе этого расстояния (м) друг к другу не ставим
RED = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)
YELLOW = ColorRGBA(r=1.0, g=1.0, b=0.2, a=1.0)
WHITE = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)


class LidarMarkers(Node):
    def __init__(self):
        super().__init__('lidar_markers')
        self.pub = self.create_publisher(MarkerArray, 'lidar_info', 10)
        self.pub_sect = self.create_publisher(Float32MultiArray, 'lidar_sectors', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

    def marker(self, scan, mid, mtype, color):
        m = Marker()
        m.header.frame_id = scan.header.frame_id   # время 0 = последнее известное положение
        m.ns, m.id, m.type, m.action = 'lidar_info', mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.color = color
        m.lifetime = Duration(seconds=0.3).to_msg()  # старая подпись исчезает, если не обновилась
        return m

    def text(self, scan, mid, x, y, txt, color, size):
        m = self.marker(scan, mid, Marker.TEXT_VIEW_FACING, color)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.1
        m.scale.z = size                           # высота букв, м
        m.text = txt
        return m

    def on_scan(self, scan):
        # точки (расстояние, угол) с допустимым расстоянием
        pts = [(r, scan.angle_min + i * scan.angle_increment)
               for i, r in enumerate(scan.ranges)
               if math.isfinite(r) and scan.range_min <= r <= scan.range_max]
        if not pts:
            return
        arr = MarkerArray()
        placed = []                                # координаты уже поставленных подписей

        # ближайшее препятствие: линия, точка и подпись
        r0, a0 = min(pts)
        x, y = r0 * math.cos(a0), r0 * math.sin(a0)
        line = self.marker(scan, 0, Marker.LINE_STRIP, RED)
        line.scale.x = 0.02                        # толщина линии, м
        line.points = [Point(), Point(x=x, y=y)]
        dot = self.marker(scan, 1, Marker.SPHERE, RED)
        dot.pose.position.x, dot.pose.position.y = x, y
        dot.scale.x = dot.scale.y = dot.scale.z = 0.08
        arr.markers += [line, dot, self.text(scan, 2, x, y + 0.15, f'{r0:.2f}m', YELLOW, 0.14)]
        placed.append((x, y + 0.15))               # место жёлтой подписи

        # минимальное расстояние в каждом секторе
        best = {}
        for r, a in pts:
            k = int(((math.degrees(a + YAW) + 22.5) % 360) // 45)   # сектор робота
            if k not in best or r < best[k][0]:
                best[k] = (r, a)
        for k in sorted(best, key=lambda k: best[k][0]):   # сначала самые близкие
            r, a = best[k]
            x, y = r * math.cos(a), r * math.sin(a)
            if all(math.hypot(x - px, y - py) >= MIN_GAP for px, py in placed):
                arr.markers.append(self.text(scan, 10 + k, x, y, f'{r:.2f}', WHITE, 0.12))
                placed.append((x, y))
        self.pub.publish(arr)

        # числа: ближайшее, его угол и минимумы по секторам
        msg = Float32MultiArray()
        msg.data = [float(r0), float(math.degrees(a0 + YAW) % 360)] + \
                   [float(best[k][0]) if k in best else 0.0 for k in range(SECTORS)]
        self.pub_sect.publish(msg)


def main():
    rclpy.init(args=[a for a in sys.argv if a != '--yaw' and a != str(YAW)][:1])
    try:
        rclpy.spin(LidarMarkers())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
