#!/bin/bash
# setup_c1.sh — поддержка SLAMTEC RPLIDAR C1 на Orange Pi Zero 3 (рядом с YDLIDAR X3 Pro).
# Запуск (от orangepi, НЕ через sudo):  bash ~/ydlidar_ros2_ws/setup_c1.sh
# Перед запуском положите в ~/ydlidar_ros2_ws/ новый slam_launch.py (версия 3).
# Что делает:
#   1) скачивает и собирает официальный драйвер sllidar_ros2 (SLAMTEC);
#   2) создаёт slam_c1.yaml (как slam.yaml, но дальность 12 м);
#   3) перезапускает службу lidar-slam, если она включена.
# После этого lidar_slam (и автозапуск) сами определяют, какой лидар подключён.

set -e
WS="$HOME/ydlidar_ros2_ws"

grep -q "detect_lidar" "$WS/slam_launch.py" \
    || { echo "В $WS лежит старый slam_launch.py — скопируйте версию 3"; exit 1; }
[ -f "$WS/slam.yaml" ] || { echo "Нет $WS/slam.yaml — сначала выполните setup_slam.sh"; exit 1; }

echo "== 1. Драйвер sllidar_ros2"
cd "$WS/src"
[ -d sllidar_ros2 ] || git clone https://github.com/Slamtec/sllidar_ros2.git
cd "$WS"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select sllidar_ros2

echo "== 2. slam_c1.yaml"
sed -e 's/max_laser_range: .*/max_laser_range: 12.0/' "$WS/slam.yaml" > "$WS/slam_c1.yaml"
grep -E "base_frame|max_laser_range|minimum_travel" "$WS/slam_c1.yaml"

echo "== 3. Служба автозапуска"
if systemctl is-enabled --quiet lidar-slam 2>/dev/null; then
    sudo systemctl restart lidar-slam
    echo "Служба lidar-slam перезапущена. Журнал: slam_log"
else
    echo "Автозапуск не включён — запускайте вручную: lidar_slam"
fi

echo
echo "Готово. Лидар определяется сам при каждом запуске."
echo "Закрепить модель вручную:  echo c1 > ~/ydlidar_ros2_ws/lidar_type   (или x3pro; удалить файл — снова авто)"
