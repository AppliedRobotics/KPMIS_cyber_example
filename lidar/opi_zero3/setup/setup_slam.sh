#!/bin/bash
# setup_slam.sh — установка SLAM на Orange Pi Zero 3 (запускать от orangepi, НЕ через sudo):
#   bash ~/ydlidar_ros2_ws/setup_slam.sh
# Перед запуском положите в ~/ydlidar_ros2_ws/ файлы:
#   slam_launch.py, lidar_markers.py, rf2o.yaml (и этот скрипт)
# Что делает:
#   1) ставит slam_toolbox и nav2_map_server;
#   2) скачивает и собирает rf2o_laser_odometry (одометрия по сканам);
#   3) копирует x3pro.yaml в ~/ydlidar_ros2_ws/ и создаёт slam.yaml;
#   4) добавляет команды lidar_slam, save_map, lidar_dist в ~/.bashrc.

set -e
WS="$HOME/ydlidar_ros2_ws"
cd "$WS"

for f in slam_launch.py lidar_markers.py rf2o.yaml; do
    [ -f "$WS/$f" ] || { echo "Нет файла $WS/$f — скопируйте его через SFTP"; exit 1; }
done

echo "== 1. Пакеты slam_toolbox и nav2_map_server (спросит пароль sudo)"
sudo apt install -y ros-humble-slam-toolbox ros-humble-nav2-map-server

echo "== 2. rf2o_laser_odometry"
cd "$WS/src"
if [ ! -d rf2o_laser_odometry ]; then
    git clone -b ros2 https://github.com/MAPIRlab/rf2o_laser_odometry.git \
        || git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git
fi
cd "$WS"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rf2o_laser_odometry

echo "== 3. Настройки"
[ -f "$WS/x3pro.yaml" ] || cp "$WS/src/ydlidar_ros2_driver/params/x3pro.yaml" "$WS/x3pro.yaml"
cp /opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml "$WS/slam.yaml"
sed -i -e 's/base_footprint/base_link/' \
       -e 's/max_laser_range: .*/max_laser_range: 8.0/' \
       -e 's/minimum_travel_distance: .*/minimum_travel_distance: 0.1/' \
       -e 's/minimum_travel_heading: .*/minimum_travel_heading: 0.1/' \
       "$WS/slam.yaml"
grep -E "base_frame|max_laser_range|minimum_travel" "$WS/slam.yaml"

echo "== 4. Короткие команды"
add_alias() { grep -q "alias $1=" ~/.bashrc || echo "alias $1='$2'" >> ~/.bashrc; }
add_alias lidar_slam 'ros2 launch ~/ydlidar_ros2_ws/slam_launch.py 2>&1 | grep --line-buffered -v "fixed points"'
add_alias save_map 'ros2 run nav2_map_server map_saver_cli -f ~/room_map'
add_alias lidar_dist 'ros2 topic echo /lidar_sectors'

echo
echo "Готово. Выполните:  source ~/.bashrc"
echo "Затем:  lidar_slam   (карта и расстояния — в Foxglove: ws://10.42.0.1:8765)"
