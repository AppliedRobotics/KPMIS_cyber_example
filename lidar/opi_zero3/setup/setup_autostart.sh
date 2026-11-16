#!/bin/bash
# setup_autostart.sh — автозапуск лидара, SLAM и расстояний при включении Orange Pi Zero 3.
# Запуск (от orangepi, НЕ через sudo):  bash ~/ydlidar_ros2_ws/setup_autostart.sh
# Отключить автозапуск:                 bash ~/ydlidar_ros2_ws/setup_autostart.sh off
#
# Создаёт службу lidar-slam: при загрузке запускает slam_launch.py
# (драйвер лидара + rf2o + slam_toolbox + lidar_markers + lidar_web + foxglove_bridge).
# Lichtblick (порт 8081) уже запускается сам — это контейнер Docker.

set -e
SERVICE=/etc/systemd/system/lidar-slam.service
WS=/home/orangepi/ydlidar_ros2_ws

if [ "$1" = "off" ]; then
    sudo systemctl disable --now lidar-slam || true
    echo "Автозапуск выключен. Запуск вручную: lidar_slam"
    exit 0
fi

[ -f "$WS/slam_launch.py" ] || { echo "Нет $WS/slam_launch.py"; exit 1; }

sudo tee "$SERVICE" > /dev/null <<'EOF'
[Unit]
Description=Lidar YDLIDAR X3 Pro + SLAM + Foxglove bridge (slam_launch.py)
After=network-online.target create_ap.service
Wants=network-online.target

[Service]
Type=simple
User=orangepi
Environment=HOME=/home/orangepi
# ROS 2, наша рабочая область и ROS_DOMAIN_ID из ~/.bashrc (его задаёт rename_board.sh)
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash; \
  source /home/orangepi/ydlidar_ros2_ws/install/setup.bash; \
  eval "$$(grep "^export ROS_DOMAIN_ID=" /home/orangepi/.bashrc)"; \
  ros2 launch /home/orangepi/ydlidar_ros2_ws/slam_launch.py 2>&1 | grep --line-buffered -v "fixed points"'
KillSignal=SIGINT
TimeoutStopSec=20
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable lidar-slam

# короткие команды для управления службой
add_alias() { grep -q "alias $1=" ~/.bashrc || echo "alias $1='$2'" >> ~/.bashrc; }
add_alias slam_status 'systemctl status lidar-slam --no-pager | head -15'
add_alias slam_log    'journalctl -u lidar-slam -f'
add_alias slam_restart 'sudo systemctl restart lidar-slam'
add_alias slam_stop   'sudo systemctl stop lidar-slam'

echo
echo "Автозапуск включён. Если сейчас работает lidar_slam в терминале — остановите его (Ctrl+C),"
echo "затем:  sudo systemctl start lidar-slam   (или просто перезагрузите плату)"
echo "Команды (после source ~/.bashrc): slam_status, slam_log, slam_restart, slam_stop"
