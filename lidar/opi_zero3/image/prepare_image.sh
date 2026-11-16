#!/bin/bash
# prepare_image.sh — подготовка эталонной Orange Pi Zero 3 к снятию образа карты памяти.
# Запуск на плате:  sudo bash ~/ydlidar_ros2_ws/prepare_image.sh [--zero]
#   --zero  заполнить свободное место нулями (образ сожмётся лучше; на карте 32 ГБ
#           займёт 15–25 минут)
# В конце плата выключается. Затем карту — в компьютер с МОС ОС, там make_image.sh.

set -e
[ "$(id -u)" -eq 0 ] || { echo "Запустите через sudo"; exit 1; }

echo "== 1. growpart (нужен rename_board.sh, чтобы копия расширила раздел на всю карту)"
apt install -y cloud-guest-utils

echo "== 2. Чистка"
apt clean
journalctl --vacuum-time=1s || true
rm -rf /home/orangepi/.ros/log/* /root/.ros/log/* 2>/dev/null || true
rm -f /home/orangepi/.bash_history /root/.bash_history
rm -f /boot/netinfo.txt
df -h /

if [ "$1" = "--zero" ]; then
    echo "== 3. Заполнение свободного места нулями (долго, ждите)"
    dd if=/dev/zero of=/zero.fill bs=4M status=progress || true
    sync; rm -f /zero.fill; sync
fi

echo
echo "Готово. Плата выключится через 5 секунд."
echo "Дождитесь, пока погаснет зелёный светодиод, выньте карту и запустите make_image.sh в МОС ОС."
sleep 5
poweroff
