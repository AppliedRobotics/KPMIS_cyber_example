#!/bin/bash
# rename_board.sh — делает копию платы Orange Pi Zero 3 уникальной.
#
# Использование:  sudo bash rename_board.sh N [пароль_wifi] [--new-ssh-keys]
#   N             — номер платы (1, 2, 3 ...)
#   пароль_wifi   — необязательно, не короче 8 символов (по умолчанию orangepizero3)
#   --new-ssh-keys — заново создать ключи SSH (тогда Bitvise при переключении
#                   между платами будет предупреждать о смене ключа)
#
# Что меняет:
#   имя сети Wi-Fi  OrangePiZero3-N   (для N=1 — OrangePiZero3)
#   канал Wi-Fi     1 / 6 / 11 по кругу
#   имя платы       orangepizero3-N   (для N=1 — orangepizero3)
#   ROS_DOMAIN_ID   N                 (в ~/.bashrc пользователя orangepi)
#   machine-id      новый (иначе по Ethernet клоны получают одинаковый адрес)
#   раздел карты    расширяется на всю карту (после образа make_image.sh)
# Адрес платы в своей сети остаётся 10.42.0.1.
# После выполнения плата перезагружается.

set -e

SERVICE=/etc/systemd/system/create_ap.service
USER_HOME=/home/orangepi

if [ "$(id -u)" -ne 0 ]; then
    echo "Запустите через sudo: sudo bash $0 N"; exit 1
fi

N="$1"
if ! [[ "$N" =~ ^[0-9]+$ ]] || [ "$N" -lt 1 ]; then
    echo "Укажите номер платы: sudo bash $0 2"; exit 1
fi

PASS="orangepizero3"
NEW_KEYS=0
for arg in "${@:2}"; do
    if [ "$arg" = "--new-ssh-keys" ]; then
        NEW_KEYS=1
    else
        PASS="$arg"
    fi
done
if [ ${#PASS} -lt 8 ]; then
    echo "Пароль Wi-Fi должен быть не короче 8 символов"; exit 1
fi

if [ "$N" -eq 1 ]; then
    SSID="OrangePiZero3"
    HOST="orangepizero3"
else
    SSID="OrangePiZero3-$N"
    HOST="orangepizero3-$N"
fi
CHANNELS=(1 6 11)
CH=${CHANNELS[$(( (N - 1) % 3 ))]}
OLD_HOST=$(hostname)

echo "Плата $N:  сеть $SSID, канал $CH, имя $HOST, ROS_DOMAIN_ID=$N"

# 1. Точка доступа
if [ ! -f "$SERVICE" ]; then
    echo "Нет $SERVICE — сначала настройте точку доступа (create_ap)"; exit 1
fi
sed -i "s|^ExecStart=.*|ExecStart=/usr/bin/create_ap -m nat wlan0 eth0 $SSID $PASS --no-virt -c $CH -g 10.42.0.1|" "$SERVICE"

# 2. Имя платы
hostnamectl set-hostname "$HOST"
if grep -q "^127\.0\.1\.1" /etc/hosts; then
    sed -i "s|^127\.0\.1\.1.*|127.0.1.1\t$HOST|" /etc/hosts
else
    printf '127.0.1.1\t%s\n' "$HOST" >> /etc/hosts
fi

# 3. Домен ROS 2
BASHRC="$USER_HOME/.bashrc"
sed -i '/^export ROS_DOMAIN_ID=/d' "$BASHRC"
echo "export ROS_DOMAIN_ID=$N" >> "$BASHRC"

# 4. Новый machine-id
rm -f /etc/machine-id /var/lib/dbus/machine-id
systemd-machine-id-setup > /dev/null
ln -sf /etc/machine-id /var/lib/dbus/machine-id

# 5. Расширить раздел на всю карту (после записи уменьшенного образа make_image.sh)
ROOTDEV=$(findmnt -n -o SOURCE /)
DISK=/dev/$(lsblk -no PKNAME "$ROOTDEV")
PARTNUM=$(echo "$ROOTDEV" | grep -o '[0-9]*$')
if command -v growpart > /dev/null; then
    if growpart "$DISK" "$PARTNUM"; then
        resize2fs "$ROOTDEV"
        echo "Раздел расширен на всю карту"
    fi
else
    echo "growpart не установлен (sudo apt install cloud-guest-utils) — раздел не расширен"
fi
df -h / | tail -1

# 6. Ключи SSH (по желанию)
if [ "$NEW_KEYS" -eq 1 ]; then
    rm -f /etc/ssh/ssh_host_*
    ssh-keygen -A > /dev/null
    echo "Ключи SSH созданы заново"
fi

systemctl daemon-reload

echo
echo "Готово ($OLD_HOST -> $HOST). Проверка:"
grep '^ExecStart=' "$SERVICE"
grep '^127.0.1.1' /etc/hosts
grep '^export ROS_DOMAIN_ID=' "$BASHRC"
echo
echo "Перезагрузка через 5 секунд. Потом подключайтесь к Wi-Fi $SSID, адрес 10.42.0.1"
sleep 5
reboot
