#!/bin/bash
# make_image.sh — снять уменьшенный образ карты Orange Pi Zero 3 (запускать в МОС ОС / Linux).
#
#   sudo bash make_image.sh /dev/sdX [размер_ГБ] [папка_для_образа]
#     /dev/sdX   — карта памяти целиком (не раздел!), смотрите lsblk
#     размер_ГБ  — до какого размера уменьшить раздел (по умолчанию 12)
#     папка      — куда положить образ (по умолчанию текущая)
#
# Что делает:
#   1) проверяет и уменьшает файловую систему и раздел карты до размер_ГБ;
#   2) снимает с карты только эту часть и сжимает: opi_zero3_lidar_ДАТА.img.gz;
#   3) считает контрольную сумму SHA-256.
# Образ подходит для карт от 16 ГБ. После записи на копии выполните
#   sudo bash ~/ydlidar_ros2_ws/rename_board.sh N
# — раздел расширится на всю карту. Эталонную карту — так же (с номером 1).

set -e
DEV="$1"; SIZE="${2:-12}"; OUT="${3:-.}"
[ "$(id -u)" -eq 0 ] || { echo "Запустите через sudo"; exit 1; }
[ -b "$DEV" ] || { echo "Укажите карту: sudo bash $0 /dev/sdX   (см. lsblk)"; exit 1; }
case "$DEV" in *[0-9]) echo "Нужно устройство целиком (/dev/sdb), а не раздел ($DEV)"; exit 1;; esac

PART="${DEV}1"
[ -b "$PART" ] || { echo "Нет раздела $PART"; exit 1; }
LABEL=$(blkid -s LABEL -o value "$PART" || true)
echo "Карта $DEV, раздел $PART, метка '$LABEL'"
lsblk "$DEV"
[ "$LABEL" = "opi_root" ] || { read -p "Метка не opi_root. Точно эта карта? (yes/no) " a; [ "$a" = yes ] || exit 1; }

echo "== 1. Отключение раздела"
umount "$DEV"?* 2>/dev/null || true

echo "== 2. Проверка и уменьшение файловой системы до ${SIZE} ГБ"
e2fsck -f -y "$PART"
resize2fs "$PART" "${SIZE}G"

echo "== 3. Уменьшение раздела"
SECT=512
START=$(sfdisk -d "$DEV" | grep "^$PART " | sed -E 's/.*start= *([0-9]+).*/\1/')
[ -n "$START" ] || { echo "Не удалось прочитать начало раздела"; exit 1; }
NEWSECT=$(( (SIZE * 1024 * 1024 * 1024 + 64 * 1024 * 1024) / SECT ))   # +64 МБ запаса
echo ", $NEWSECT" | sfdisk --no-reread -N 1 "$DEV"
partprobe "$DEV" 2>/dev/null || blockdev --rereadpt "$DEV" || true
sleep 2
e2fsck -f -y "$PART"

END_BYTES=$(( (START + NEWSECT) * SECT ))
COUNT_MB=$(( (END_BYTES + 1024 * 1024 - 1) / (1024 * 1024) ))
NAME="opi_zero3_lidar_$(date +%Y%m%d).img.gz"
ZIP=gzip; command -v pigz >/dev/null && ZIP=pigz

echo "== 4. Снятие образа: ${COUNT_MB} МБ -> $OUT/$NAME"
dd if="$DEV" bs=1M count="$COUNT_MB" status=progress | $ZIP -1 > "$OUT/$NAME"
sync

echo "== 5. Проверка и контрольная сумма"
$ZIP -t "$OUT/$NAME"
sha256sum "$OUT/$NAME" | tee "$OUT/$NAME.sha256"
ls -lh "$OUT/$NAME"
echo
echo "Готово. Запись на новую карту: balenaEtcher (Windows) принимает .img.gz напрямую,"
echo "в МОС ОС:  gunzip -c $NAME | sudo dd of=/dev/sdX bs=4M status=progress conv=fsync"
