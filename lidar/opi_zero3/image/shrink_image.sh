#!/bin/bash
# shrink_image.sh — уменьшить и сжать полный образ карты Orange Pi Zero 3 (МОС ОС / Linux).
#
# Сначала снимите полный образ карты (одно последовательное чтение):
#   sudo dd if=/dev/sdX of=$HOME/zero3_full.img bs=4M status=progress conv=fsync
# Затем:
#   sudo bash shrink_image.sh $HOME/zero3_full.img [размер_ГБ] [папка_для_образа]
#     размер_ГБ — до какого размера уменьшить раздел (по умолчанию 12)
#
# Скрипт работает с файлом, а не с картой: проверяет и уменьшает файловую систему,
# уменьшает раздел, обрезает файл и сжимает его в opi_zero3_lidar_ДАТА.img.gz (+ .sha256).
# Образ подходит для карт от 16 ГБ; на копии rename_board.sh расширит раздел на всю карту.

set -e
IMG="$1"; SIZE="${2:-12}"; OUT="${3:-.}"
[ "$(id -u)" -eq 0 ] || { echo "Запустите через sudo"; exit 1; }
[ -f "$IMG" ] || { echo "Нет файла образа: $IMG"; exit 1; }

echo "== 1. Подключение образа как диска"
LOOP=$(losetup -fP --show "$IMG")
trap 'losetup -d "$LOOP" 2>/dev/null || true' EXIT
sleep 1
PART="${LOOP}p1"
[ -b "$PART" ] || { echo "В образе нет раздела $PART"; exit 1; }
lsblk "$LOOP"

echo "== 2. Проверка и уменьшение файловой системы до ${SIZE} ГБ"
e2fsck -fy "$PART" || [ $? -le 2 ]          # 1–2 = ошибки исправлены, это нормально
resize2fs "$PART" "${SIZE}G"
e2fsck -fy "$PART" || [ $? -le 2 ]
losetup -d "$LOOP"; trap - EXIT

echo "== 3. Уменьшение раздела в образе"
SECT=512
START=$(sfdisk -d "$IMG" | grep -E "^[^ ]+1 " | sed -E 's/.*start= *([0-9]+).*/\1/')
[ -n "$START" ] || { echo "Не удалось прочитать начало раздела"; exit 1; }
NEWSECT=$(( (SIZE * 1024 * 1024 * 1024 + 64 * 1024 * 1024) / SECT ))   # +64 МБ запаса
echo ", $NEWSECT" | sfdisk --no-reread -N 1 "$IMG"
END_BYTES=$(( (START + NEWSECT) * SECT ))
truncate -s "$END_BYTES" "$IMG"
ls -lh "$IMG"

echo "== 4. Контрольная проверка уменьшенного образа"
LOOP=$(losetup -fP --show "$IMG"); sleep 1
e2fsck -fn "${LOOP}p1"
losetup -d "$LOOP"

NAME="opi_zero3_lidar_$(date +%Y%m%d).img.gz"
ZIP=gzip; command -v pigz >/dev/null && ZIP=pigz
echo "== 5. Сжатие -> $OUT/$NAME"
$ZIP -1 -c "$IMG" > "$OUT/$NAME"
sync
$ZIP -t "$OUT/$NAME"
sha256sum "$OUT/$NAME" | tee "$OUT/$NAME.sha256"
ls -lh "$OUT/$NAME"
echo
echo "Готово. Файл $IMG можно удалить:  sudo rm $IMG"
echo "Запись: balenaEtcher принимает .img.gz напрямую; в МОС ОС —"
echo "  gunzip -c $NAME | sudo dd of=/dev/sdX bs=4M status=progress conv=fsync"
