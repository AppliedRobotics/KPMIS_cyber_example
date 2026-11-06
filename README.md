# Примеры для робототехнического контроллера «Кибернетический»

Примеры кода для робототехнического контроллера «Кибернетический» (Ardu8, плата Arduino V8.3, ООО «Прикладная робототехника ПРО»): Arduino IDE и блочная среда AR BBL.

Контроллер состоит из двух частей, у каждой свой USB-C:
- **Arduino** (ATmega328P, верхний USB-C на левом краю) — в Arduino IDE плата *Arduino Uno*, в AR BBL — *Uno*;
- **ESP32** (ESP32-WROVER, нижний USB-C) — в Arduino IDE плата *ESP32 Wrover Module* (пакет плат esp32 от Espressif), в AR BBL — *ESP32*.

## Что где лежит

| Папка | Что внутри |
|---|---|
| [`lessons`](lessons) | 51 пример к учебным программам на 34 и 68 часов (Arduino IDE) и 24 проекта AR BBL, таблица по урокам |
| [`ar_bbl_examples`](ar_bbl_examples) | 19 проектов AR BBL для ESP32 и Arduino с описанием особенностей блоков, сгенерированный C++-код |
| [`board_tests`](board_tests) | Тесты контроллера, моторов и модулей датчиков Kirus |
| [`mecanum_4wd`](mecanum_4wd) | Мобильная платформа 4WD с меканум-колёсами: тест и пульт по Wi-Fi |
| [`libraries`](libraries) | Библиотеки DxlMaster2 и FeetechSTS |
| [`docs`](docs) | Инструкции: установка Arduino IDE и AR BBL в МОС Desktop 12 |
| `_old_examples` | Предыдущий набор примеров |

## Быстрый старт

1. Установить Arduino IDE и пакет плат **esp32** (Espressif). Для МОС Desktop 12 есть [инструкция](docs/install_arduino_ide_mos.md); для блочной среды — [установка AR BBL](docs/install_ar_bbl_mos.md).
2. Установить библиотеки из папки [`libraries`](libraries) («Добавить .ZIP библиотеку»).
3. Открыть пример: папка и файл `.ino` должны называться одинаково.
4. Выставить DIP-переключатели, как написано в начале примера.

## DIP-переключатель

| № | ON = соединяет |
|---|---|
| 1 | ESP32 TX (IO23) → Arduino RX (D0) |
| 2 | Arduino TX (D1) → ESP32 RX (IO22) |
| 3 | ESP32 ↔ шина DXL (моторы, сервоприводы) |
| 4 | Arduino ↔ шина DXL |
| 5 | Включает светодиоды Arduino (74HC595) |

- У Arduino один аппаратный `Serial`, общий для USB, шины DXL и связи с ESP32. Прошивать Arduino при DIP1 и DIP4 = OFF.
- DIP3 и DIP4 одновременно не включать: шиной управляет один контроллер.
- Для моторов и сервоприводов нужно внешнее питание (XT60 или DC).
