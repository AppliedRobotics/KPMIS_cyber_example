# Документация

## Электронные модули

Описания и инфографика (PDF) — в папке [`hardware`](hardware): контроллер «Кибернетический», модули датчиков Kirus, аккумуляторный модуль 12V_3A_PowerBank, USB-адаптер USB-DXL-AR.

## Инструкции

| Инструкция | Что внутри |
|---|---|
| [Установка Arduino IDE в МОС Desktop 12](install_arduino_ide_mos.md) | Установка из репозитория МОС, доступ к USB-портам, проверка сборки и загрузки скетча |
| [Установка AR BBL в МОС Desktop 12](install_ar_bbl_mos.md) | Скачивание с Яндекс Диска, права на запуск, обновление среды и установка библиотек |
| [Обновление прошивки аккумуляторного модуля](update_powerbank_firmware.md) | Обновление 12V_3A_PowerBank до версии 2 через USB-адаптер USB-DXL-AR в МОС Desktop 12 и Windows. Программы — в [`tools/powerbank_firmware_update`](../tools/powerbank_firmware_update) |

Эти же инструкции в PDF — в папке [`pdf`](pdf).

После установки Arduino IDE для работы с контроллером «Кибернетический» понадобятся пакет плат **esp32** (Espressif) и библиотеки из папки [`libraries`](../libraries).
