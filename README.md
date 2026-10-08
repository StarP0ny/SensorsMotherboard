# SensorsMotherboard

Шлюз CH32V303CBT6 → USB CDC / UART → VisserLab. Первый этап: бинарный
протокол, пинги, синхронизация UTC, TCA9548A, SCD41 и SHT41.

[Протокол, подключения и согласованные решения](docs/PROTOCOL.md).

## Сборка

```powershell
git submodule update --init --recursive
cmake --preset firmware -DWK_SDK_PATH=C:/Users/StarPony/Documents/Welrok/wk-sdk
cmake --build --preset firmware
```

Можно задать WK_SDK_PATH переменной окружения. Результаты в
`build/firmware/firmware.elf`, `.hex`, `.bin`. Нужны CMake ≥3.21, Ninja,
WCH RISCV_GCC12 из Welrok SDK. Локальные пути к ktz не нужны.

GPIO/каналы: `config/board_config.h`. До проверки USB-разъёма можно
использовать UART PA9/PA10 с адаптером 3,3 В.
После включения опрос запускается кнопкой PB0 либо START от ПК.
PB1 показывает автоматический режим. Первый отсчёт SCD41 примерно
через 6 с, SHT41 через 10 мс после запуска.

В соседнем VisserLab добавлены драйверы `vgw`, `scd41`, `sht41`.
Пример стенда: `examples/devices.yaml`, заменить COM-порт.
Запуск из VisserLab: `python -m visserlab --config ../SensorsMotherboard/examples collect`.

## Прошивка через WCH-Link

Из каталога SensorsMotherboard после сборки:

```powershell
& 'C:/Users/StarPony/Documents/Welrok/wk-sdk/WCH/OpenOCD-wch/bin/openocd.exe' -f 'C:/Users/StarPony/Documents/Welrok/wk-sdk/WCH/OpenOCD-wch/bin/wch-riscv.cfg' -c 'program build/firmware/firmware.elf verify reset exit'
```

На текущем стенде проверены запись с верификацией, USB CDC (COM27), HELLO,
ENUM, START/STOP, пинги и синхронизация UTC. За 12 секунд обмена потерь
кадров не обнаружено. COM3 относится к UART WCH-Link; ответа прошивки
через него пока нет. Опрос SCD41/SHT41 возвращает ошибки I²C — реальные
измерения ещё не проверены.

## Проверка без платы

```powershell
python -m pytest tests -q
```

Нужен нативный GCC (MinGW) либо переменная HOST_CC с его путём.
Тесты сверяют C/Python кадры и выполняют код прошивки с подменённой периферией.
Сборка и тесты не заменяют проверку USB/I²C на плате.
SDP810, flash-журнал, сохранение настроек и RTC — следующий этап.
