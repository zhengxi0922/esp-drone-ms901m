# ESP-Drone ATK-MS901M

[中文](./README_cn.md)

ESP-IDF based drone firmware forked from Espressif ESP-Drone, adding ATK-MS901M IMU (gyro/accel/mag/baro) over UART. It keeps the ESP-Drone flight stack and configuration flow while swapping the sensor path to the integrated MS901M.

> Supports ESP32 / ESP32-S2 / ESP32-S3. Use ESP-IDF release/v5.0.

![ESP-Drone](./docs/_static/espdrone_s2_v1_2_2.png)

## Features
1. Stabilize mode
2. Height-hold mode (requires extension boards)
3. Position-hold mode (requires extension boards)
4. Mobile APP control
5. cfclient support (https://github.com/leeebo/crazyflie-clients-python)
6. ESP-BOX3 joystick control (ESP-NOW)

## Build
1. Install ESP-IDF v5.0 (release/v5.0).
2. `idf.py set-target esp32s2` (or `esp32` / `esp32s3` for your board).
3. `idf.py menuconfig` and select the ESP-Drone hardware target.
4. `idf.py build flash monitor`.

## ATK-MS901M Configuration
Edit `components/config/include/config.h`:

```c
#define USE_ATK_MS901M 1
#define ATK_MS901M_UART_PORT_NUM 1
#define ATK_MS901M_UART_TX_PIN 37
#define ATK_MS901M_UART_RX_PIN 36
#define ATK_MS901M_UART_BAUDRATE 115200
#define ATK_MS901M_RETURNSET 0x1C
#define ATK_MS901M_RETURNRATE 0x01
#define ATK_MS901M_USE_MODULE_ORIENTATION 0
#define ATK_MS901M_ORIENTATION_SOURCE ATK_MS901M_ORIENTATION_SOURCE_QUAT
```

Axis alignment (if needed):

```
ATK_MS901M_AXIS_SWAP_XY
ATK_MS901M_AXIS_INVERT_X
```

Notes:
- For `CONFIG_TARGET_ESP32_S3_DRONE_WROOM_1` (and legacy `CONFIG_TARGET_ESP32_S2_DRONE_V1_2`), the default UART pins are TX=IO36 and RX=IO37.
- When `USE_ATK_MS901M=1`, deck SPI is disabled to avoid pin conflicts.

## Docs and Hardware
- Documentation: `docs/`
- Schematics and references: `hardware/`

## Third Party Code
| Component | License | Origin | Commit ID |
| :---: | :---: | :---: | :---: |
| core/crazyflie | GPL-3.0 | https://github.com/bitcraze/crazyflie-firmware/tree/2021.01 | tag_2021_01 b448553 |
| lib/dsp_lib |  | https://github.com/whyengineer/esp32-lin/tree/master/components/dsp_lib | 6fa39f4c |

## License
GPL-3.0. See `LICENSE`.

## Thanks
- Bitcraze for Crazyflie
- Espressif for ESP-IDF and ESP-Drone
- WhyEngineer for ESP-DSP
