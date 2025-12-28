# ESP-Drone ATK-MS901M

[English](./README.md)

基于 ESP-IDF 的无人机固件，fork 自 Espressif ESP-Drone，新增 ATK-MS901M IMU 模块（陀螺仪/加速度计/磁力计/气压计）UART 接入。保留 ESP-Drone 飞控框架与配置流程，同时替换传感器路径为 MS901M。

> 支持 ESP32 / ESP32-S2 / ESP32-S3。请使用 ESP-IDF release/v5.0。

![ESP-Drone](./docs/_static/espdrone_s2_v1_2_2.png)

## 功能
1. 自稳定模式
2. 定高模式（需要扩展板）
3. 定点模式（需要扩展板）
4. APP 控制
5. 支持 cfclient（https://github.com/leeebo/crazyflie-clients-python）
6. ESP-BOX3 手柄控制（ESP-NOW）

## 构建
1. 安装 ESP-IDF v5.0（release/v5.0）。
2. `idf.py set-target esp32s2`（或你的板型 `esp32` / `esp32s3`）。
3. `idf.py menuconfig` 选择 ESP-Drone 硬件目标。
4. `idf.py build flash monitor`。

## ATK-MS901M 配置
编辑 `components/config/include/config.h`：

```c
#define USE_ATK_MS901M 1
#define ATK_MS901M_UART_PORT_NUM 1
#define ATK_MS901M_UART_TX_PIN 36
#define ATK_MS901M_UART_RX_PIN 37
#define ATK_MS901M_UART_BAUDRATE 115200
#define ATK_MS901M_RETURNSET 0x1C
#define ATK_MS901M_RETURNRATE 0x01
#define ATK_MS901M_USE_MODULE_ORIENTATION 0
#define ATK_MS901M_ORIENTATION_SOURCE ATK_MS901M_ORIENTATION_SOURCE_QUAT
```

轴向映射可调：

```
ATK_MS901M_AXIS_SWAP_XY
ATK_MS901M_AXIS_INVERT_X
```

备注：
- 对于 `CONFIG_TARGET_ESP32_S2_DRONE_V1_2`，默认 UART 引脚 TX=IO36，RX=IO37。
- 当 `USE_ATK_MS901M=1` 时，deck SPI 会关闭以避免引脚冲突。

## 文档与硬件
- 文档：`docs/`
- 硬件参考：`hardware/`

## 第三方代码
| 组件 | License | 源代码 | Commit ID |
| :---: | :---: | :---: | :---: |
| core/crazyflie | GPL-3.0 | https://github.com/bitcraze/crazyflie-firmware/tree/2021.01 | tag_2021_01 b448553 |
| lib/dsp_lib |  | https://github.com/whyengineer/esp32-lin/tree/master/components/dsp_lib | 6fa39f4c |

## 许可证
GPL-3.0，见 `LICENSE`。

## 致谢
- Bitcraze 的 Crazyflie
- Espressif 的 ESP-IDF 和 ESP-Drone
- WhyEngineer 的 ESP-DSP