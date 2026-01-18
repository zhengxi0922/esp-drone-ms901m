/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011-2018 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * sensors_atk_ms901m.c - ATK-MS901M UART sensor interface.
 */

#define DEBUG_MODULE "SENSORS"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "driver/uart.h"

#include "sensors_atk_ms901m.h"
#include "system.h"
#include "stm32_legacy.h"
#include "param.h"
#include "log.h"
#include "filter.h"
#include "config.h"
#if ATK_MS901M_GYRO_BIAS_LIGHT_WEIGHT
#ifndef GYRO_BIAS_LIGHT_WEIGHT
#define GYRO_BIAS_LIGHT_WEIGHT
#endif
#endif
#include "usec_time.h"
#include "static_mem.h"
#include "debug_cf.h"

#include "imu.h"

#ifndef PITCH_CALIB
#define PITCH_CALIB (CONFIG_PITCH_CALIB * 1.0f / 100.0f)
#endif
#ifndef ROLL_CALIB
#define ROLL_CALIB (CONFIG_ROLL_CALIB * 1.0f / 100.0f)
#endif

#define ATK_MS901M_FRAME_HEAD_L             0x55
#define ATK_MS901M_FRAME_HEAD_UPLOAD_H      0x55
#define ATK_MS901M_FRAME_HEAD_ACK_H         0xAF
#define ATK_MS901M_FRAME_DAT_MAX_SIZE       28

#define ATK_MS901M_FRAME_ID_ATTITUDE        0x01
#define ATK_MS901M_FRAME_ID_QUAT            0x02
#define ATK_MS901M_FRAME_ID_GYRO_ACCE       0x03
#define ATK_MS901M_FRAME_ID_MAG             0x04
#define ATK_MS901M_FRAME_ID_BARO            0x05
#define ATK_MS901M_FRAME_ID_PORT            0x06

#define ATK_MS901M_FRAME_ID_REG_GYROFSR     0x03
#define ATK_MS901M_FRAME_ID_REG_ACCFSR      0x04
#define ATK_MS901M_FRAME_ID_REG_RETURNSET   0x08
#define ATK_MS901M_FRAME_ID_REG_RETURNRATE  0x0A

#define ATK_MS901M_READ_REG_ID(id)          ((id) | 0x80)

#define ATK_MS901M_RETURNSET_ATTITUDE       (1U << 0)
#define ATK_MS901M_RETURNSET_QUAT           (1U << 1)
#define ATK_MS901M_RETURNSET_GYRO_ACC       (1U << 2)
#define ATK_MS901M_RETURNSET_MAG            (1U << 3)
#define ATK_MS901M_RETURNSET_BARO           (1U << 4)
#define ATK_MS901M_RETURNSET_PORT           (1U << 5)

#define ATK_MS901M_UART_PORT                ((uart_port_t)ATK_MS901M_UART_PORT_NUM)

#define MAG_GAUSS_PER_LSB                   666.7f

#define SENSORS_BIAS_SAMPLES                1000
#define SENSORS_ACC_SCALE_SAMPLES           200
#define SENSORS_NBR_OF_BIAS_SAMPLES         1024
#define GYRO_NBR_OF_AXES                    3
#define GYRO_MIN_BIAS_TIMEOUT_MS            M2T(1 * 1000)
#define GYRO_VARIANCE_BASE                  5000
#define GYRO_VARIANCE_THRESHOLD_X           (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y           (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z           (GYRO_VARIANCE_BASE)

#define GYRO_LPF_CUTOFF_FREQ                80
#define ACCEL_LPF_CUTOFF_FREQ               30

typedef struct {
  Axis3f bias;
  Axis3f variance;
  Axis3f mean;
  bool isBiasValueFound;
  bool isBufferFilled;
  Axis3i16 *bufHead;
  Axis3i16 buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
} BiasObj;

typedef struct {
  uint8_t head_h;
  uint8_t id;
  uint8_t len;
  uint8_t dat[ATK_MS901M_FRAME_DAT_MAX_SIZE];
  uint8_t check_sum;
} atk_ms901m_frame_t;

typedef enum {
  wait_for_head_l = 0x00,
  wait_for_head_h = 0x01,
  wait_for_id     = 0x02,
  wait_for_len    = 0x04,
  wait_for_dat    = 0x08,
  wait_for_sum    = 0x16,
} atk_ms901m_handle_state_t;

typedef struct {
  atk_ms901m_handle_state_t state;
  atk_ms901m_frame_t frame;
  uint8_t dat_index;
} atk_ms901m_parser_t;

static const uint16_t atk_ms901m_gyro_fsr_table[4] = {250, 500, 1000, 2000};
static const uint8_t atk_ms901m_acc_fsr_table[4] = {2, 4, 8, 16};

static struct {
  uint8_t gyro;
  uint8_t accelerometer;
} atk_ms901m_fsr = {
  .gyro = 3,
  .accelerometer = 1,
};

static xQueueHandle accelerometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(accelerometerDataQueue, 1, sizeof(Axis3f));
static xQueueHandle gyroDataQueue;
STATIC_MEM_QUEUE_ALLOC(gyroDataQueue, 1, sizeof(Axis3f));
static xQueueHandle magnetometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(magnetometerDataQueue, 1, sizeof(Axis3f));
static xQueueHandle barometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(barometerDataQueue, 1, sizeof(baro_t));

static xSemaphoreHandle dataReady;
static StaticSemaphore_t dataReadyBuffer;

static xSemaphoreHandle atkUartMutex;
static StaticSemaphore_t atkUartMutexBuffer;
static volatile bool atkParserResetPending = false;

static bool isInit = false;
static bool isMs901mPresent = false;
static bool gyroBiasFound = false;
static uint32_t gyroBiasSampleCount = 0;
static float accScaleSum = 0;
static float accScale = 1;

static sensorData_t sensorData;
static Axis3i16 gyroRaw;
static Axis3i16 accelRaw;
static attitude_t attitudeData;
static quaternion_t quaternionData;
static bool attitudeValid = false;
static bool quaternionValid = false;
#if ATK_MS901M_STREAM_1HZ
static bool gyroAccValid = false;
static bool magValid = false;
static bool baroValid = false;
#endif
static BiasObj gyroBiasRunning;
static Axis3f gyroBias;

static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];

static float cosPitch;
static float sinPitch;
static float cosRoll;
static float sinRoll;

#if ATK_MS901M_CAL_DEBUG
static uint32_t atkCalLastLogTick = 0;
static uint32_t atkRxBytes = 0;
static uint32_t atkRxFrames = 0;
static uint32_t atkRxGyroFrames = 0;
static uint8_t atkLastFrameId = 0;
static uint8_t atkLastFrameLen = 0;
static Axis3i16 atkLastGyroRaw;
static Axis3i16 atkLastAccRaw;
#endif

STATIC_MEM_TASK_ALLOC(sensorsTask, SENSORS_TASK_STACKSIZE);

static void atk_ms901m_uart_lock(void);
static void atk_ms901m_uart_unlock(void);
static bool atk_ms901m_uart_try_lock(TickType_t wait_ticks);
static void atk_ms901m_request_parser_reset(void);
static void atk_ms901m_parser_reset(atk_ms901m_parser_t *parser);
static bool atk_ms901m_parser_feed(atk_ms901m_parser_t *parser, uint8_t dat, atk_ms901m_frame_t *out);
static uint8_t atk_ms901m_read_reg_by_id(uint8_t id, uint8_t *dat, uint32_t timeout_ms);
static void atk_ms901m_write_reg_by_id(uint8_t id, uint8_t len, const uint8_t *dat);
static void sensorsBiasObjInit(BiasObj *bias);
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut);
#ifdef GYRO_BIAS_LIGHT_WEIGHT
static bool processGyroBiasNoBuffer(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut);
#endif
static bool processAccScale(int16_t ax, int16_t ay, int16_t az);
static void sensorsCalculateVarianceAndMean(BiasObj *bias, Axis3f *varOut, Axis3f *meanOut);
static void sensorsAddBiasValue(BiasObj *bias, int16_t x, int16_t y, int16_t z);
static bool sensorsFindBiasValue(BiasObj *bias);
static void sensorsAccAlignToGravity(Axis3f *in, Axis3f *out);
static void applyAxis3fLpf(lpf2pData *data, Axis3f *in);
static void mapAxesI16(int16_t in_x, int16_t in_y, int16_t in_z, Axis3i16 *out);
static void sensorsTask(void *param);

/**
 * 初始�?ATK-MS901M �?UART 通道�? * 配置波特�?数据格式/引脚/缓冲区，并清空接收缓冲�? */
static void atk_ms901m_uart_init(void)
{
  uart_config_t uart_config = {0};

  // 配置 UART 参数用于 ATK-MS901M 通讯�?
  uart_config.baud_rate = ATK_MS901M_UART_BAUDRATE;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uart_config.source_clk = UART_SCLK_DEFAULT;

  uart_param_config(ATK_MS901M_UART_PORT, &uart_config);
  uart_set_pin(ATK_MS901M_UART_PORT, ATK_MS901M_UART_TX_PIN, ATK_MS901M_UART_RX_PIN,
               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(ATK_MS901M_UART_PORT, ATK_MS901M_UART_RX_BUF_SIZE, 0, 0, NULL, 0);
  uart_flush_input(ATK_MS901M_UART_PORT);
}

static void atk_ms901m_uart_lock(void)
{
  if (atkUartMutex)
  {
    xSemaphoreTake(atkUartMutex, portMAX_DELAY);
  }
}

static void atk_ms901m_uart_unlock(void)
{
  if (atkUartMutex)
  {
    xSemaphoreGive(atkUartMutex);
  }
}

static bool atk_ms901m_uart_try_lock(TickType_t wait_ticks)
{
  if (!atkUartMutex)
  {
    return true;
  }
  return (xSemaphoreTake(atkUartMutex, wait_ticks) == pdTRUE);
}

static void atk_ms901m_request_parser_reset(void)
{
  atkParserResetPending = true;
}

bool sensorsAtkMs901mGetAttitude(attitude_t *attitude)
{
  if ((attitude == NULL) || !attitudeValid)
  {
    return false;
  }

  *attitude = attitudeData;
  return true;
}

bool sensorsAtkMs901mGetQuaternion(quaternion_t *quaternion)
{
  if ((quaternion == NULL) || !quaternionValid)
  {
    return false;
  }

  *quaternion = quaternionData;
  return true;
}

/**
 * 读取陀螺仪数据（非阻塞）�? * 队列有新数据则返�?true，否则返�?false�? */
bool sensorsAtkMs901mReadGyro(Axis3f *gyro)
{
  // 非阻塞读取陀螺仪队列，有新数据则返回 true�?
  return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}

/**
 * 读取加速度计数据（非阻塞）�? * 队列有新数据则返�?true，否则返�?false�? */
bool sensorsAtkMs901mReadAcc(Axis3f *acc)
{
  // 非阻塞读取加速度计队列，有新数据则返�?true�?
  return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}

/**
 * 读取磁力计数据（非阻塞）�? * 队列有新数据则返�?true，否则返�?false�? */
bool sensorsAtkMs901mReadMag(Axis3f *mag)
{
  // 非阻塞读取磁力计队列，有新数据则返回 true�?
  return (pdTRUE == xQueueReceive(magnetometerDataQueue, mag, 0));
}

/**
 * 读取气压计数据（非阻塞）�? * 队列有新数据则返�?true，否则返�?false�? */
bool sensorsAtkMs901mReadBaro(baro_t *baro)
{
  // 非阻塞读取气压计队列，有新数据则返回 true�?
  return (pdTRUE == xQueueReceive(barometerDataQueue, baro, 0));
}

/**
 * 读取当前传感器数据并填充上层结构体�? * tick 参数目前未使用，仅保持接口一致性�? */
void sensorsAtkMs901mAcquire(sensorData_t *sensors, const uint32_t tick)
{
  // 通过统一的读取接口获取各传感器数据�?
  sensorsReadGyro(&sensors->gyro);
  sensorsReadAcc(&sensors->acc);
  sensorsReadMag(&sensors->mag);
  sensorsReadBaro(&sensors->baro);
  sensors->interruptTimestamp = sensorData.interruptTimestamp;
}

/**
 * 判断传感器是否完成标定（当前以陀螺仪零偏为准）�? */
bool sensorsAtkMs901mAreCalibrated(void)
{
  return gyroBiasFound;
}

#if ATK_MS901M_CAL_DEBUG
static void atkCalMaybeLog(void)
{
  if (ATK_MS901M_CAL_DEBUG_INTERVAL_MS == 0)
  {
    return;
  }

  uint32_t now = xTaskGetTickCount();
  if ((now - atkCalLastLogTick) < M2T(ATK_MS901M_CAL_DEBUG_INTERVAL_MS))
  {
    return;
  }

  atkCalLastLogTick = now;
  DEBUG_PRINTI("ATK CAL: rxBytes=%"PRIu32" frames=%"PRIu32" gyroFrames=%"PRIu32" gyroSamples=%"PRIu32" biasFound=%d lastId=0x%02X len=%u accRaw=%d,%d,%d gyroRaw=%d,%d,%d\n",
               atkRxBytes,
               atkRxFrames,
               atkRxGyroFrames,
               gyroBiasSampleCount,
               gyroBiasFound ? 1 : 0,
               atkLastFrameId,
               (unsigned int)atkLastFrameLen,
               (int)atkLastAccRaw.x,
               (int)atkLastAccRaw.y,
               (int)atkLastAccRaw.z,
               (int)atkLastGyroRaw.x,
               (int)atkLastGyroRaw.y,
               (int)atkLastGyroRaw.z);
}
#endif

/**
 * 重置帧解析器状态机�? * 清空状态、索引与校验和，准备解析新帧�? */
static void atk_ms901m_parser_reset(atk_ms901m_parser_t *parser)
{
  parser->state = wait_for_head_l;
  parser->dat_index = 0;
  parser->frame.check_sum = 0;
}

/**
 * 按字节喂入数据并尝试解析出完整帧�? * 返回 true 表示解析到一帧且校验通过，out 填充该帧内容�? */
static bool atk_ms901m_parser_feed(atk_ms901m_parser_t *parser, uint8_t dat, atk_ms901m_frame_t *out)
{
  switch (parser->state)
  {
    case wait_for_head_l:
      // 等待帧头低字�?
      if (dat == ATK_MS901M_FRAME_HEAD_L)
      {
        parser->frame.check_sum = dat;
        parser->state = wait_for_head_h;
      }
      break;
    case wait_for_head_h:
      // 等待帧头高字节（上传/应答�?
      if ((dat == ATK_MS901M_FRAME_HEAD_UPLOAD_H) || (dat == ATK_MS901M_FRAME_HEAD_ACK_H))
      {
        parser->frame.head_h = dat;
        parser->frame.check_sum += dat;
        parser->state = wait_for_id;
      }
      else
      {
        parser->state = wait_for_head_l;
      }
      break;
    case wait_for_id:
      // 读取�?ID
      parser->frame.id = dat;
      parser->frame.check_sum += dat;
      parser->state = wait_for_len;
      break;
    case wait_for_len:
      // 读取数据长度并校验合法�?
      if (dat > ATK_MS901M_FRAME_DAT_MAX_SIZE)
      {
        parser->state = wait_for_head_l;
        break;
      }
      parser->frame.len = dat;
      parser->frame.check_sum += dat;
      if (parser->frame.len == 0)
      {
        parser->state = wait_for_sum;
      }
      else
      {
        parser->dat_index = 0;
        parser->state = wait_for_dat;
      }
      break;
    case wait_for_dat:
      // 读取数据�?
      parser->frame.dat[parser->dat_index] = dat;
      parser->frame.check_sum += dat;
      parser->dat_index++;
      if (parser->dat_index == parser->frame.len)
      {
        parser->state = wait_for_sum;
      }
      break;
    case wait_for_sum:
      // 校验校验�?
      if (dat == parser->frame.check_sum)
      {
        *out = parser->frame;
        atk_ms901m_parser_reset(parser);
        return true;
      }
      atk_ms901m_parser_reset(parser);
      break;
    default:
      atk_ms901m_parser_reset(parser);
      break;
  }

  return false;
}

/**
 * 通过寄存�?ID 读取模块配置�? * 发送读寄存器请求并等待 ACK 帧返回，成功返回长度�? */
static uint8_t atk_ms901m_read_reg_by_id(uint8_t id, uint8_t *dat, uint32_t timeout_ms)
{
  uint8_t buf[6];
  atk_ms901m_parser_t parser = {0};
  atk_ms901m_frame_t frame = {0};
  uint8_t rxbuf[32];
  int64_t deadline = (int64_t)usecTimestamp() + ((int64_t)timeout_ms * 1000);

  buf[0] = ATK_MS901M_FRAME_HEAD_L;
  buf[1] = ATK_MS901M_FRAME_HEAD_ACK_H;
  buf[2] = ATK_MS901M_READ_REG_ID(id);
  buf[3] = 0x01;
  buf[4] = 0x00;
  buf[5] = (uint8_t)(buf[0] + buf[1] + buf[2] + buf[3] + buf[4]);

  // 发送读寄存器请求并等待 ACK 帧�?
  uart_flush_input(ATK_MS901M_UART_PORT);
  uart_write_bytes(ATK_MS901M_UART_PORT, (const char *)buf, sizeof(buf));

  atk_ms901m_parser_reset(&parser);
  while ((int64_t)usecTimestamp() < deadline)
  {
    int len = uart_read_bytes(ATK_MS901M_UART_PORT, rxbuf, sizeof(rxbuf), M2T(10));
    if (len <= 0)
    {
      continue;
    }

    for (int i = 0; i < len; i++)
    {
      if (!atk_ms901m_parser_feed(&parser, rxbuf[i], &frame))
      {
        continue;
      }

#if ATK_MS901M_CAL_DEBUG
      atkRxFrames++;
      atkLastFrameId = frame.id;
      atkLastFrameLen = frame.len;
#endif

      // 只接�?ACK 帧且 ID 匹配
      if (frame.head_h != ATK_MS901M_FRAME_HEAD_ACK_H)
      {
        continue;
      }

      if (frame.id != id)
      {
        continue;
      }

      for (uint8_t j = 0; j < frame.len; j++)
      {
        dat[j] = frame.dat[j];
      }

      return frame.len;
    }
  }

  return 0;
}

/**
 * 通过寄存�?ID 写入模块配置�? * 支持 1~2 字节写入；长度不合法则直接返回�? */
static void atk_ms901m_write_reg_by_id(uint8_t id, uint8_t len, const uint8_t *dat)
{
  uint8_t buf[7];

  if ((len == 0) || (len > 2) || (dat == NULL))
  {
    return;
  }

  // 组包写寄存器帧并计算校验和�?
  buf[0] = ATK_MS901M_FRAME_HEAD_L;
  buf[1] = ATK_MS901M_FRAME_HEAD_ACK_H;
  buf[2] = id;
  buf[3] = len;
  buf[4] = dat[0];
  if (len == 1)
  {
    buf[5] = (uint8_t)(buf[0] + buf[1] + buf[2] + buf[3] + buf[4]);
    uart_write_bytes(ATK_MS901M_UART_PORT, (const char *)buf, 6);
  }
  else
  {
    buf[5] = dat[1];
    buf[6] = (uint8_t)(buf[0] + buf[1] + buf[2] + buf[3] + buf[4] + buf[5]);
    uart_write_bytes(ATK_MS901M_UART_PORT, (const char *)buf, 7);
  }
}

/**
 * 将模块输出的原始坐标映射到系统坐标系�? * 支持 XY 交换�?X 轴反向配置�? */
static void mapAxesI16(int16_t in_x, int16_t in_y, int16_t in_z, Axis3i16 *out)
{
  // 根据编译期配置进行坐标轴交换与方向修正�?
#if ATK_MS901M_AXIS_SWAP_XY
  out->x = in_y;
  out->y = in_x;
#else
  out->x = in_x;
  out->y = in_y;
#endif
  out->z = in_z;

#if ATK_MS901M_AXIS_INVERT_X
  out->x = -out->x;
#endif
}

#if ATK_MS901M_STREAM_1HZ
static void streamMaybePrint(void)
{
  static uint32_t lastStreamMs = 0;
  uint32_t nowMs = (uint32_t)(usecTimestamp() / 1000);

  if (ATK_MS901M_STREAM_INTERVAL_MS != 0)
  {
    if ((nowMs - lastStreamMs) < ATK_MS901M_STREAM_INTERVAL_MS)
    {
      return;
    }
  }
  lastStreamMs = nowMs;

  if (!gyroAccValid && !magValid && !baroValid)
  {
    DEBUG_PRINTI("ATK 1Hz: no data\n");
    return;
  }

  if (gyroAccValid)
  {
    DEBUG_PRINTI("ATK 1Hz: acc[g]=%.3f %.3f %.3f gyro[dps]=%.3f %.3f %.3f\n",
                 sensorData.acc.x, sensorData.acc.y, sensorData.acc.z,
                 sensorData.gyro.x, sensorData.gyro.y, sensorData.gyro.z);
  }
  else
  {
    DEBUG_PRINTI("ATK 1Hz: acc/gyro: n/a\n");
  }

  if (magValid)
  {
    DEBUG_PRINTI("ATK 1Hz: mag[gauss]=%.3f %.3f %.3f\n",
                 sensorData.mag.x, sensorData.mag.y, sensorData.mag.z);
  }

  if (baroValid)
  {
    DEBUG_PRINTI("ATK 1Hz: baro[mbar]=%.2f t=%.2fC alt=%.2fm\n",
                 sensorData.baro.pressure, sensorData.baro.temperature, sensorData.baro.asl);
  }
}
#endif

/**
 * ATK-MS901M 数据解析任务�? * 解析模块上传帧，计算标定与滤波后数据，并写入各传感器队列�? */
static void sensorsTask(void *param)
{
  atk_ms901m_parser_t parser = {0};
  atk_ms901m_frame_t frame = {0};
  uint8_t rxbuf[64];
  Axis3f accScaled;

#if !ATK_MS901M_SKIP_SYSTEM_WAIT
  systemWaitStart();
#endif
  atk_ms901m_parser_reset(&parser);

  while (1)
  {
#if ATK_MS901M_STREAM_1HZ
    streamMaybePrint();
#endif
    if (atkParserResetPending)
    {
      atk_ms901m_parser_reset(&parser);
      atkParserResetPending = false;
    }

    if (!atk_ms901m_uart_try_lock(M2T(1)))
    {
      vTaskDelay(M2T(1));
#if ATK_MS901M_CAL_DEBUG
      atkCalMaybeLog();
#endif
      continue;
    }
    int len = uart_read_bytes(ATK_MS901M_UART_PORT, rxbuf, sizeof(rxbuf), M2T(10));
    atk_ms901m_uart_unlock();
    if (len <= 0)
    {
#if ATK_MS901M_CAL_DEBUG
      atkCalMaybeLog();
#endif
      continue;
    }
#if ATK_MS901M_CAL_DEBUG
    atkRxBytes += (uint32_t)len;
#endif

    for (int i = 0; i < len; i++)
    {
      if (!atk_ms901m_parser_feed(&parser, rxbuf[i], &frame))
      {
        continue;
      }

      // 仅处理模块上传帧�?
      if (frame.head_h != ATK_MS901M_FRAME_HEAD_UPLOAD_H)
      {
        continue;
      }

      isMs901mPresent = true;

      if (frame.id == ATK_MS901M_FRAME_ID_GYRO_ACCE && frame.len >= 12)
      {
        // 解析加速度与陀螺仪数据并更新共享数据�?
        int16_t ax = (int16_t)((frame.dat[1] << 8) | frame.dat[0]);
        int16_t ay = (int16_t)((frame.dat[3] << 8) | frame.dat[2]);
        int16_t az = (int16_t)((frame.dat[5] << 8) | frame.dat[4]);
        int16_t gx = (int16_t)((frame.dat[7] << 8) | frame.dat[6]);
        int16_t gy = (int16_t)((frame.dat[9] << 8) | frame.dat[8]);
        int16_t gz = (int16_t)((frame.dat[11] << 8) | frame.dat[10]);

        mapAxesI16(ax, ay, az, &accelRaw);
        mapAxesI16(gx, gy, gz, &gyroRaw);

#if ATK_MS901M_CAL_DEBUG
        atkRxGyroFrames++;
        atkLastAccRaw = accelRaw;
        atkLastGyroRaw = gyroRaw;
#endif

#ifdef GYRO_BIAS_LIGHT_WEIGHT
        gyroBiasFound = processGyroBiasNoBuffer(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
#else
        gyroBiasFound = processGyroBias(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
#endif

        // 在陀螺仪零偏稳定后计算加速度计比例系数�?
        if (gyroBiasFound)
        {
          processAccScale(accelRaw.x, accelRaw.y, accelRaw.z);
        }

        float gyro_scale = (float)atk_ms901m_gyro_fsr_table[atk_ms901m_fsr.gyro] / 32768.0f;
        float acc_scale = (float)atk_ms901m_acc_fsr_table[atk_ms901m_fsr.accelerometer] / 32768.0f;

        sensorData.gyro.x = (gyroRaw.x - gyroBias.x) * gyro_scale;
        sensorData.gyro.y = (gyroRaw.y - gyroBias.y) * gyro_scale;
        sensorData.gyro.z = (gyroRaw.z - gyroBias.z) * gyro_scale;
        applyAxis3fLpf((lpf2pData *)(&gyroLpf), &sensorData.gyro);

        accScaled.x = accelRaw.x * acc_scale / accScale;
        accScaled.y = accelRaw.y * acc_scale / accScale;
        accScaled.z = accelRaw.z * acc_scale / accScale;
        sensorsAccAlignToGravity(&accScaled, &sensorData.acc);
        applyAxis3fLpf((lpf2pData *)(&accLpf), &sensorData.acc);

        sensorData.interruptTimestamp = usecTimestamp();

        xQueueOverwrite(accelerometerDataQueue, &sensorData.acc);
        xQueueOverwrite(gyroDataQueue, &sensorData.gyro);
        xSemaphoreGive(dataReady);
#if ATK_MS901M_STREAM_1HZ
        gyroAccValid = true;
#endif
      }
      else if (frame.id == ATK_MS901M_FRAME_ID_MAG && frame.len >= 8)
      {
        // 磁力计原始数据，单位�?LSB�?
        Axis3i16 magRaw;
        int16_t mx = (int16_t)((frame.dat[1] << 8) | frame.dat[0]);
        int16_t my = (int16_t)((frame.dat[3] << 8) | frame.dat[2]);
        int16_t mz = (int16_t)((frame.dat[5] << 8) | frame.dat[4]);

        mapAxesI16(mx, my, mz, &magRaw);

        sensorData.mag.x = (float)magRaw.x / MAG_GAUSS_PER_LSB;
        sensorData.mag.y = (float)magRaw.y / MAG_GAUSS_PER_LSB;
        sensorData.mag.z = (float)magRaw.z / MAG_GAUSS_PER_LSB;
        xQueueOverwrite(magnetometerDataQueue, &sensorData.mag);
#if ATK_MS901M_STREAM_1HZ
        magValid = true;
#endif
      }
      else if (frame.id == ATK_MS901M_FRAME_ID_BARO && frame.len >= 10)
      {
        // 气压数据�?2 位气�?高度 + 16 位温度�?
        int32_t pressure = (int32_t)((frame.dat[3] << 24) | (frame.dat[2] << 16) | (frame.dat[1] << 8) | frame.dat[0]);
        int32_t altitude = (int32_t)((frame.dat[7] << 24) | (frame.dat[6] << 16) | (frame.dat[5] << 8) | frame.dat[4]);
        int16_t temp = (int16_t)((frame.dat[9] << 8) | frame.dat[8]);

        sensorData.baro.pressure = (float)pressure / 100.0f;
        sensorData.baro.asl = (float)altitude / 100.0f;
        sensorData.baro.temperature = (float)temp / 100.0f;
        xQueueOverwrite(barometerDataQueue, &sensorData.baro);
#if ATK_MS901M_STREAM_1HZ
        baroValid = true;
#endif
      }
      else if (frame.id == ATK_MS901M_FRAME_ID_ATTITUDE && frame.len >= 6)
      {
        attitudeData.roll = (float)((int16_t)((frame.dat[1] << 8) | frame.dat[0])) / 32768.0f * 180.0f;
        attitudeData.pitch = (float)((int16_t)((frame.dat[3] << 8) | frame.dat[2])) / 32768.0f * 180.0f;
        attitudeData.yaw = (float)((int16_t)((frame.dat[5] << 8) | frame.dat[4])) / 32768.0f * 180.0f;
        attitudeData.timestamp = usecTimestamp();
        attitudeValid = true;
      }
      else if (frame.id == ATK_MS901M_FRAME_ID_QUAT && frame.len >= 8)
      {
        quaternionData.q0 = (float)((int16_t)((frame.dat[1] << 8) | frame.dat[0])) / 32768.0f;
        quaternionData.q1 = (float)((int16_t)((frame.dat[3] << 8) | frame.dat[2])) / 32768.0f;
        quaternionData.q2 = (float)((int16_t)((frame.dat[5] << 8) | frame.dat[4])) / 32768.0f;
        quaternionData.q3 = (float)((int16_t)((frame.dat[7] << 8) | frame.dat[6])) / 32768.0f;
        quaternionData.timestamp = usecTimestamp();
        quaternionValid = true;
      }
    }
#if ATK_MS901M_CAL_DEBUG
    atkCalMaybeLog();
#endif
  }
}

/**
 * 等待新一帧陀螺仪/加速度计数据到达�? * 通过信号量阻塞等待，确保上层同步读取�? */
void sensorsAtkMs901mWaitDataReady(void)
{
  xSemaphoreTake(dataReady, portMAX_DELAY);
}

/**
 * 初始�?ATK-MS901M 传感器接口�? * 完成 UART 初始化、读�?FSR、配置回传、初始化滤波器与任务等�? */
void sensorsAtkMs901mInit(void)
{
  if (isInit)
  {
    return;
  }

  atkUartMutex = xSemaphoreCreateMutexStatic(&atkUartMutexBuffer);

  // 先初始化 UART，确保读写寄存器可用�?
  atk_ms901m_uart_init();
#if ATK_MS901M_BOOT_DELAY_MS > 0
  vTaskDelay(M2T(ATK_MS901M_BOOT_DELAY_MS));
#endif

  sensorsBiasObjInit(&gyroBiasRunning);

  uint8_t fsr_value = 0;
  if (atk_ms901m_read_reg_by_id(ATK_MS901M_FRAME_ID_REG_GYROFSR, &fsr_value, 100))
  {
    atk_ms901m_fsr.gyro = fsr_value & 0x03;
    isMs901mPresent = true;
  }
  else
  {
    DEBUG_PRINTW("ATK-MS901M gyro FSR read failed\n");
  }

  if (atk_ms901m_read_reg_by_id(ATK_MS901M_FRAME_ID_REG_ACCFSR, &fsr_value, 100))
  {
    atk_ms901m_fsr.accelerometer = fsr_value & 0x03;
    isMs901mPresent = true;
  }
  else
  {
    DEBUG_PRINTW("ATK-MS901M acc FSR read failed\n");
  }

#if ATK_MS901M_APPLY_CONFIG
  // 根据编译期配置设置回传内容与回传频率�?
  uint8_t returnset = ATK_MS901M_RETURNSET;
  uint8_t returnrate = ATK_MS901M_RETURNRATE;
  atk_ms901m_write_reg_by_id(ATK_MS901M_FRAME_ID_REG_RETURNSET, 1, &returnset);
  atk_ms901m_write_reg_by_id(ATK_MS901M_FRAME_ID_REG_RETURNRATE, 1, &returnrate);
#endif

  // 按模块采样率初始化各轴低通滤波器�?
  for (uint8_t i = 0; i < 3; i++)
  {
    lpf2pInit(&gyroLpf[i], ATK_MS901M_SAMPLE_RATE_HZ, GYRO_LPF_CUTOFF_FREQ);
    lpf2pInit(&accLpf[i], ATK_MS901M_SAMPLE_RATE_HZ, ACCEL_LPF_CUTOFF_FREQ);
  }

  // 预计算姿态校准所需的三角函数项�?
  cosPitch = cosf(PITCH_CALIB * (float)M_PI / 180);
  sinPitch = sinf(PITCH_CALIB * (float)M_PI / 180);
  cosRoll = cosf(ROLL_CALIB * (float)M_PI / 180);
  sinRoll = sinf(ROLL_CALIB * (float)M_PI / 180);

  accelerometerDataQueue = STATIC_MEM_QUEUE_CREATE(accelerometerDataQueue);
  gyroDataQueue = STATIC_MEM_QUEUE_CREATE(gyroDataQueue);
  magnetometerDataQueue = STATIC_MEM_QUEUE_CREATE(magnetometerDataQueue);
  barometerDataQueue = STATIC_MEM_QUEUE_CREATE(barometerDataQueue);

  dataReady = xSemaphoreCreateBinaryStatic(&dataReadyBuffer);

  STATIC_MEM_TASK_CREATE(sensorsTask, sensorsTask, SENSORS_TASK_NAME, NULL, SENSORS_TASK_PRI);

  isInit = true;
}

/**
 * 传感器自检：初始化完成且检测到模块即可认为通过�? */
bool sensorsAtkMs901mTest(void)
{
  return (isInit && isMs901mPresent);
}

/**
 * 产测用检测：仅判断模块是否存在�? */
bool sensorsAtkMs901mManufacturingTest(void)
{
  return isMs901mPresent;
}

/**
 * 估计加速度计比例系数（标定重力）�? * 对若干样本的加速度模长求平均，得到缩放系数�? */
static bool processAccScale(int16_t ax, int16_t ay, int16_t az)
{
  static bool accBiasFound = false;
  static uint32_t accScaleSumCount = 0;
  float acc_scale = (float)atk_ms901m_acc_fsr_table[atk_ms901m_fsr.accelerometer] / 32768.0f;

  if (!accBiasFound)
  {
    // 累积加速度模长以估算比例系数�?
    accScaleSum += sqrtf(powf(ax * acc_scale, 2) + powf(ay * acc_scale, 2) + powf(az * acc_scale, 2));
    accScaleSumCount++;

    if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES)
    {
      accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
      accBiasFound = true;
    }
  }

  return accBiasFound;
}

#ifdef GYRO_BIAS_LIGHT_WEIGHT
/**
 * 轻量级陀螺仪零偏估计（不使用环形缓冲）�? * 通过累积均值（可选方差）在固定样本数后给出偏置�? */
static bool processGyroBiasNoBuffer(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
  static bool gyroBiasNoBuffFound = false;
  static Axis3i64 gyroBiasSampleSum;
  static Axis3i64 gyroBiasSampleSumSquares;

  if (!gyroBiasNoBuffFound)
  {
    // 累积求和用于估计均值偏置�?
    gyroBiasSampleSum.x += gx;
    gyroBiasSampleSum.y += gy;
    gyroBiasSampleSum.z += gz;
#ifdef SENSORS_GYRO_BIAS_CALCULATE_STDDEV
    gyroBiasSampleSumSquares.x += gx * gx;
      gyroBiasSampleSumSquares.y += gy * gy;
      gyroBiasSampleSumSquares.z += gz * gz;
#endif
      gyroBiasSampleCount += 1;

    if (gyroBiasSampleCount == SENSORS_BIAS_SAMPLES)
    {
      gyroBiasOut->x = (float)(gyroBiasSampleSum.x) / SENSORS_BIAS_SAMPLES;
      gyroBiasOut->y = (float)(gyroBiasSampleSum.y) / SENSORS_BIAS_SAMPLES;
      gyroBiasOut->z = (float)(gyroBiasSampleSum.z) / SENSORS_BIAS_SAMPLES;
      gyroBiasNoBuffFound = true;
    }
  }

  return gyroBiasNoBuffFound;
}
#else
/**
 * 基于滑动窗口的陀螺仪零偏估计�? * 使用方差阈值与时间门限判断零偏稳定�? */
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
  gyroBiasSampleCount += 1;
  // 用环形缓冲统计均�?方差以判断零偏稳定�?
  sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

  if (!gyroBiasRunning.isBiasValueFound)
  {
    sensorsFindBiasValue(&gyroBiasRunning);
  }

  gyroBiasOut->x = gyroBiasRunning.bias.x;
  gyroBiasOut->y = gyroBiasRunning.bias.y;
  gyroBiasOut->z = gyroBiasRunning.bias.z;

  return gyroBiasRunning.isBiasValueFound;
}
#endif

/**
 * 初始化陀螺仪偏置统计结构�? */
static void sensorsBiasObjInit(BiasObj *bias)
{
  // 清空状态并重置环形缓冲头指针�?
  bias->isBufferFilled = false;
  bias->bufHead = bias->buffer;
}

/**
 * 计算环形缓冲中的均值与方差�? */
static void sensorsCalculateVarianceAndMean(BiasObj *bias, Axis3f *varOut, Axis3f *meanOut)
{
  uint32_t i;
  int64_t sum[GYRO_NBR_OF_AXES] = {0};
  int64_t sumSq[GYRO_NBR_OF_AXES] = {0};

  // 逐轴累加样本和与平方和�?
  for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
    sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
    sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
    sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
  }

  varOut->x = (sumSq[0] - ((int64_t)sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
  varOut->y = (sumSq[1] - ((int64_t)sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
  varOut->z = (sumSq[2] - ((int64_t)sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

  meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

/**
 * 向偏置缓冲区写入一个样本（环形缓冲）�? */
static void sensorsAddBiasValue(BiasObj *bias, int16_t x, int16_t y, int16_t z)
{
  // 写入样本并判断是否回绕到缓冲区起始�?
  bias->bufHead->x = x;
  bias->bufHead->y = y;
  bias->bufHead->z = z;
  bias->bufHead++;

  if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES])
  {
    bias->bufHead = bias->buffer;
    bias->isBufferFilled = true;
  }
}

/**
 * 在环形缓冲填满后尝试检测稳定零偏�? * 通过方差阈值与最小时间间隔判断是否有效�? */
static bool sensorsFindBiasValue(BiasObj *bias)
{
  static int32_t varianceSampleTime;
  bool foundBias = false;

  if (bias->isBufferFilled)
  {
    sensorsCalculateVarianceAndMean(bias, &bias->variance, &bias->mean);

    // 只有方差足够小且满足时间间隔才认定偏置稳定�?
    if (bias->variance.x < GYRO_VARIANCE_THRESHOLD_X &&
        bias->variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
        bias->variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
        (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount()))
    {
      varianceSampleTime = xTaskGetTickCount();
      bias->bias.x = bias->mean.x;
      bias->bias.y = bias->mean.y;
      bias->bias.z = bias->mean.z;
      foundBias = true;
      bias->isBiasValueFound = true;
    }
  }

  return foundBias;
}

/**
 * 将加速度向量对齐到重力坐标系�? * 按照 PITCH/ROLL 标定角对加速度进行旋转补偿�? */
static void sensorsAccAlignToGravity(Axis3f *in, Axis3f *out)
{
  Axis3f rx;
  Axis3f ry;

  // 先进�?roll 修正，再进行 pitch 修正�?
  rx.x = in->x;
  rx.y = in->y * cosRoll - in->z * sinRoll;
  rx.z = in->y * sinRoll + in->z * cosRoll;

  ry.x = rx.x * cosPitch - rx.z * sinPitch;
  ry.y = rx.y;
  ry.z = -rx.x * sinPitch + rx.z * cosPitch;

  out->x = ry.x;
  out->y = ry.y;
  out->z = ry.z;
}

/**
 * 设置加速度计工作模式（影响低通滤波截止频率）�? */
void sensorsAtkMs901mSetAccMode(accModes accMode)
{
  switch (accMode)
  {
    case ACC_MODE_PROPTEST:
      // 桨叶测试模式使用更高的截止频率�?
      for (uint8_t i = 0; i < 3; i++)
      {
        lpf2pInit(&accLpf[i], ATK_MS901M_SAMPLE_RATE_HZ, 250);
      }
      break;
    case ACC_MODE_FLIGHT:
    default:
      // 飞行模式使用默认低通滤波截止频率�?
      for (uint8_t i = 0; i < 3; i++)
      {
        lpf2pInit(&accLpf[i], ATK_MS901M_SAMPLE_RATE_HZ, ACCEL_LPF_CUTOFF_FREQ);
      }
      break;
  }
}

/**
 * �?Axis3f 三轴数据应用二阶低通滤波�? */
static void applyAxis3fLpf(lpf2pData *data, Axis3f *in)
{
  // 逐轴就地滤波�?
  for (uint8_t i = 0; i < 3; i++)
  {
    in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
  }
}

bool sensorsAtkMs901mReadReg(uint8_t id, uint8_t *data, uint8_t *len, uint32_t timeout_ms)
{
  uint8_t tmp[ATK_MS901M_FRAME_DAT_MAX_SIZE];

  if (!isInit || (data == NULL) || (len == NULL))
  {
    return false;
  }

  atk_ms901m_uart_lock();
  uint8_t read_len = atk_ms901m_read_reg_by_id(id, tmp, timeout_ms);
  atk_ms901m_request_parser_reset();
  atk_ms901m_uart_unlock();

  if (read_len == 0)
  {
    *len = 0;
    return false;
  }

  if (read_len > ATK_MS901M_FRAME_DAT_MAX_SIZE)
  {
    read_len = ATK_MS901M_FRAME_DAT_MAX_SIZE;
  }

  memcpy(data, tmp, read_len);
  *len = read_len;
  return true;
}

bool sensorsAtkMs901mWriteReg(uint8_t id, const uint8_t *data, uint8_t len)
{
  if (!isInit || (data == NULL) || (len == 0) || (len > 2))
  {
    return false;
  }

  atk_ms901m_uart_lock();
  atk_ms901m_write_reg_by_id(id, len, data);
  atk_ms901m_request_parser_reset();
  atk_ms901m_uart_unlock();
  return true;
}

int sensorsAtkMs901mRawTransfer(const uint8_t *tx, uint8_t tx_len, uint8_t *rx, uint8_t rx_len, uint32_t timeout_ms)
{
  int total = 0;
  int64_t deadline = (int64_t)usecTimestamp() + ((int64_t)timeout_ms * 1000);

  if (!isInit || (rx == NULL) || (rx_len == 0))
  {
    return -1;
  }

  atk_ms901m_uart_lock();
  uart_flush_input(ATK_MS901M_UART_PORT);
  if ((tx != NULL) && (tx_len > 0))
  {
    uart_write_bytes(ATK_MS901M_UART_PORT, (const char *)tx, tx_len);
  }

  while ((total < rx_len) && ((int64_t)usecTimestamp() < deadline))
  {
    int len = uart_read_bytes(ATK_MS901M_UART_PORT, rx + total, rx_len - total, M2T(10));
    if (len > 0)
    {
      total += len;
    }
  }

  atk_ms901m_request_parser_reset();
  atk_ms901m_uart_unlock();
  return total;
}

PARAM_GROUP_START(imu_sensors)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, MS901M, &isMs901mPresent)
PARAM_GROUP_STOP(imu_sensors)

PARAM_GROUP_START(imu_tests)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, MS901M, &isMs901mPresent)
PARAM_GROUP_STOP(imu_tests)



