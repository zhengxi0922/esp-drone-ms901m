/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
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
 * Implements HAL for sensors MPU9250 and LPS25H
 *
 * 2016.06.15: Initial version by Mike Hamer, http://mikehamer.info
 */

#include "sensors_mpu9250_lps25h.h"

#include <math.h>
#include <stm32f4xx.h>

#include "lps25h.h"
#include "mpu6500.h"
#include "ak8963.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "system.h"
#include "configblock.h"
#include "param.h"
#include "log.h"
#include "debug.h"
#include "imu.h"
#include "nvicconf.h"
#include "ledseq.h"
#include "sound.h"
#include "filter.h"
#include "static_mem.h"

/**
 * 启用 250Hz 数字低通滤波模式。但通过 MPU9250 读取多个从设备（磁力计、气压计）
 * 时不工作，只能单独读取，原因不明。
 */
//#define SENSORS_MPU6500_DLPF_256HZ

#define SENSORS_ENABLE_PRESSURE_LPS25H
//#define GYRO_ADD_RAW_AND_VARIANCE_LOG_VALUES

#define SENSORS_ENABLE_MAG_AK8963
#define MAG_GAUSS_PER_LSB     666.7f

#define SENSORS_GYRO_FS_CFG       MPU6500_GYRO_FS_2000
#define SENSORS_DEG_PER_LSB_CFG   MPU6500_DEG_PER_LSB_2000

#define SENSORS_ACCEL_FS_CFG      MPU6500_ACCEL_FS_16
#define SENSORS_G_PER_LSB_CFG     MPU6500_G_PER_LSB_16

#define SENSORS_VARIANCE_MAN_TEST_TIMEOUT M2T(2000) // 超时（毫秒）
#define SENSORS_MAN_TEST_LEVEL_MAX        5.0f      // 最大偏差角度（度）

#define SENSORS_BIAS_SAMPLES       1000
#define SENSORS_ACC_SCALE_SAMPLES  200
#define SENSORS_GYRO_BIAS_CALCULATE_STDDEV

// MPU9250 从设备读取缓冲区长度
#define SENSORS_MPU6500_BUFF_LEN    14
#define SENSORS_MAG_BUFF_LEN        8
#define SENSORS_BARO_BUFF_S_P_LEN   4
#define SENSORS_BARO_BUFF_T_LEN     2
#define SENSORS_BARO_BUFF_LEN       (SENSORS_BARO_BUFF_S_P_LEN + SENSORS_BARO_BUFF_T_LEN)

#define GYRO_NBR_OF_AXES            3
#define GYRO_MIN_BIAS_TIMEOUT_MS    M2T(1*1000)
// 方差计算使用的样本数量，修改会影响阈值
#define SENSORS_NBR_OF_BIAS_SAMPLES     1024
// 陀螺零偏判定的方差阈值
#define GYRO_VARIANCE_BASE          5000
#define GYRO_VARIANCE_THRESHOLD_X   (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y   (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z   (GYRO_VARIANCE_BASE)

typedef struct
{
  Axis3f     bias;
  Axis3f     variance;
  Axis3f     mean;
  bool       isBiasValueFound;
  bool       isBufferFilled;
  Axis3i16*  bufHead;
  Axis3i16   buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
} BiasObj;

static xQueueHandle accelerometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(accelerometerDataQueue, 1, sizeof(Axis3f));
static xQueueHandle gyroDataQueue;
STATIC_MEM_QUEUE_ALLOC(gyroDataQueue, 1, sizeof(Axis3f));
static xQueueHandle magnetometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(magnetometerDataQueue, 1, sizeof(Axis3f));
static xQueueHandle barometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(barometerDataQueue, 1, sizeof(baro_t));

static xSemaphoreHandle sensorsDataReady;
static StaticSemaphore_t sensorsDataReadyBuffer;
static xSemaphoreHandle dataReady;
static StaticSemaphore_t dataReadyBuffer;

static bool isInit = false;
static sensorData_t sensorData;
static volatile uint64_t imuIntTimestamp;

static Axis3i16 gyroRaw;
static Axis3i16 accelRaw;
NO_DMA_CCM_SAFE_ZERO_INIT static BiasObj gyroBiasRunning;
static Axis3f  gyroBias;
#if defined(SENSORS_GYRO_BIAS_CALCULATE_STDDEV) && defined (GYRO_BIAS_LIGHT_WEIGHT)
static Axis3f  gyroBiasStdDev;
#endif
static bool    gyroBiasFound = false;
static float accScaleSum = 0;
static float accScale = 1;

// 低通滤波
#define GYRO_LPF_CUTOFF_FREQ  80
#define ACCEL_LPF_CUTOFF_FREQ 30
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];
static void applyAxis3fLpf(lpf2pData *data, Axis3f* in);

static bool isBarometerPresent = false;
static bool isMagnetometerPresent = false;

static bool isMpu6500TestPassed = false;
static bool isAK8963TestPassed = false;
static bool isLPS25HTestPassed = false;

// 加速度计对齐的预计算值
static float cosPitch;
static float sinPitch;
static float cosRoll;
static float sinRoll;

// 该缓冲区需容纳所有传感器的数据
static uint8_t buffer[SENSORS_MPU6500_BUFF_LEN + SENSORS_MAG_BUFF_LEN + SENSORS_BARO_BUFF_LEN] = {0};

static void processAccGyroMeasurements(const uint8_t *buffer);
static void processMagnetometerMeasurements(const uint8_t *buffer);
static void processBarometerMeasurements(const uint8_t *buffer);
static void sensorsSetupSlaveRead(void);

#ifdef GYRO_GYRO_BIAS_LIGHT_WEIGHT
static bool processGyroBiasNoBuffer(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut);
#else
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz,  Axis3f *gyroBiasOut);
#endif
static bool processAccScale(int16_t ax, int16_t ay, int16_t az);
static void sensorsBiasObjInit(BiasObj* bias);
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut);
static void sensorsCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut);
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z);
static bool sensorsFindBiasValue(BiasObj* bias);
static void sensorsAccAlignToGravity(Axis3f* in, Axis3f* out);

STATIC_MEM_TASK_ALLOC(sensorsTask, SENSORS_TASK_STACKSIZE);

bool sensorsMpu9250Lps25hReadGyro(Axis3f *gyro)
{
  return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}

bool sensorsMpu9250Lps25hReadAcc(Axis3f *acc)
{
  return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}

bool sensorsMpu9250Lps25hReadMag(Axis3f *mag)
{
  return (pdTRUE == xQueueReceive(magnetometerDataQueue, mag, 0));
}

bool sensorsMpu9250Lps25hReadBaro(baro_t *baro)
{
  return (pdTRUE == xQueueReceive(barometerDataQueue, baro, 0));
}

void sensorsMpu9250Lps25hAcquire(sensorData_t *sensors, const uint32_t tick)
{
  sensorsReadGyro(&sensors->gyro);
  sensorsReadAcc(&sensors->acc);
  sensorsReadMag(&sensors->mag);
  sensorsReadBaro(&sensors->baro);
  sensors->interruptTimestamp = sensorData.interruptTimestamp;
}

bool sensorsMpu9250Lps25hAreCalibrated() {
  return gyroBiasFound;
}

static void sensorsTask(void *param)
{
  systemWaitStart();

  sensorsSetupSlaveRead();

  while (1)
  {
    if (pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY))
    {
      sensorData.interruptTimestamp = imuIntTimestamp;
      // 数据已准备好可读取
      uint8_t dataLen = (uint8_t) (SENSORS_MPU6500_BUFF_LEN +
              (isMagnetometerPresent ? SENSORS_MAG_BUFF_LEN : 0) +
              (isBarometerPresent ? SENSORS_BARO_BUFF_LEN : 0));

      i2cdevReadReg8(I2C3_DEV, MPU6500_ADDRESS_AD0_HIGH, MPU6500_RA_ACCEL_XOUT_H, dataLen, buffer);
      // 这些函数处理对应数据并将其写入输出队列
      processAccGyroMeasurements(&(buffer[0]));
      if (isMagnetometerPresent)
      {
          processMagnetometerMeasurements(&(buffer[SENSORS_MPU6500_BUFF_LEN]));
      }
      if (isBarometerPresent)
      {
          processBarometerMeasurements(&(buffer[isMagnetometerPresent ?
                  SENSORS_MPU6500_BUFF_LEN + SENSORS_MAG_BUFF_LEN : SENSORS_MPU6500_BUFF_LEN]));
      }

      xQueueOverwrite(accelerometerDataQueue, &sensorData.acc);
      xQueueOverwrite(gyroDataQueue, &sensorData.gyro);
      if (isMagnetometerPresent)
      {
        xQueueOverwrite(magnetometerDataQueue, &sensorData.mag);
      }
      if (isBarometerPresent)
      {
        xQueueOverwrite(barometerDataQueue, &sensorData.baro);
      }

      // 解锁稳定器任务
      xSemaphoreGive(dataReady);
    }
  }
}

void sensorsMpu9250Lps25hWaitDataReady(void)
{
  xSemaphoreTake(dataReady, portMAX_DELAY);
}

void processBarometerMeasurements(const uint8_t *buffer)
{
  static uint32_t rawPressure = 0;
  static int16_t rawTemp = 0;

  // 检查是否有新的气压更新
  if (buffer[0] & 0x02) {
    rawPressure = ((uint32_t) buffer[3] << 16) | ((uint32_t) buffer[2] << 8) | buffer[1];
  }
  // 检查是否有新的温度更新
  if (buffer[0] & 0x01) {
    rawTemp = ((int16_t) buffer[5] << 8) | buffer[4];
  }

  sensorData.baro.pressure = (float) rawPressure / LPS25H_LSB_PER_MBAR;
  sensorData.baro.temperature = LPS25H_TEMP_OFFSET + ((float) rawTemp / LPS25H_LSB_PER_CELSIUS);
  sensorData.baro.asl = lps25hPressureToAltitude(&sensorData.baro.pressure);
}

void processMagnetometerMeasurements(const uint8_t *buffer)
{
  if (buffer[0] & (1 << AK8963_ST1_DRDY_BIT)) {
    int16_t headingx = (((int16_t) buffer[2]) << 8) | buffer[1];
    int16_t headingy = (((int16_t) buffer[4]) << 8) | buffer[3];
    int16_t headingz = (((int16_t) buffer[6]) << 8) | buffer[5];

    sensorData.mag.x = (float)headingx / MAG_GAUSS_PER_LSB;
    sensorData.mag.y = (float)headingy / MAG_GAUSS_PER_LSB;
    sensorData.mag.z = (float)headingz / MAG_GAUSS_PER_LSB;
  }
}

void processAccGyroMeasurements(const uint8_t *buffer)
{
  Axis3f accScaled;
  // 注意顺序以修正旋转 90º 的 IMU 坐标系
  accelRaw.y = (((int16_t) buffer[0]) << 8) | buffer[1];
  accelRaw.x = (((int16_t) buffer[2]) << 8) | buffer[3];
  accelRaw.z = (((int16_t) buffer[4]) << 8) | buffer[5];
  gyroRaw.y = (((int16_t) buffer[8]) << 8) | buffer[9];
  gyroRaw.x = (((int16_t) buffer[10]) << 8) | buffer[11];
  gyroRaw.z = (((int16_t) buffer[12]) << 8) | buffer[13];


#ifdef GYRO_BIAS_LIGHT_WEIGHT
  gyroBiasFound = processGyroBiasNoBuffer(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
#else
  gyroBiasFound = processGyroBias(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
#endif
  if (gyroBiasFound)
  {
     processAccScale(accelRaw.x, accelRaw.y, accelRaw.z);
  }

  sensorData.gyro.x = -(gyroRaw.x - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;
  sensorData.gyro.y =  (gyroRaw.y - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
  sensorData.gyro.z =  (gyroRaw.z - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;
  applyAxis3fLpf((lpf2pData*)(&gyroLpf), &sensorData.gyro);

  accScaled.x = -(accelRaw.x) * SENSORS_G_PER_LSB_CFG / accScale;
  accScaled.y =  (accelRaw.y) * SENSORS_G_PER_LSB_CFG / accScale;
  accScaled.z =  (accelRaw.z) * SENSORS_G_PER_LSB_CFG / accScale;
  sensorsAccAlignToGravity(&accScaled, &sensorData.acc);
  applyAxis3fLpf((lpf2pData*)(&accLpf), &sensorData.acc);
}

static void sensorsDeviceInit(void)
{
  isMagnetometerPresent = false;
  isBarometerPresent = false;

  // 等待传感器启动
  while (xTaskGetTickCount() < 1000);

  i2cdevInit(I2C3_DEV);
  mpu6500Init(I2C3_DEV);
  if (mpu6500TestConnection() == true)
  {
    DEBUG_PRINT("MPU9250 I2C connection [OK].\n");
  }
  else
  {
    DEBUG_PRINT("MPU9250 I2C connection [FAIL].\n");
  }

  mpu6500Reset();
  vTaskDelay(M2T(50));
  // 激活 MPU6500
  mpu6500SetSleepEnabled(false);
  // 等待寄存器复位完成
  vTaskDelay(M2T(100));
  // 将 X 轴陀螺作为时钟源
  mpu6500SetClockSource(MPU6500_CLOCK_PLL_XGYRO);
  // 等待时钟设置并稳定
  vTaskDelay(M2T(200));
  // 使能温度传感器
  mpu6500SetTempSensorEnabled(true);
  // 禁用中断
  mpu6500SetIntEnabled(false);
  // 将磁力计和气压计连接到主 I2C 总线
  mpu6500SetI2CBypassEnabled(true);
  // 设置陀螺满量程
  mpu6500SetFullScaleGyroRange(SENSORS_GYRO_FS_CFG);
  // 设置加速度计满量程
  mpu6500SetFullScaleAccelRange(SENSORS_ACCEL_FS_CFG);
  // 设置加速度计数字低通带宽
  mpu6500SetAccelDLPF(MPU6500_ACCEL_DLPF_BW_41);

#if SENSORS_MPU6500_DLPF_256HZ
  // 256Hz 数字低通仅适用于振动较小的情况
  // 设置输出率 (15)：8000 / (1 + 7) = 1000Hz
  mpu6500SetRate(7);
  // 设置数字低通带宽
  mpu6500SetDLPFMode(MPU6500_DLPF_BW_256);
#else
  // DLPF 带宽过低可能导致不稳定并降低敏捷性，
  // 但有利于处理振动和螺旋桨不平衡
  // 设置输出率 (1)：1000 / (1 + 0) = 1000Hz
  mpu6500SetRate(0);
  // 设置陀螺数字低通带宽
  mpu6500SetDLPFMode(MPU6500_DLPF_BW_98);
  // 初始化加速度计二阶滤波器
  for (uint8_t i = 0; i < 3; i++)
  {
    lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
    lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
  }
#endif


#ifdef SENSORS_ENABLE_MAG_AK8963
  ak8963Init(I2C3_DEV);
  if (ak8963TestConnection() == true)
  {
    isMagnetometerPresent = true;
    ak8963SetMode(AK8963_MODE_16BIT | AK8963_MODE_CONT2); // 16 位 100Hz
    DEBUG_PRINT("AK8963 I2C connection [OK].\n");
  }
  else
  {
    DEBUG_PRINT("AK8963 I2C connection [FAIL].\n");
  }
#endif

#ifdef SENSORS_ENABLE_PRESSURE_LPS25H
  lps25hInit(I2C3_DEV);
  if (lps25hTestConnection() == true)
  {
    lps25hSetEnabled(true);
    isBarometerPresent = true;
    DEBUG_PRINT("LPS25H I2C connection [OK].\n");
  }
  else
  {
    // TODO：无连接时是否应让传感器测试失败并退出
    DEBUG_PRINT("LPS25H I2C connection [FAIL].\n");
  }
#endif

  cosPitch = cosf(configblockGetCalibPitch() * (float) M_PI/180);
  sinPitch = sinf(configblockGetCalibPitch() * (float) M_PI/180);
  cosRoll = cosf(configblockGetCalibRoll() * (float) M_PI/180);
  sinRoll = sinf(configblockGetCalibRoll() * (float) M_PI/180);
}


static void sensorsSetupSlaveRead(void)
{
  // 开始配置从设备
#ifdef SENSORS_MPU6500_DLPF_256HZ
  // 根据寄存器手册 4.4：“数据应以不低于采样率采样；
  // SMPLRT_DIV 仅用于 1kHz 内部采样。” 最慢更新率为 500Hz。
  mpu6500SetSlave4MasterDelay(15); // 读取从设备 500Hz = (8000Hz / (1 + 15))
#else
  mpu6500SetSlave4MasterDelay(9); // 读取从设备 100Hz = (500Hz / (1 + 4))
#endif

  mpu6500SetI2CBypassEnabled(false);
  mpu6500SetWaitForExternalSensorEnabled(true); // 从设备数据对状态估计不太关键
  mpu6500SetInterruptMode(0); // 高电平有效
  mpu6500SetInterruptDrive(0); // 推挽输出
  mpu6500SetInterruptLatch(0); // 直到清除才解除锁存
  mpu6500SetInterruptLatchClear(1); // 任意寄存器读清除
  mpu6500SetSlaveReadWriteTransitionEnabled(false); // 从设备读取结束发送停止位
  mpu6500SetMasterClockSpeed(13); // 设置 I2C 速度为 400kHz

#ifdef SENSORS_ENABLE_MAG_AK8963
  if (isMagnetometerPresent)
  {
    // 设置 MPU6500 主机端需要读取的寄存器
    mpu6500SetSlaveAddress(0, 0x80 | AK8963_ADDRESS_00); // 磁力计设为从设备 0，启用读
    mpu6500SetSlaveRegister(0, AK8963_RA_ST1); // 读取磁力计数据寄存器
    mpu6500SetSlaveDataLength(0, SENSORS_MAG_BUFF_LEN); // 读取 8 字节（ST1、x、y、z、ST2（溢出检查））
    mpu6500SetSlaveDelayEnabled(0, true);
    mpu6500SetSlaveEnabled(0, true);
  }
#endif

#ifdef SENSORS_ENABLE_PRESSURE_LPS25H
  if (isBarometerPresent)
  {
    // 将 LPS25H 配置为从设备并启用读取
    // 通过两次读取可兼容 LPS25H FIFO 平均滤波，
    // 自动递增在读取 LPS25H_PRESS_OUT_H 后回绕到 LPS25H_PRESS_OUT_L。
    mpu6500SetSlaveAddress(1, 0x80 | LPS25H_I2C_ADDR);
    mpu6500SetSlaveRegister(1, LPS25H_STATUS_REG | LPS25H_ADDR_AUTO_INC);
    mpu6500SetSlaveDataLength(1, SENSORS_BARO_BUFF_S_P_LEN);
    mpu6500SetSlaveDelayEnabled(1, true);
    mpu6500SetSlaveEnabled(1, true);

    mpu6500SetSlaveAddress(2, 0x80 | LPS25H_I2C_ADDR);
    mpu6500SetSlaveRegister(2, LPS25H_TEMP_OUT_L | LPS25H_ADDR_AUTO_INC);
    mpu6500SetSlaveDataLength(2, SENSORS_BARO_BUFF_T_LEN);
    mpu6500SetSlaveDelayEnabled(2, true);
    mpu6500SetSlaveEnabled(2, true);
  }
#endif

  // 配置完成后启用传感器
  mpu6500SetI2CMasterModeEnabled(true);

  mpu6500SetIntDataReadyEnabled(true);
}

static void sensorsTaskInit(void)
{
  accelerometerDataQueue = STATIC_MEM_QUEUE_CREATE(accelerometerDataQueue);
  gyroDataQueue = STATIC_MEM_QUEUE_CREATE(gyroDataQueue);
  magnetometerDataQueue = STATIC_MEM_QUEUE_CREATE(magnetometerDataQueue);
  barometerDataQueue = STATIC_MEM_QUEUE_CREATE(barometerDataQueue);

  STATIC_MEM_TASK_CREATE(sensorsTask, sensorsTask, SENSORS_TASK_NAME, NULL, SENSORS_TASK_PRI);
}

static void sensorsInterruptInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  sensorsDataReady = xSemaphoreCreateBinaryStatic(&sensorsDataReadyBuffer);
  dataReady = xSemaphoreCreateBinaryStatic(&dataReadyBuffer);

  // FSYNC “不可悬空，必须由 MCU 拉高或拉低”
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOC, GPIO_Pin_14);

  // 在 PC13 上启用 MPU6500 中断
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);

  EXTI_InitStructure.EXTI_Line = EXTI_Line13;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  portDISABLE_INTERRUPTS();
  EXTI_Init(&EXTI_InitStructure);
  EXTI_ClearITPendingBit(EXTI_Line13);
  portENABLE_INTERRUPTS();
}

void sensorsMpu9250Lps25hInit(void)
{
  if (isInit)
  {
    return;
  }

  sensorsBiasObjInit(&gyroBiasRunning);
  sensorsDeviceInit();
  sensorsInterruptInit();
  sensorsTaskInit();

  isInit = true;
}

bool sensorsMpu9250Lps25hTest(void)
{
  bool testStatus = true;

  if (!isInit)
  {
    DEBUG_PRINT("Error while initializing sensor task\r\n");
    testStatus = false;
  }

  // 尝试 3 秒，让飞控稳定后再通过测试
  for (int i = 0; i < 300; i++)
  {
    if(mpu6500SelfTest() == true)
    {
      isMpu6500TestPassed = true;
      break;
    }
    else
    {
      vTaskDelay(M2T(10));
    }
  }
  testStatus &= isMpu6500TestPassed;

#ifdef SENSORS_ENABLE_MAG_AK8963
  testStatus &= isMagnetometerPresent;
  if (testStatus)
  {
    isAK8963TestPassed = ak8963SelfTest();
    testStatus = isAK8963TestPassed;
  }
#endif

#ifdef SENSORS_ENABLE_PRESSURE_LPS25H
  testStatus &= isBarometerPresent;
  if (testStatus)
  {
    isLPS25HTestPassed = lps25hSelfTest();
    testStatus = isLPS25HTestPassed;
  }
#endif

  return testStatus;
}

/**
 * 通过 SENSORS_ACC_SCALE_SAMPLES 个样本计算加速度计比例系数。
 * 应在平台稳定时调用。
 */
static bool processAccScale(int16_t ax, int16_t ay, int16_t az)
{
  static bool accBiasFound = false;
  static uint32_t accScaleSumCount = 0;

  if (!accBiasFound)
  {
    accScaleSum += sqrtf(powf(ax * SENSORS_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_G_PER_LSB_CFG, 2) + powf(az * SENSORS_G_PER_LSB_CFG, 2));
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
 * 基于前 SENSORS_BIAS_SAMPLES 个样本计算偏置，不需要缓冲区，
 * 但要求启动时平台保持稳定。
 */
static bool processGyroBiasNoBuffer(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
  static uint32_t gyroBiasSampleCount = 0;
  static bool gyroBiasNoBuffFound = false;
  static Axis3i64 gyroBiasSampleSum;
  static Axis3i64 gyroBiasSampleSumSquares;

  if (!gyroBiasNoBuffFound)
  {
    // 如果陀螺尚未校准：
    // 将当前样本累加到运行均值和方差
    gyroBiasSampleSum.x += gx;
    gyroBiasSampleSum.y += gy;
    gyroBiasSampleSum.z += gz;
#ifdef SENSORS_GYRO_BIAS_CALCULATE_STDDEV
    gyroBiasSampleSumSquares.x += gx * gx;
    gyroBiasSampleSumSquares.y += gy * gy;
    gyroBiasSampleSumSquares.z += gz * gz;
#endif
    gyroBiasSampleCount += 1;

    // 如果样本数量足够，计算均值与标准差
    if (gyroBiasSampleCount == SENSORS_BIAS_SAMPLES)
    {
      gyroBiasOut->x = (float)(gyroBiasSampleSum.x) / SENSORS_BIAS_SAMPLES;
      gyroBiasOut->y = (float)(gyroBiasSampleSum.y) / SENSORS_BIAS_SAMPLES;
      gyroBiasOut->z = (float)(gyroBiasSampleSum.z) / SENSORS_BIAS_SAMPLES;

#ifdef SENSORS_GYRO_BIAS_CALCULATE_STDDEV
      gyroBiasStdDev.x = sqrtf((float)(gyroBiasSampleSumSquares.x) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->x * gyroBiasOut->x));
      gyroBiasStdDev.y = sqrtf((float)(gyroBiasSampleSumSquares.y) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->y * gyroBiasOut->y));
      gyroBiasStdDev.z = sqrtf((float)(gyroBiasSampleSumSquares.z) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->z * gyroBiasOut->z));
#endif
      gyroBiasNoBuffFound = true;
    }
  }

  return gyroBiasNoBuffFound;
}
#else
/**
 * 当陀螺方差低于阈值时计算偏置，需要缓冲区，
 * 并在平台稳定后进行校准。
 */
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
  sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

  if (!gyroBiasRunning.isBiasValueFound)
  {
    sensorsFindBiasValue(&gyroBiasRunning);
    if (gyroBiasRunning.isBiasValueFound)
    {
      soundSetEffect(SND_CALIB);
      ledseqRun(&seq_calibrated);
    }
  }

  gyroBiasOut->x = gyroBiasRunning.bias.x;
  gyroBiasOut->y = gyroBiasRunning.bias.y;
  gyroBiasOut->z = gyroBiasRunning.bias.z;

  return gyroBiasRunning.isBiasValueFound;
}
#endif

static void sensorsBiasObjInit(BiasObj* bias)
{
  bias->isBufferFilled = false;
  bias->bufHead = bias->buffer;
}

/**
 * 计算偏置缓冲区的方差与均值。
 */
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut)
{
  uint32_t i;
  int64_t sum[GYRO_NBR_OF_AXES] = {0};
  int64_t sumSq[GYRO_NBR_OF_AXES] = {0};

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
 * 计算偏置缓冲区的均值。
 */
static void __attribute__((used)) sensorsCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut)
{
  uint32_t i;
  int32_t sum[GYRO_NBR_OF_AXES] = {0};

  for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
  }

  meanOut->x = sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->y = sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->z = sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

/**
 * 将新值加入方差缓冲区，满时替换最旧数据，
 * 即为环形缓冲区。
 */
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z)
{
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
 * 检查方差是否低于预设阈值。
 * 调用前应已将偏置值写入。
 * @param bias  偏置对象
 */
static bool sensorsFindBiasValue(BiasObj* bias)
{
  static int32_t varianceSampleTime;
  bool foundBias = false;

  if (bias->isBufferFilled)
  {
    sensorsCalculateVarianceAndMean(bias, &bias->variance, &bias->mean);

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

bool sensorsMpu9250Lps25hManufacturingTest(void)
{
  bool testStatus = false;
  Axis3i16 g;
  Axis3i16 a;
  Axis3f acc;  // 加速度计轴向数据（mG）
  float pitch, roll;
  uint32_t startTick = xTaskGetTickCount();

  testStatus = mpu6500SelfTest();

  if (testStatus)
  {
    sensorsBiasObjInit(&gyroBiasRunning);
    while (xTaskGetTickCount() - startTick < SENSORS_VARIANCE_MAN_TEST_TIMEOUT)
    {
      mpu6500GetMotion6(&a.y, &a.x, &a.z, &g.y, &g.x, &g.z);

      if (processGyroBias(g.x, g.y, g.z, &gyroBias))
      {
        gyroBiasFound = true;
        DEBUG_PRINT("Gyro variance test [OK]\n");
        break;
      }
    }

    if (gyroBiasFound)
    {
      acc.x = -(a.x) * SENSORS_G_PER_LSB_CFG;
      acc.y =  (a.y) * SENSORS_G_PER_LSB_CFG;
      acc.z =  (a.z) * SENSORS_G_PER_LSB_CFG;

      // 基于加速度计计算俯仰与横滚，要求板子水平
      pitch = tanf(-acc.x/(sqrtf(acc.y*acc.y + acc.z*acc.z))) * 180/(float) M_PI;
      roll = tanf(acc.y/acc.z) * 180/(float) M_PI;

      if ((fabsf(roll) < SENSORS_MAN_TEST_LEVEL_MAX) && (fabsf(pitch) < SENSORS_MAN_TEST_LEVEL_MAX))
      {
        DEBUG_PRINT("Acc level test [OK]\n");
        testStatus = true;
      }
      else
      {
        DEBUG_PRINT("Acc level test Roll:%0.2f, Pitch:%0.2f [FAIL]\n", (double)roll, (double)pitch);
        testStatus = false;
      }
    }
    else
    {
      DEBUG_PRINT("Gyro variance test [FAIL]\n");
      testStatus = false;
    }
  }

  return testStatus;
}

void __attribute__((used)) EXTI13_Callback(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  imuIntTimestamp = usecTimestamp();
  xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken)
  {
    portYIELD();
  }
}

/**
 * 补偿未对齐的加速度计。使用 UI 采集并写入配置块的校准数据，
 * 旋转加速度计以对齐重力方向。
 */
static void sensorsAccAlignToGravity(Axis3f* in, Axis3f* out)
{
  Axis3f rx;
  Axis3f ry;

  // 绕 x 轴旋转
  rx.x = in->x;
  rx.y = in->y * cosRoll - in->z * sinRoll;
  rx.z = in->y * sinRoll + in->z * cosRoll;

  // 绕 y 轴旋转
  ry.x = rx.x * cosPitch - rx.z * sinPitch;
  ry.y = rx.y;
  ry.z = -rx.x * sinPitch + rx.z * cosPitch;

  out->x = ry.x;
  out->y = ry.y;
  out->z = ry.z;
}

void sensorsMpu9250Lps25hSetAccMode(accModes accMode)
{
  switch (accMode)
  {
    case ACC_MODE_PROPTEST:
      mpu6500SetAccelDLPF(MPU6500_ACCEL_DLPF_BW_460);
      for (uint8_t i = 0; i < 3; i++)
      {
        lpf2pInit(&accLpf[i],  1000, 500);
      }
      break;
    case ACC_MODE_FLIGHT:
    default:
      mpu6500SetAccelDLPF(MPU6500_ACCEL_DLPF_BW_41);
      for (uint8_t i = 0; i < 3; i++)
      {
        lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
      }
      break;
  }
}

static void applyAxis3fLpf(lpf2pData *data, Axis3f* in)
{
  for (uint8_t i = 0; i < 3; i++) {
    in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
  }
}

#ifdef GYRO_ADD_RAW_AND_VARIANCE_LOG_VALUES
LOG_GROUP_START(gyro)
LOG_ADD(LOG_INT16, xRaw, &gyroRaw.x)
LOG_ADD(LOG_INT16, yRaw, &gyroRaw.y)
LOG_ADD(LOG_INT16, zRaw, &gyroRaw.z)
LOG_ADD(LOG_FLOAT, xVariance, &gyroBiasRunning.variance.x)
LOG_ADD(LOG_FLOAT, yVariance, &gyroBiasRunning.variance.y)
LOG_ADD(LOG_FLOAT, zVariance, &gyroBiasRunning.variance.z)
LOG_GROUP_STOP(gyro)
#endif

PARAM_GROUP_START(imu_sensors)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, HMC5883L, &isMagnetometerPresent)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, MS5611, &isBarometerPresent) // TODO：将 MS5611 重命名为 LPS25H，客户端需同步更新
PARAM_GROUP_STOP(imu_sensors)

PARAM_GROUP_START(imu_tests)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, MPU6500, &isMpu6500TestPassed)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, HMC5883L, &isAK8963TestPassed)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, MS5611, &isLPS25HTestPassed) // TODO：将 MS5611 重命名为 LPS25H，客户端需同步更新
PARAM_GROUP_STOP(imu_tests)
