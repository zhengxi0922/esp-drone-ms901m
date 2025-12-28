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
 * Implements HAL for sensors MPU9250 and LPS25H
 *
 * 2016.06.15: Initial version by Mike Hamer, http://mikehamer.info
 */
#include <math.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "queue.h"
#include "projdefs.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#include "sensors_mpu6050_hm5883L_ms5611.h"
#include "system.h"
#include "configblock.h"
#include "param.h"
#include "log.h"

#include "imu.h"
#include "nvicconf.h"
#include "ledseq.h"
#include "sound.h"
#include "filter.h"
#include "config.h"
#include "stm32_legacy.h"

#include "i2cdev.h"
// #include "lps25h.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "ms5611.h"
// #include "ak8963.h"
#include "zranger.h"
#include "zranger2.h"
#include "vl53l1x.h"
#include "flowdeck_v1v2.h"
#define DEBUG_MODULE "SENSORS"
#include "debug_cf.h"
#include "static_mem.h"
#include "crtp_commander.h"

/**
 * 启用 250Hz 数字低通滤波模式。但通过 MPU9250 读取多个从设备（磁力计、气压计）
 * 时不工作，只能单独读取，原因不明。
 */
//#define SENSORS_mpu6050_DLPF_256HZ

//#define GYRO_ADD_RAW_AND_VARIANCE_LOG_VALUES

#define MAG_GAUSS_PER_LSB 666.7f

/**
 * 启用板载传感器
 */
// #define SENSORS_ENABLE_MAG_HM5883L
// #define SENSORS_ENABLE_PRESSURE_MS5611
//#define SENSORS_ENABLE_RANGE_VL53L0X
#define SENSORS_ENABLE_RANGE_VL53L1X
#define SENSORS_ENABLE_FLOW_PMW3901

#define SENSORS_GYRO_FS_CFG MPU6050_GYRO_FS_2000
#define SENSORS_DEG_PER_LSB_CFG MPU6050_DEG_PER_LSB_2000

#define SENSORS_ACCEL_FS_CFG MPU6050_ACCEL_FS_16
#define SENSORS_G_PER_LSB_CFG MPU6050_G_PER_LSB_16

#define SENSORS_VARIANCE_MAN_TEST_TIMEOUT M2T(2000) // 超时（毫秒）
#define SENSORS_MAN_TEST_LEVEL_MAX 5.0f             // 最大偏差角度（度）

#define SENSORS_BIAS_SAMPLES 1000
#define SENSORS_ACC_SCALE_SAMPLES 200
#define SENSORS_GYRO_BIAS_CALCULATE_STDDEV

// MPU9250 从设备读取缓冲区长度
#define GPIO_INTA_MPU6050_IO CONFIG_MPU_PIN_INT
#define SENSORS_MPU6050_BUFF_LEN 14
#define SENSORS_MAG_BUFF_LEN 8
#define SENSORS_BARO_BUFF_S_P_LEN MS5611_D1D2_SIZE
#define SENSORS_BARO_BUFF_T_LEN MS5611_D1D2_SIZE
#define SENSORS_BARO_BUFF_LEN (SENSORS_BARO_BUFF_S_P_LEN + SENSORS_BARO_BUFF_T_LEN)

#define GYRO_NBR_OF_AXES 3
#define GYRO_MIN_BIAS_TIMEOUT_MS M2T(1 * 1000)
// 方差计算使用的样本数量，修改会影响阈值
#define SENSORS_NBR_OF_BIAS_SAMPLES 1024
// 陀螺零偏判定的方差阈值
#define GYRO_VARIANCE_BASE 5000
#define GYRO_VARIANCE_THRESHOLD_X (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z (GYRO_VARIANCE_BASE)
#define ESP_INTR_FLAG_DEFAULT 0

#define PITCH_CALIB (CONFIG_PITCH_CALIB*1.0/100)
#define ROLL_CALIB (CONFIG_ROLL_CALIB*1.0/100)

typedef struct {
    Axis3f bias;
    Axis3f variance;
    Axis3f mean;
    bool isBiasValueFound;
    bool isBufferFilled;
    Axis3i16 *bufHead;
    Axis3i16 buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
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
static xSemaphoreHandle dataReady;

static bool isInit = false;
static sensorData_t sensorData;
static volatile uint64_t imuIntTimestamp;

static Axis3i16 gyroRaw;
static Axis3i16 accelRaw;
static BiasObj gyroBiasRunning;
static Axis3f gyroBias;
#if defined(SENSORS_GYRO_BIAS_CALCULATE_STDDEV) && defined(GYRO_BIAS_LIGHT_WEIGHT)
static Axis3f gyroBiasStdDev;
#endif
static bool gyroBiasFound = false;
static float accScaleSum = 0;
static float accScale = 1;

// 低通滤波
#define GYRO_LPF_CUTOFF_FREQ 80
#define ACCEL_LPF_CUTOFF_FREQ 30
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];
static void applyAxis3fLpf(lpf2pData *data, Axis3f *in);

static bool isBarometerPresent = false;
static bool isMagnetometerPresent = false;
#ifdef SENSORS_ENABLE_RANGE_VL53L1X
static bool isVl53l1xPresent = false;
#endif
#ifdef SENSORS_ENABLE_RANGE_VL53L0X
static bool isVl53l0xPresent = false;
#endif
#ifdef SENSORS_ENABLE_FLOW_PMW3901
static bool isPmw3901Present = false;
#endif
static bool isMpu6050TestPassed = false;
#ifdef SENSORS_ENABLE_MAG_HM5883L
static bool isHmc5883lTestPassed = false;
#endif
#ifdef SENSORS_ENABLE_PRESSURE_MS5611
static bool isMs5611TestPassed = false;
#endif

// 加速度计对齐的预计算值
float cosPitch;
float sinPitch;
float cosRoll;
float sinRoll;

// 该缓冲区需容纳所有传感器的数据
static uint8_t buffer[SENSORS_MPU6050_BUFF_LEN + SENSORS_MAG_BUFF_LEN + SENSORS_BARO_BUFF_LEN] = {0};

static void processAccGyroMeasurements(const uint8_t *buffer);
static void processMagnetometerMeasurements(const uint8_t *buffer);
static void processBarometerMeasurements(const uint8_t *buffer);
static void sensorsSetupSlaveRead(void);

#ifdef GYRO_GYRO_BIAS_LIGHT_WEIGHT
static bool processGyroBiasNoBuffer(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut);
#else
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut);
#endif
static bool processAccScale(int16_t ax, int16_t ay, int16_t az);
static void sensorsBiasObjInit(BiasObj *bias);
static void sensorsCalculateVarianceAndMean(BiasObj *bias, Axis3f *varOut, Axis3f *meanOut);
static void sensorsCalculateBiasMean(BiasObj *bias, Axis3i32 *meanOut);
static void sensorsAddBiasValue(BiasObj *bias, int16_t x, int16_t y, int16_t z);
static bool sensorsFindBiasValue(BiasObj *bias);
static void sensorsAccAlignToGravity(Axis3f *in, Axis3f *out);

STATIC_MEM_TASK_ALLOC(sensorsTask, SENSORS_TASK_STACKSIZE);
bool sensorsMpu6050Hmc5883lMs5611ReadGyro(Axis3f *gyro)
{
    return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}

bool sensorsMpu6050Hmc5883lMs5611ReadAcc(Axis3f *acc)
{
    return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}

bool sensorsMpu6050Hmc5883lMs5611ReadMag(Axis3f *mag)
{
    return (pdTRUE == xQueueReceive(magnetometerDataQueue, mag, 0));
}

bool sensorsMpu6050Hmc5883lMs5611ReadBaro(baro_t *baro)
{
    return (pdTRUE == xQueueReceive(barometerDataQueue, baro, 0));
}

void sensorsMpu6050Hmc5883lMs5611Acquire(sensorData_t *sensors, const uint32_t tick)
{
    sensorsReadGyro(&sensors->gyro);
    sensorsReadAcc(&sensors->acc);
    sensorsReadMag(&sensors->mag);
    sensorsReadBaro(&sensors->baro);
    sensors->interruptTimestamp = sensorData.interruptTimestamp;
}

bool sensorsMpu6050Hmc5883lMs5611AreCalibrated()
{
    return gyroBiasFound;
}

static void sensorsTask(void *param)
{
    // TODO：
    systemWaitStart();
    vTaskDelay(M2T(200));
    DEBUG_PRINTD("xTaskCreate sensorsTask IN");
    sensorsSetupSlaveRead(); //
    DEBUG_PRINTD("xTaskCreate sensorsTask SetupSlave done");

    while (1) {

        /* mpu6050 中断触发：数据已准备好读取 */
        if (pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY)) {
            sensorData.interruptTimestamp = imuIntTimestamp;

            /* 传感器步骤 1：从 I2C 读取数据 */
            uint8_t dataLen = (uint8_t)(SENSORS_MPU6050_BUFF_LEN +
                                        (isMagnetometerPresent ? SENSORS_MAG_BUFF_LEN : 0) +
                                        (isBarometerPresent ? SENSORS_BARO_BUFF_LEN : 0));
            i2cdevReadReg8(I2C0_DEV, MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_ACCEL_XOUT_H, dataLen, buffer);

            /* 传感器步骤 2：处理对应数据 */
            processAccGyroMeasurements(&(buffer[0]));

            if (isMagnetometerPresent) {
                processMagnetometerMeasurements(&(buffer[SENSORS_MPU6050_BUFF_LEN]));
            }

            if (isBarometerPresent) {
                processBarometerMeasurements(&(buffer[isMagnetometerPresent ? SENSORS_MPU6050_BUFF_LEN + SENSORS_MAG_BUFF_LEN : SENSORS_MPU6050_BUFF_LEN]));
            }

            /* 传感器步骤 3：将数据写入输出队列 */
            xQueueOverwrite(accelerometerDataQueue, &sensorData.acc);
            xQueueOverwrite(gyroDataQueue, &sensorData.gyro);

            if (isMagnetometerPresent) {
                xQueueOverwrite(magnetometerDataQueue, &sensorData.mag);
            }

            if (isBarometerPresent) {
                xQueueOverwrite(barometerDataQueue, &sensorData.baro);
            }

            /* 传感器步骤 4：解锁稳定器任务 */
            xSemaphoreGive(dataReady);
#ifdef DEBUG_EP2
            DEBUG_PRINT_LOCAL("ax = %f,  ay = %f,  az = %f,  gx = %f,  gy = %f,  gz = %f , hx = %f , hy = %f, hz =%f \n", sensorData.acc.x, sensorData.acc.y, sensorData.acc.z, sensorData.gyro.x, sensorData.gyro.y, sensorData.gyro.z, sensorData.mag.x, sensorData.mag.y, sensorData.mag.z);
#endif
        }
    }
}

void sensorsMpu6050Hmc5883lMs5611WaitDataReady(void)
{
    xSemaphoreTake(dataReady, portMAX_DELAY);
}

void processBarometerMeasurements(const uint8_t *buffer)
{
    // TODO：替换为 MS5611
    DEBUG_PRINTW("processBarometerMeasurements NEED TODO");
//   static uint32_t rawPressure = 0;
//   static int16_t rawTemp = 0;

// 检查是否有新的气压更新

// 检查是否有新的温度更新

//   sensorData.baro.pressure = (float) rawPressure / LPS25H_LSB_PER_MBAR;
//   sensorData.baro.temperature = LPS25H_TEMP_OFFSET + ((float) rawTemp / LPS25H_LSB_PER_CELSIUS);
//   sensorData.baro.asl = lps25hPressureToAltitude(&sensorData.baro.pressure);
}

void processMagnetometerMeasurements(const uint8_t *buffer)
{
    // TODO：替换为 hmc5883l
    if (buffer[7] & (1 << HMC5883L_STATUS_READY_BIT)) {
        int16_t headingx = (((int16_t)buffer[2]) << 8) | buffer[1]; // hmc5883 与其他不同
        int16_t headingz = (((int16_t)buffer[4]) << 8) | buffer[3];
        int16_t headingy = (((int16_t)buffer[6]) << 8) | buffer[5];

        sensorData.mag.x = (float)headingx / MAG_GAUSS_PER_LSB; // 转为高斯
        sensorData.mag.y = (float)headingy / MAG_GAUSS_PER_LSB;
        sensorData.mag.z = (float)headingz / MAG_GAUSS_PER_LSB;
        DEBUG_PRINTI("hmc5883l DATA ready");
    } else {

        DEBUG_PRINTW("hmc5883l DATA not ready");
    }
}

void processAccGyroMeasurements(const uint8_t *buffer)
{
    /* 注意顺序以修正旋转 90º 的 IMU 坐标系 */

    Axis3f accScaled;

#ifdef CONFIG_TARGET_ESPLANE_V1
    /* 传感器步骤 2.1：从缓冲区读取 */
    accelRaw.x = (((int16_t)buffer[0]) << 8) | buffer[1];
    accelRaw.y = (((int16_t)buffer[2]) << 8) | buffer[3];
    accelRaw.z = (((int16_t)buffer[4]) << 8) | buffer[5];
    gyroRaw.x = (((int16_t)buffer[8]) << 8) | buffer[9];
    gyroRaw.y = (((int16_t)buffer[10]) << 8) | buffer[11];
    gyroRaw.z = (((int16_t)buffer[12]) << 8) | buffer[13];
#else
    /* 传感器步骤 2.1：从缓冲区读取 */
    accelRaw.y = (((int16_t)buffer[0]) << 8) | buffer[1];
    accelRaw.x = (((int16_t)buffer[2]) << 8) | buffer[3];
    accelRaw.z = (((int16_t)buffer[4]) << 8) | buffer[5];
    gyroRaw.y = (((int16_t)buffer[8]) << 8) | buffer[9];
    gyroRaw.x = (((int16_t)buffer[10]) << 8) | buffer[11];
    gyroRaw.z = (((int16_t)buffer[12]) << 8) | buffer[13];
#endif

#ifdef GYRO_BIAS_LIGHT_WEIGHT
    gyroBiasFound = processGyroBiasNoBuffer(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
#else
    /* 传感器步骤 2.2：当方差低于阈值时计算陀螺偏置 */
    gyroBiasFound = processGyroBias(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
#endif

    /* 传感器步骤 2.3：平台稳定时计算加速度计比例 */
    if (gyroBiasFound) {
        processAccScale(accelRaw.x, accelRaw.y, accelRaw.z);
    }

    /* 传感器步骤 2.4：将数字值转换为物理角度 */
#ifdef CONFIG_TARGET_ESPLANE_V1
    sensorData.gyro.x = (gyroRaw.x - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;
#else
    sensorData.gyro.x = -(gyroRaw.x - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;
#endif

    sensorData.gyro.y = (gyroRaw.y - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
    sensorData.gyro.z = (gyroRaw.z - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;
    /* 传感器步骤 2.5：低通滤波 */
    applyAxis3fLpf((lpf2pData *)(&gyroLpf), &sensorData.gyro);

#ifdef CONFIG_TARGET_ESPLANE_V1
    accScaled.x = (accelRaw.x) * SENSORS_G_PER_LSB_CFG / accScale;
#else
    accScaled.x = -(accelRaw.x) * SENSORS_G_PER_LSB_CFG / accScale;   
#endif

    accScaled.y = (accelRaw.y) * SENSORS_G_PER_LSB_CFG / accScale;
    accScaled.z = (accelRaw.z) * SENSORS_G_PER_LSB_CFG / accScale;

    /* 传感器步骤 2.6：补偿加速度计未对齐 */
    sensorsAccAlignToGravity(&accScaled, &sensorData.acc);
    applyAxis3fLpf((lpf2pData *)(&accLpf), &sensorData.acc);
}
static void sensorsDeviceInit(void)
{
    isMagnetometerPresent = false;
    isBarometerPresent = false;

    // 等待传感器启动
    while (xTaskGetTickCount() < 2000){
        vTaskDelay(M2T(50));
    };

    i2cdevInit(I2C0_DEV);
    mpu6050Init(I2C0_DEV);

    if (mpu6050TestConnection() == true) {
        DEBUG_PRINTI("MPU6050 I2C connection [OK].\n");
    } else {
        DEBUG_PRINTE("MPU6050 I2C connection [FAIL].\n");
        DEBUG_PRINTE("Please power off and power on the device.\n");
        while (1) 
        {
            vTaskDelay(M2T(100));
        }
    }

    mpu6050Reset();
    vTaskDelay(M2T(50));
    // 激活 mpu6050
    mpu6050SetSleepEnabled(false);
    // 等待寄存器复位完成
    vTaskDelay(M2T(100));
    // 将 X 轴陀螺作为时钟源
    mpu6050SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
    // 等待时钟设置并稳定
    vTaskDelay(M2T(200));
    // 使能温度传感器
    mpu6050SetTempSensorEnabled(true);
    // 禁用中断
    mpu6050SetIntEnabled(false);
    // 将磁力计和气压计连接到主 I2C 总线
    mpu6050SetI2CBypassEnabled(true);
    // 设置陀螺满量程
    mpu6050SetFullScaleGyroRange(SENSORS_GYRO_FS_CFG);
    // 设置加速度计满量程
    mpu6050SetFullScaleAccelRange(SENSORS_ACCEL_FS_CFG);

    // 设置陀螺和加速度计的数字低通带宽
    // ESP-Drone v1.2 板振动更大，带宽应更低
#ifdef SENSORS_MPU6050_DLPF_256HZ
    // 256Hz 数字低通仅适用于振动较小的情况
    // 设置输出率 (15)：8000 / (1 + 7) = 1000Hz
    mpu6050SetRate(7);
    // 设置数字低通带宽
    mpu6050SetDLPFMode(MPU6050_DLPF_BW_256);
#elif defined(CONFIG_TARGET_ESP32_S2_DRONE_V1_2) || defined(CONFIG_TARGET_ESP32_S3_DRONE_WROOM_1)
    // DLPF 带宽过低可能导致不稳定并降低敏捷性，
    // 但有利于处理振动和螺旋桨不平衡
    // 设置输出率 (1)：1000 / (1 + 0) = 1000Hz
    mpu6050SetRate(0);
    mpu6050SetDLPFMode(MPU6050_DLPF_BW_42);
    // 初始化加速度计二阶滤波器
    for (uint8_t i = 0; i < 3; i++) {
        lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
        lpf2pInit(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
    }
#else
    mpu6050SetRate(0);
    mpu6050SetDLPFMode(MPU6050_DLPF_BW_98);
    // 初始化加速度计二阶滤波器
    for (uint8_t i = 0; i < 3; i++) {
        lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
        lpf2pInit(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
    }
#endif

#ifdef SENSORS_ENABLE_MAG_HM5883L
    hmc5883lInit(I2C0_DEV);

    if (hmc5883lTestConnection() == true) {
        isMagnetometerPresent = true;
        hmc5883lSetMode(HMC5883L_MODE_CONTINUOUS); // 16 位 100Hz
        DEBUG_PRINTI("hmc5883l I2C connection [OK].\n");
    } else {
        DEBUG_PRINTW("hmc5883l I2C connection [FAIL].\n");
    }

#endif
#ifdef SENSORS_ENABLE_PRESSURE_MS5611
    ms5611Init(I2C0_DEV);

    if (false) {
        isBarometerPresent = true;
        DEBUG_PRINTI("MS5611 I2C connection [OK].\n");
    } else {
        // TODO：无连接时是否应让传感器测试失败并退出
        DEBUG_PRINTW("MS5611 I2C connection [FAIL].\n");
    }

#endif

#ifdef SENSORS_ENABLE_RANGE_VL53L1X
    zRanger2Init();

    if (zRanger2Test() == true) {
        isVl53l1xPresent = true;
        DEBUG_PRINTI("VL53L1X I2C connection [OK].\n");
    } else {
        // TODO：无连接时是否应让传感器测试失败并退出
        DEBUG_PRINTW("VL53L1X I2C connection [FAIL].\n");
    }

#endif

#ifdef SENSORS_ENABLE_RANGE_VL53L0X
    zRangerInit();

    if (zRangerTest() == true) {
        isVl53l0xPresent = true;
        DEBUG_PRINTI("VL53L0X I2C connection [OK].\n");
    } else {
        // TODO：无连接时是否应让传感器测试失败并退出
        DEBUG_PRINTW("VL53L0X I2C connection [FAIL].\n");
    }

#endif

#ifdef SENSORS_ENABLE_FLOW_PMW3901
    flowdeck2Init();

    if (flowdeck2Test() == true) {
        isPmw3901Present = true;
        setCommandermode(POSHOLD_MODE);
        DEBUG_PRINTI("PMW3901 SPI connection [OK].\n");
    } else {
        // TODO：无连接时是否应让传感器测试失败并退出
        DEBUG_PRINTW("PMW3901 SPI connection [FAIL].\n");
    }
#endif

    DEBUG_PRINTI("sensors init done");
    /*
    * 从 NVS 获取校准角度
    */
    // cosPitch = cosf(configblockGetCalibPitch() * (float)M_PI / 180);
    // sinPitch = sinf(configblockGetCalibPitch() * (float)M_PI / 180);
    // cosRoll = cosf(configblockGetCalibRoll() * (float)M_PI / 180);
    // sinRoll = sinf(configblockGetCalibRoll() * (float)M_PI / 180);
    cosPitch = cosf(PITCH_CALIB * (float)M_PI / 180);
    sinPitch = sinf(PITCH_CALIB * (float)M_PI / 180);
    cosRoll = cosf(ROLL_CALIB * (float)M_PI / 180);
    sinRoll = sinf(ROLL_CALIB * (float)M_PI / 180);
    DEBUG_PRINTI("pitch_calib = %f,roll_calib = %f",PITCH_CALIB,ROLL_CALIB);
}

static void sensorsSetupSlaveRead(void)
{
    // 开始配置从设备
#ifdef SENSORS_MPU6050_DLPF_256HZ
    // 根据寄存器手册 4.4：“数据应以不低于采样率采样；
    // SMPLRT_DIV 仅用于 1kHz 内部采样。” 最慢更新率为 500Hz。
    mpu6050SetSlave4MasterDelay(15); // 读取从设备 500Hz = (8000Hz / (1 + 15))
#else
    mpu6050SetSlave4MasterDelay(9); // 读取从设备 100Hz = (500Hz / (1 + 4))
#endif

    mpu6050SetI2CBypassEnabled(false);
    mpu6050SetWaitForExternalSensorEnabled(true);     // 从设备数据对状态估计不太关键
    mpu6050SetInterruptMode(0);                       // 高电平有效
    mpu6050SetInterruptDrive(0);                      // 推挽输出
    mpu6050SetInterruptLatch(0);                      // 直到清除才解除锁存
    mpu6050SetInterruptLatchClear(1);                 // 任意寄存器读清除
    mpu6050SetSlaveReadWriteTransitionEnabled(false); // 从设备读取结束发送停止位
    mpu6050SetMasterClockSpeed(13);                   // 设置 I2C 速度为 400kHz

#ifdef SENSORS_ENABLE_MAG_HM5883L

    if (isMagnetometerPresent) {
        // 设置 mpu6050 主机端需要读取的寄存器
        mpu6050SetSlaveAddress(0, 0x80 | HMC5883L_ADDRESS);        // 磁力计设为从设备 0，启用读
        mpu6050SetSlaveRegister(0, HMC5883L_RA_MODE);       // 读取磁力计数据寄存器
        mpu6050SetSlaveDataLength(0, SENSORS_MAG_BUFF_LEN); // hmc5883l：mode,x,z,y,status；ak8963：读取 8 字节（ST1、x、y、z、ST2（溢出检查））
        mpu6050SetSlaveDelayEnabled(0, true);
        mpu6050SetSlaveEnabled(0, true);
        DEBUG_PRINTD("mpu6050SetSlaveAddress HMC5883L done \n");
    }

#endif

#ifdef SENSORS_ENABLE_PRESSURE_MS5611

    if (isBarometerPresent) {
        // 将 LPS25H 配置为从设备并启用读取
        // 通过两次读取可兼容 LPS25H FIFO 平均滤波，
        // 自动递增在读取 LPS25H_PRESS_OUT_H 后回绕到 LPS25H_PRESS_OUT_L。
        mpu6050SetSlaveAddress(1, 0x80 | MS5611_ADDR_CSB_LOW);
        mpu6050SetSlaveRegister(1, MS5611_D1);
        mpu6050SetSlaveDataLength(1, MS5611_D1D2_SIZE);
        mpu6050SetSlaveDelayEnabled(1, true);
        mpu6050SetSlaveEnabled(1, true);

        mpu6050SetSlaveAddress(2, 0x80 | MS5611_ADDR_CSB_LOW); // 温度
        mpu6050SetSlaveRegister(2, MS5611_D2);
        mpu6050SetSlaveDataLength(2, MS5611_D1D2_SIZE);
        mpu6050SetSlaveDelayEnabled(2, true);
        mpu6050SetSlaveEnabled(2, true);
        DEBUG_PRINTD("mpu6050SetSlaveAddress MS5611 done \n");
    }

#endif

    // 配置完成后启用传感器
    mpu6050SetI2CMasterModeEnabled(true);

    mpu6050SetIntDataReadyEnabled(true);

    DEBUG_PRINTD("sensorsSetupSlaveRead done \n");
}

static void sensorsTaskInit(void)
{
  accelerometerDataQueue = STATIC_MEM_QUEUE_CREATE(accelerometerDataQueue);
  gyroDataQueue = STATIC_MEM_QUEUE_CREATE(gyroDataQueue);
  magnetometerDataQueue = STATIC_MEM_QUEUE_CREATE(magnetometerDataQueue);
  barometerDataQueue = STATIC_MEM_QUEUE_CREATE(barometerDataQueue);

  STATIC_MEM_TASK_CREATE(sensorsTask, sensorsTask, SENSORS_TASK_NAME, NULL, SENSORS_TASK_PRI);
  DEBUG_PRINTD("xTaskCreate sensorsTask \n");
}

static void IRAM_ATTR sensors_inta_isr_handler(void *arg)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    imuIntTimestamp = usecTimestamp(); // 返回自 esp_timer 初始化以来的微秒数
    xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

static void sensorsInterruptInit(void)
{

    DEBUG_PRINTD("sensorsInterruptInit \n");
    gpio_config_t io_conf = {
        // 上升沿中断
#if ESP_IDF_VERSION_MAJOR > 4
        .intr_type = GPIO_INTR_POSEDGE,
#else
        .intr_type = GPIO_PIN_INTR_POSEDGE,
#endif
        // 引脚位掩码
        .pin_bit_mask = (1ULL << GPIO_INTA_MPU6050_IO),
        // 设置为输入模式
        .mode = GPIO_MODE_INPUT,
        // 禁用下拉
        .pull_down_en = 0,
        // 启用上拉
        .pull_up_en = 1,
    };
    sensorsDataReady = xSemaphoreCreateBinary();
    dataReady = xSemaphoreCreateBinary();
    gpio_config(&io_conf);
    // 安装 GPIO ISR 服务
    //portDISABLE_INTERRUPTS();
    gpio_set_intr_type(GPIO_INTA_MPU6050_IO, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // 为指定 GPIO 引脚挂接 ISR 处理函数
    gpio_isr_handler_add(GPIO_INTA_MPU6050_IO, sensors_inta_isr_handler, (void *)GPIO_INTA_MPU6050_IO);
    //portENABLE_INTERRUPTS();
    DEBUG_PRINTD("sensorsInterruptInit done \n");

    //   FSYNC “不可悬空，必须由 MCU 拉高或拉低”

}

void sensorsMpu6050Hmc5883lMs5611Init(void)
{
    if (isInit) {
        return;
    }

    sensorsBiasObjInit(&gyroBiasRunning);
    sensorsDeviceInit();
    sensorsInterruptInit();
    sensorsTaskInit();
    isInit = true;
}

bool sensorsMpu6050Hmc5883lMs5611Test(void)
{

    bool testStatus = true;

    if (!isInit) {
        DEBUG_PRINTE("Error while initializing sensor task\r\n");
        testStatus = false;
    }

    // 尝试 3 秒，让飞控稳定后再通过测试
    for (int i = 0; i < 300; i++) {
        if (mpu6050SelfTest() == true) {
            isMpu6050TestPassed = true;
            break;
        } else {
            vTaskDelay(M2T(10));
        }
    }

    testStatus &= isMpu6050TestPassed;

#ifdef SENSORS_ENABLE_MAG_HM5883L
    testStatus &= isMagnetometerPresent;

    if (testStatus) {
        isHmc5883lTestPassed = hmc5883lSelfTest();
        testStatus &= isHmc5883lTestPassed;
    }

#endif

#ifdef SENSORS_ENABLE_PRESSURE_MS5611
    testStatus &= isBarometerPresent;

    if (testStatus) {
        isMs5611TestPassed = ms5611SelfTest();

        testStatus &= isMs5611TestPassed;
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

    if (!accBiasFound) {
        accScaleSum += sqrtf(powf(ax * SENSORS_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_G_PER_LSB_CFG, 2) + powf(az * SENSORS_G_PER_LSB_CFG, 2));
        accScaleSumCount++;

        if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES) {
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

    if (!gyroBiasNoBuffFound) {
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
        if (gyroBiasSampleCount == SENSORS_BIAS_SAMPLES) {
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

    if (!gyroBiasRunning.isBiasValueFound) {
        sensorsFindBiasValue(&gyroBiasRunning);

        if (gyroBiasRunning.isBiasValueFound) {
            // TODO：
            soundSetEffect(SND_CALIB);
            ledseqRun(&seq_calibrated);
            DEBUG_PRINTI("isBiasValueFound!");
        }
    }

    gyroBiasOut->x = gyroBiasRunning.bias.x;
    gyroBiasOut->y = gyroBiasRunning.bias.y;
    gyroBiasOut->z = gyroBiasRunning.bias.z;

    return gyroBiasRunning.isBiasValueFound;
}
#endif

static void sensorsBiasObjInit(BiasObj *bias)
{
    bias->isBufferFilled = false;
    bias->bufHead = bias->buffer;
}

/**
 * 计算偏置缓冲区的方差与均值。
 */
static void sensorsCalculateVarianceAndMean(BiasObj *bias, Axis3f *varOut, Axis3f *meanOut)
{
    uint32_t i;
    int64_t sum[GYRO_NBR_OF_AXES] = {0};
    int64_t sumSq[GYRO_NBR_OF_AXES] = {0};

    for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++) {
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
static void __attribute__((used)) sensorsCalculateBiasMean(BiasObj *bias, Axis3i32 *meanOut)
{
    uint32_t i;
    int32_t sum[GYRO_NBR_OF_AXES] = {0};

    for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++) {
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
static void sensorsAddBiasValue(BiasObj *bias, int16_t x, int16_t y, int16_t z)
{
    bias->bufHead->x = x;
    bias->bufHead->y = y;
    bias->bufHead->z = z;
    bias->bufHead++;

    if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES]) {
        bias->bufHead = bias->buffer;
        bias->isBufferFilled = true;
    }
}

/**
 * 检查方差是否低于预设阈值。
 * 调用前应已将偏置值写入。
 * @param bias  偏置对象
 */
static bool sensorsFindBiasValue(BiasObj *bias)
{
    static int32_t varianceSampleTime;
    bool foundBias = false;

    if (bias->isBufferFilled) {
        sensorsCalculateVarianceAndMean(bias, &bias->variance, &bias->mean);

        if (bias->variance.x < GYRO_VARIANCE_THRESHOLD_X &&
                bias->variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
                bias->variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
                (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount())) {
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

bool sensorsMpu6050Hmc5883lMs5611ManufacturingTest(void)
{
    bool testStatus = false;
    Axis3i16 g;
    Axis3i16 a;
    Axis3f acc; // 加速度计轴向数据（mG）
    float pitch, roll;
    uint32_t startTick = xTaskGetTickCount();

    testStatus = mpu6050SelfTest();

    if (testStatus) {
        sensorsBiasObjInit(&gyroBiasRunning);

        while (xTaskGetTickCount() - startTick < SENSORS_VARIANCE_MAN_TEST_TIMEOUT) {
            mpu6050GetMotion6(&a.y, &a.x, &a.z, &g.y, &g.x, &g.z);

            if (processGyroBias(g.x, g.y, g.z, &gyroBias)) {
                gyroBiasFound = true;
                DEBUG_PRINTI("Gyro variance test [OK]\n");
                break;
            }
        }

        if (gyroBiasFound) {
            acc.x = (a.x) * SENSORS_G_PER_LSB_CFG;
            acc.y = (a.y) * SENSORS_G_PER_LSB_CFG;
            acc.z = (a.z) * SENSORS_G_PER_LSB_CFG;

            // 基于加速度计计算俯仰与横滚，要求板子水平
            pitch = tanf(-acc.x / (sqrtf(acc.y * acc.y + acc.z * acc.z))) * 180 / (float)M_PI;
            roll = tanf(acc.y / acc.z) * 180 / (float)M_PI;

            if ((fabsf(roll) < SENSORS_MAN_TEST_LEVEL_MAX) && (fabsf(pitch) < SENSORS_MAN_TEST_LEVEL_MAX)) {
                DEBUG_PRINTI("Acc level test [OK]\n");
                testStatus = true;
            } else {
                DEBUG_PRINTE("Acc level test Roll:%0.2f, Pitch:%0.2f [FAIL]\n", (double)roll, (double)pitch);
                testStatus = false;
            }
        } else {
            DEBUG_PRINTE("Gyro variance test [FAIL]\n");
            testStatus = false;
        }
    }

    return testStatus;
}

/**
 * 补偿未对齐的加速度计。使用 UI 采集并写入配置块的校准数据，
 * 旋转加速度计以对齐重力方向。
 */
static void sensorsAccAlignToGravity(Axis3f *in, Axis3f *out)
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

/** 在不同环境下设置不同的低通滤波参数
 *
 *
 */
void sensorsMpu6050Hmc5883lMs5611SetAccMode(accModes accMode)
{
    switch (accMode)
    {
    case ACC_MODE_PROPTEST:
        mpu6050SetRate(7);
        mpu6050SetDLPFMode(MPU6050_DLPF_BW_256);
        for (uint8_t i = 0; i < 3; i++)
        {
            lpf2pInit(&accLpf[i], 1000, 250);
        }
        break;
    case ACC_MODE_FLIGHT:
    default:
        mpu6050SetRate(0);
#if defined(CONFIG_TARGET_ESP32_S2_DRONE_V1_2) || defined(CONFIG_TARGET_ESP32_S3_DRONE_WROOM_1)
        mpu6050SetDLPFMode(MPU6050_DLPF_BW_42);
        for (uint8_t i = 0; i < 3; i++) {
        lpf2pInit(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
        }
#else
        mpu6050SetDLPFMode(MPU6050_DLPF_BW_98);
        for (uint8_t i = 0; i < 3; i++) {
        lpf2pInit(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
        }
#endif
        break;
    }
}

static void applyAxis3fLpf(lpf2pData *data, Axis3f *in)
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

// TODO：
PARAM_GROUP_START(imu_sensors)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, HMC5883L, &isMagnetometerPresent)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, MS5611, &isBarometerPresent) // TODO：将 MS5611 重命名为 LPS25H，客户端需同步更新
PARAM_GROUP_STOP(imu_sensors)

PARAM_GROUP_START(imu_tests)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, mpu6050, &isMpu6050TestPassed)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, HMC5883L, &isMagnetometerPresent)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, pmw3901, &isPmw3901Present)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, MS5611, &isBarometerPresent) // TODO：将 MS5611 重命名为 LPS25H，客户端需同步更新
PARAM_GROUP_STOP(imu_tests)
