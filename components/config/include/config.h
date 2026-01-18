/**
 *
 * ESP-Drone Firmware
 * 
 * Copyright 2019-2020  Espressif Systems (Shanghai) 
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * config.h - Main configuration file
 *
 * This file define the default configuration of the copter
 * It contains two types of parameters:
 * - The global parameters are globally defined and independent of any
 *   compilation profile. An example of such define could be some pinout.
 * - The profiled defines, they are parameter that can be specific to each
 *   dev build. The vanilla build is intended to be a "customer" build without
 *   fancy spinning debugging stuff. The developers build are anything the
 *   developer could need to debug and run his code/crazy stuff.
 *
 * The golden rule for the profile is NEVER BREAK ANOTHER PROFILE. When adding a
 * new parameter, one shall take care to modified everything necessary to
 * preserve the behavior of the other profiles.
 *
 * For the flag. T_ means task. H_ means HAL module. U_ would means utils.
 */

#ifndef CONFIG_H_
#define CONFIG_H_
//#include "nrf24l01.h"

//#include "trace.h"
#include "usec_time.h"
#include "sdkconfig.h"

// ATK-MS901M integration (single place to tune defaults)
#ifndef USE_ATK_MS901M
#define USE_ATK_MS901M 1  // 0: discrete sensors, 1: ATK-MS901M module
#endif

#ifndef ATK_MS901M_UART_PORT_NUM
#define ATK_MS901M_UART_PORT_NUM 0  // UART port for module
#endif
#ifndef ATK_MS901M_UART_TX_PIN
#define ATK_MS901M_UART_TX_PIN 43  // 榛樿鍗犱綅锛屽悗闈㈡寜鏉垮瀷瑕嗙洊
#endif
#ifndef ATK_MS901M_UART_RX_PIN
#define ATK_MS901M_UART_RX_PIN 44  // 榛樿鍗犱綅锛屽悗闈㈡寜鏉垮瀷瑕嗙洊
#endif
#ifndef ATK_MS901M_UART_BAUDRATE
#define ATK_MS901M_UART_BAUDRATE 115200
#endif
#ifndef ATK_MS901M_UART_RX_BUF_SIZE
#define ATK_MS901M_UART_RX_BUF_SIZE 1024
#endif
#ifndef ATK_MS901M_BOOT_DELAY_MS
#define ATK_MS901M_BOOT_DELAY_MS 100  // delay before first register read
#endif
#ifndef ATK_MS901M_SKIP_SYSTEM_WAIT
#define ATK_MS901M_SKIP_SYSTEM_WAIT 1  // run sensor UART task before systemStart
#endif
#ifndef ATK_MS901M_ECHO_PC_TX
#define ATK_MS901M_ECHO_PC_TX 0  // echo PC->ATK TX bytes on USB-CDC console
#endif
#ifndef ATK_MS901M_DEBUG_RX
#define ATK_MS901M_DEBUG_RX 0  // log ATK->ESP32 RX bytes
#endif
#ifndef ATK_MS901M_DEBUG_RX_MAX_BYTES
#define ATK_MS901M_DEBUG_RX_MAX_BYTES 16
#endif
#ifndef ATK_MS901M_DEBUG_RX_INTERVAL_MS
#define ATK_MS901M_DEBUG_RX_INTERVAL_MS 100  // 0 to log every RX chunk
#endif
#ifndef ATK_MS901M_CAL_DEBUG
#define ATK_MS901M_CAL_DEBUG 1  // print calibration progress for ATK-MS901M
#endif
#ifndef ATK_MS901M_CAL_DEBUG_INTERVAL_MS
#define ATK_MS901M_CAL_DEBUG_INTERVAL_MS 1000
#endif
#ifndef ATK_MS901M_GYRO_BIAS_LIGHT_WEIGHT
#define ATK_MS901M_GYRO_BIAS_LIGHT_WEIGHT 1  // use fixed-sample gyro bias (faster, less strict)
#endif
#ifndef ATK_MS901M_STREAM_1HZ
#define ATK_MS901M_STREAM_1HZ 0  // print sensor data periodically on USB-CDC
#endif
#ifndef ATK_MS901M_STREAM_INTERVAL_MS
#define ATK_MS901M_STREAM_INTERVAL_MS 1000
#endif

#ifndef ATK_MS901M_APPLY_CONFIG
#define ATK_MS901M_APPLY_CONFIG 1  // write returnset/returnrate at boot
#endif
#ifndef ATK_MS901M_RETURNSET
#define ATK_MS901M_RETURNSET 0x1C  // default: gyro+acc+mag+baro
#endif
#ifndef ATK_MS901M_RETURNRATE
#define ATK_MS901M_RETURNRATE 0x01  // 200 Hz
#endif
#ifndef ATK_MS901M_SAMPLE_RATE_HZ
#define ATK_MS901M_SAMPLE_RATE_HZ 200  // used for LPF setup
#endif

#ifndef ATK_MS901M_USE_MODULE_ORIENTATION
#define ATK_MS901M_USE_MODULE_ORIENTATION 0  // 0: use existing fusion, 1: use module orientation
#endif
#ifndef ATK_MS901M_ORIENTATION_SOURCE_ATTITUDE
#define ATK_MS901M_ORIENTATION_SOURCE_ATTITUDE 0
#endif
#ifndef ATK_MS901M_ORIENTATION_SOURCE_QUAT
#define ATK_MS901M_ORIENTATION_SOURCE_QUAT 1
#endif
#ifndef ATK_MS901M_ORIENTATION_SOURCE
#define ATK_MS901M_ORIENTATION_SOURCE ATK_MS901M_ORIENTATION_SOURCE_QUAT
#endif

#ifndef ATK_MS901M_AXIS_SWAP_XY
#define ATK_MS901M_AXIS_SWAP_XY 1  // keep same axes as current MPU6050 path
#endif
#ifndef ATK_MS901M_AXIS_INVERT_X
#define ATK_MS901M_AXIS_INVERT_X 1
#endif

#ifdef CONFIG_TARGET_ESPLANE_V1
#undef ATK_MS901M_AXIS_SWAP_XY
#undef ATK_MS901M_AXIS_INVERT_X
#define ATK_MS901M_AXIS_SWAP_XY 0
#define ATK_MS901M_AXIS_INVERT_X 0
#endif

// Deck I2C (I2C1). Disable on boards that repurpose IO40/41 for camera.
#ifndef USE_DECK_I2C
#define USE_DECK_I2C 1
#endif

// Deck SPI (Flow/other decks). Disable when ATK-MS901M occupies SPI pins.
#ifndef USE_DECK_SPI
#define USE_DECK_SPI 1
#endif

// Buzzer enable switch (override sdkconfig if needed)
#ifndef ENABLE_BUZZER
#define ENABLE_BUZZER 1
#endif

// USB-CDC ATK-MS901M console commands
#ifndef ENABLE_ATK_USB_CONSOLE
#define ENABLE_ATK_USB_CONSOLE 1
#endif

#if defined(CONFIG_TARGET_ESP32_S2_DRONE_V1_2) || defined(CONFIG_TARGET_ESP32_S3_DRONE_WROOM_1)
#undef ATK_MS901M_UART_PORT_NUM
#define ATK_MS901M_UART_PORT_NUM 1

#undef USE_DECK_I2C
#define USE_DECK_I2C 0
#undef ATK_MS901M_UART_TX_PIN
#undef ATK_MS901M_UART_RX_PIN
#define ATK_MS901M_UART_TX_PIN 43  // IO43 -> ATK-MS901M RX
#define ATK_MS901M_UART_RX_PIN 44  // IO44 <- ATK-MS901M TX
#undef ENABLE_BUZZER
#define ENABLE_BUZZER 0
#endif

#if USE_ATK_MS901M
#undef USE_DECK_SPI
#define USE_DECK_SPI 0
#endif

#if !ENABLE_BUZZER
#ifdef CONFIG_BUZZER_ON
#undef CONFIG_BUZZER_ON
#endif
#endif

// OV2640 camera streaming
#ifndef ENABLE_OV2640_STREAM
#define ENABLE_OV2640_STREAM 0
#endif

#ifndef OV2640_HTTP_PORT
#define OV2640_HTTP_PORT 80
#endif

#ifndef OV2640_PIN_SCL
#define OV2640_PIN_SCL 10
#endif
#ifndef OV2640_PIN_SDA
#define OV2640_PIN_SDA 11
#endif
#ifndef OV2640_PIN_VSYNC
#define OV2640_PIN_VSYNC 13
#endif
#ifndef OV2640_PIN_HREF
#define OV2640_PIN_HREF 14
#endif
#ifndef OV2640_PIN_Y9
#define OV2640_PIN_Y9 15
#endif
#ifndef OV2640_PIN_XCLK
#define OV2640_PIN_XCLK 16
#endif
#ifndef OV2640_PIN_Y8
#define OV2640_PIN_Y8 17
#endif
#ifndef OV2640_PIN_RESET
#define OV2640_PIN_RESET 18
#endif
#ifndef OV2640_PIN_Y6
#define OV2640_PIN_Y6 21
#endif
#ifndef OV2640_PIN_Y2
#define OV2640_PIN_Y2 39
#endif
#ifndef OV2640_PIN_PCLK
#define OV2640_PIN_PCLK 40
#endif
#ifndef OV2640_PIN_Y7
#define OV2640_PIN_Y7 41
#endif
#ifndef OV2640_PIN_Y5
#define OV2640_PIN_Y5 42
#endif
#ifndef OV2640_PIN_Y4
#define OV2640_PIN_Y4 45
#endif
#ifndef OV2640_PIN_Y3
#define OV2640_PIN_Y3 46
#endif
#ifndef OV2640_PIN_PWDN
#define OV2640_PIN_PWDN -1
#endif
#ifndef OV2640_XCLK_FREQ_HZ
#define OV2640_XCLK_FREQ_HZ 20000000
#endif
#ifndef OV2640_JPEG_QUALITY
#define OV2640_JPEG_QUALITY 12
#endif

#define PROTOCOL_VERSION 4
#define QUAD_FORMATION_X

#ifdef CONFIG_TARGET_ESPLANE_V2_S2
#ifndef CONFIG_IDF_TARGET_ESP32S2
#error "ESPLANE_V2 hardware with ESP32S2 onboard"
#endif
#elif defined(CONFIG_TARGET_ESPLANE_V1)
#ifndef CONFIG_IDF_TARGET_ESP32
#error "ESPLANE_V1 hardware with ESP32 onboard"
#endif
#elif defined(CONFIG_TARGET_ESP32_S2_DRONE_V1_2) || defined(CONFIG_TARGET_ESP32_S3_DRONE_WROOM_1)
#ifdef CONFIG_IDF_TARGET_ESP32
#error "ESP32-S2/S3 drone hardware with ESP32S2/S3 onboard"
#endif
#endif

#ifdef STM32F4XX 
  #define CONFIG_BLOCK_ADDRESS    (2048 * (64-1))
  #define MCU_ID_ADDRESS          0x1FFF7A10
  #define MCU_FLASH_SIZE_ADDRESS  0x1FFF7A22
  #ifndef FREERTOS_HEAP_SIZE
    #define FREERTOS_HEAP_SIZE      20000
  #endif
  #define FREERTOS_MIN_STACK_SIZE 150       // M4-FPU register setup is bigger so stack needs to be bigger
  #define FREERTOS_MCU_CLOCK_HZ   168000000

  #define configGENERATE_RUN_TIME_STATS 1
  #define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() initUsecTimer()
  #define portGET_RUN_TIME_COUNTER_VALUE() usecTimestamp()
#endif


//#define DEBUG_UDP
#ifndef WIFI_UDP_DIAG_PING
#define WIFI_UDP_DIAG_PING 1  // respond to UDP diagnostic ping (CFPING/CFPONG)
#endif
#ifndef WIFI_UDP_DIAG_LOG_INTERVAL_MS
#define WIFI_UDP_DIAG_LOG_INTERVAL_MS 1000  // 0 to log every diag ping
#endif
#ifndef WIFI_UDP_LASTPKT_LOG_INTERVAL_MS
#define WIFI_UDP_LASTPKT_LOG_INTERVAL_MS 1000  // 0 to log every valid UDP packet
#endif
#ifndef CRTP_SETPOINT_LOG_INTERVAL_MS
#define CRTP_SETPOINT_LOG_INTERVAL_MS 500  // 0 to log every setpoint packet
#endif
#ifndef STABILIZER_STATUS_LOG_INTERVAL_MS
#define STABILIZER_STATUS_LOG_INTERVAL_MS 1000  // 0 to disable stabilizer status logs
#endif
#ifndef FORCE_SYSTEM_START
#define FORCE_SYSTEM_START 1  // force systemStart even if selftest fails
#endif
//#define DEBUG_EP2

// Task priorities. Higher number higher priority
// system state tasks
#define SYSTEM_TASK_PRI         1
#define PM_TASK_PRI             1
#define LEDSEQCMD_TASK_PRI      1
// communication TX tasks
#define UDP_TX_TASK_PRI         2
#define CRTP_TX_TASK_PRI        2
// communication RX tasks
#define UDP_RX_TASK_PRI         2
#define EXTRX_TASK_PRI          2
#define UART2_TASK_PRI          2
#define SYSLINK_TASK_PRI        2
#define USBLINK_TASK_PRI        2
#define WIFILINK_TASK_PRI       2
#define ATK_CONSOLE_TASK_PRI    2
#define CRTP_RX_TASK_PRI        2
#define CMD_HIGH_LEVEL_TASK_PRI 3
#define INFO_TASK_PRI           2
#define LOG_TASK_PRI            2
#define MEM_TASK_PRI            2
#define PARAM_TASK_PRI          2
// sensors and stabilize related tasks
#define PROXIMITY_TASK_PRI      5
#define FLOW_TASK_PRI           5
#define ZRANGER2_TASK_PRI       5
#define ZRANGER_TASK_PRI        5
#define SENSORS_TASK_PRI        6
#define STABILIZER_TASK_PRI     7
#define KALMAN_TASK_PRI         4

// the kalman filter consumes a lot of CPU
// for single core systems, we need to lower the priority
#if CONFIG_FREERTOS_UNICORE
  #undef KALMAN_TASK_PRI
  #define KALMAN_TASK_PRI         1
#endif

// Task names
#define CMD_HIGH_LEVEL_TASK_NAME "CMDHL"
#define CRTP_RX_TASK_NAME       "CRTP-RX"
#define CRTP_TX_TASK_NAME       "CRTP-TX"
#define EXTRX_TASK_NAME         "EXTRX"
#define FLOW_TASK_NAME          "FLOW"
#define KALMAN_TASK_NAME        "KALMAN"
#define LEDSEQCMD_TASK_NAME     "LEDSEQCMD"
#define LOG_TASK_NAME           "LOG"
#define MEM_TASK_NAME           "MEM"
#define PARAM_TASK_NAME         "PARAM"
#define PM_TASK_NAME            "PWRMGNT"
#define PROXIMITY_TASK_NAME     "PROXIMITY"
#define SENSORS_TASK_NAME       "SENSORS"
#define STABILIZER_TASK_NAME    "STABILIZER"
#define SYSLINK_TASK_NAME       "SYSLINK"
#define SYSTEM_TASK_NAME        "SYSTEM"
#define UART2_TASK_NAME         "UART2"
#define UDP_RX_TASK_NAME        "UDP_RX"
#define UDP_TX_TASK_NAME        "UDP_TX"
#define USBLINK_TASK_NAME       "USBLINK"
#define WIFILINK_TASK_NAME      "WIFILINK"
#define ATK_CONSOLE_TASK_NAME   "ATKCDC"
#define ZRANGER2_TASK_NAME      "ZRANGER2"
#define ZRANGER_TASK_NAME       "ZRANGER"

//Task stack sizes
#define configBASE_STACK_SIZE CONFIG_BASE_STACK_SIZE
#define CMD_HIGH_LEVEL_TASK_STACKSIZE (2 * configBASE_STACK_SIZE)
#define CRTP_RX_TASK_STACKSIZE        (3 * configBASE_STACK_SIZE)
#define CRTP_TX_TASK_STACKSIZE        (3 * configBASE_STACK_SIZE)
#define EXTRX_TASK_STACKSIZE          (1 * configBASE_STACK_SIZE)
#define FLOW_TASK_STACKSIZE           (3 * configBASE_STACK_SIZE)
#define KALMAN_TASK_STACKSIZE         (3 * configBASE_STACK_SIZE)
#define LEDSEQCMD_TASK_STACKSIZE      (2 * configBASE_STACK_SIZE)
#define LOG_TASK_STACKSIZE            (3 * configBASE_STACK_SIZE)
#define MEM_TASK_STACKSIZE            (2 * configBASE_STACK_SIZE)
#define PARAM_TASK_STACKSIZE          (2 * configBASE_STACK_SIZE)
#define PM_TASK_STACKSIZE             (4 * configBASE_STACK_SIZE)
#define SENSORS_TASK_STACKSIZE        (5 * configBASE_STACK_SIZE)
#define STABILIZER_TASK_STACKSIZE     (5 * configBASE_STACK_SIZE)
#define SYSLINK_TASK_STACKSIZE        (1 * configBASE_STACK_SIZE)
#define SYSTEM_TASK_STACKSIZE         (6 * configBASE_STACK_SIZE)
#define UART2_TASK_STACKSIZE          (1 * configBASE_STACK_SIZE)
#define UDP_RX_TASK_STACKSIZE         (4 * configBASE_STACK_SIZE)
#define UDP_TX_TASK_STACKSIZE         (4 * configBASE_STACK_SIZE)
#define USBLINK_TASK_STACKSIZE        (1 * configBASE_STACK_SIZE)
#define WIFILINK_TASK_STACKSIZE       (4 * configBASE_STACK_SIZE)
#define ATK_CONSOLE_TASK_STACKSIZE    (4 * configBASE_STACK_SIZE)
#define ZRANGER2_TASK_STACKSIZE       (4 * configBASE_STACK_SIZE)
#define ZRANGER_TASK_STACKSIZE        (2 * configBASE_STACK_SIZE)

//The radio channel. From 0 to 125
#define RADIO_RATE_2M 2
#define RADIO_CHANNEL 80
#define RADIO_DATARATE RADIO_RATE_2M
#define RADIO_ADDRESS 0xE7E7E7E7E7ULL

/**
 * \def PROPELLER_BALANCE_TEST_THRESHOLD
 * This is the threshold for a propeller/motor to pass. It calculates the variance of the accelerometer X+Y
 * when the propeller is spinning.
 */
#define PROPELLER_BALANCE_TEST_THRESHOLD  2.5f

/**
 * \def ACTIVATE_AUTO_SHUTDOWN
 * Will automatically shot of system if no radio activity
 */
//#define ACTIVATE_AUTO_SHUTDOWN

/**
 * \def ACTIVATE_STARTUP_SOUND
 * Playes a startup melody using the motors and PWM modulation
 */
//#define ACTIVATE_STARTUP_SOUND

// Define to force initialization of expansion board drivers. For test-rig and programming.
//#define FORCE_EXP_DETECT

/**
 * \def PRINT_OS_DEBUG_INFO
 * Print with an interval information about freertos mem/stack usage to console.
 */
//#define PRINT_OS_DEBUG_INFO


//Debug defines
//#define BRUSHLESS_MOTORCONTROLLER
//#define ADC_OUTPUT_RAW_DATA
//#define UART_OUTPUT_TRACE_DATA
//#define UART_OUTPUT_RAW_DATA_ONLY
//#define IMU_OUTPUT_RAW_DATA_ON_UART
//#define T_LAUCH_MOTORS
//#define T_LAUCH_MOTOR_TEST
//#define MOTOR_RAMPUP_TEST
/**
 * \def ADC_OUTPUT_RAW_DATA
 * When defined the gyro data will be written to the UART channel.
 * The UART must be configured to run really fast, e.g. in 2Mb/s.
 */
//#define ADC_OUTPUT_RAW_DATA

#if defined(UART_OUTPUT_TRACE_DATA) && defined(ADC_OUTPUT_RAW_DATA)
#  error "Can't define UART_OUTPUT_TRACE_DATA and ADC_OUTPUT_RAW_DATA at the same time"
#endif

#if defined(UART_OUTPUT_TRACE_DATA) || defined(ADC_OUTPUT_RAW_DATA) || defined(IMU_OUTPUT_RAW_DATA_ON_UART)
#define UART_OUTPUT_RAW_DATA_ONLY
#endif

#if defined(UART_OUTPUT_TRACE_DATA) && defined(T_LAUNCH_ACC)
#  error "UART_OUTPUT_TRACE_DATA and T_LAUNCH_ACC doesn't work at the same time yet due to dma sharing..."
#endif

#endif /* CONFIG_H_ */
