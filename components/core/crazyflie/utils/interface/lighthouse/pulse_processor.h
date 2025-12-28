/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 - 2020 Bitcraze AB
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
 *
 * pulse_processor.h - pulse decoding for lighthouse V1 base stations
 *
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "ootx_decoder.h"
#include "lighthouse_calibration.h"
#include "lighthouse_geometry.h"

#define PULSE_PROCESSOR_N_SWEEPS 2
#define PULSE_PROCESSOR_N_BASE_STATIONS 2
#define PULSE_PROCESSOR_N_SENSORS 4
#define PULSE_PROCRSSOR_N_CONCURRENT_BLOCKS 2
#define PULSE_PROCESSOR_N_WORKSPACE (PULSE_PROCESSOR_N_SENSORS * PULSE_PROCRSSOR_N_CONCURRENT_BLOCKS)

#define PULSE_PROCESSOR_HISTORY_LENGTH 8
#define PULSE_PROCESSOR_TIMESTAMP_BITWIDTH 24
#define PULSE_PROCESSOR_TIMESTAMP_MAX ((1 << PULSE_PROCESSOR_TIMESTAMP_BITWIDTH) - 1)
#define PULSE_PROCESSOR_TIMESTAMP_BITMASK PULSE_PROCESSOR_TIMESTAMP_MAX

// 工具函数与宏

/**
 * @brief 计算两个时间戳的差值，并截断到时间戳位宽（PULSE_PROCESSOR_TIMESTAMP_BITWIDTH）
 *
 * @param x 时间戳
 * @param y 时间戳
 * @return x - y（截断后）
 */
inline static uint32_t TS_DIFF(const uint32_t x, const uint32_t y) {
  return (x - y) & PULSE_PROCESSOR_TIMESTAMP_BITMASK;
}

/**
 * @brief 判断 abs(a - b) 是否大于 limit，适用于时间戳位宽为 PULSE_PROCESSOR_TIMESTAMP_BITWIDTH 的情况
 *
 * @param a 时间戳
 * @param b 时间戳
 * @param limit a 与 b 的最小差值
 * @return true 若 abs(a - b) > limit
 */
inline static uint32_t TS_ABS_DIFF_LARGER_THAN(const uint32_t a, const uint32_t b, const uint32_t limit) {
    return TS_DIFF(a + limit, b) > (limit * 2);
}


/**
 * @brief 单个脉冲的数据，由单个传感器检测并由甲板上的 FPGA 解码。
 * 同时用于 Lighthouse V1 和 V2。
 *
 */
typedef struct {
  uint8_t sensor;
  uint32_t timestamp;

  // V1 基站数据 --------
  uint16_t width;

  // V2 基站数据 --------
  uint32_t beamData;
  uint32_t offset;
  // 通道在此为 0 索引（0-15），而基站配置为 1 索引（1-16）
  uint8_t channel; // channelFound 为 true 时有效
  uint8_t slowbit; // channelFound 为 true 时有效
  bool channelFound;
} pulseProcessorFrame_t;

typedef enum {
    lighthouseBsTypeUnknown = 0,
    lighthouseBsTypeV1 = 1,
    lighthouseBsTypeV2 = 2,
} lighthouseBaseStationType_t;

enum pulseClass_e {unknown, sync0, sync1, sweep};

typedef struct {
  uint32_t timestamp;
  int width;
} pulseProcessorPulse_t;

typedef enum {
  sweepIdFirst = 0,  // LH1 的 X 扫描
  sweepIdSecond = 1  // LH1 的 Y 扫描
} SweepId_t;

typedef enum {
  sweepStorageStateWaiting = 0,
  sweepStorageStateValid,
  sweepStorageStateError,
} SweepStorageState_t;

/**
 * @brief 保存 V1 基站解码所需数据
 *
 */
typedef struct {
  bool synchronized;    // 为 true 表示当前已同步
  int basestationsSynchronizedCount;

  // 同步状态
  pulseProcessorPulse_t pulseHistory[PULSE_PROCESSOR_N_SENSORS][PULSE_PROCESSOR_HISTORY_LENGTH];
  int pulseHistoryIdx[PULSE_PROCESSOR_N_SENSORS];


  // 同步脉冲时间戳估计
  uint32_t lastSync;        // 最近一次看到的同步
  uint64_t currentSyncSum;  // 所有相邻同步时间戳的累加和
  int nSyncPulses;          // 累计的同步脉冲数量

  // 同步脉冲时间戳
  uint32_t currentSync;   // 当前用于扫描相位测量的同步
  uint32_t currentSync0;  // 当前帧的 Sync0
  uint32_t currentSync0Width;  // 当前帧 Sync0 的宽度
  uint32_t currentSync1Width;  // 当前帧 Sync1 的宽度

  uint32_t currentSync0X;
  uint32_t currentSync0Y;
  uint32_t currentSync1X;
  uint32_t currentSync1Y;

  float frameWidth[2][2];

  // 当前帧的基站与轴
  int currentBaseStation;
  SweepId_t currentAxis;

  // 扫描时间戳
  struct {
    uint32_t timestamp;
    SweepStorageState_t state;
  } sweeps[PULSE_PROCESSOR_N_SENSORS];
  bool sweepDataStored;
} pulseProcessorV1_t;


/**
 * @brief 传感器采集到的原始脉冲数据。时间上接近且可能来自同一扫描的脉冲数据，
 * 可能包含多个基站的脉冲数据。
 *
 */
typedef struct {
    pulseProcessorFrame_t slots[PULSE_PROCESSOR_N_WORKSPACE];
    int slotsUsed;
    uint32_t latestTimestamp;
} pulseProcessorV2PulseWorkspace_t;

/**
 * @brief 保存一次穿过所有传感器的扫描的派生数据，由脉冲数据计算得到
 *
 */
typedef struct {
    uint32_t offset[PULSE_PROCESSOR_N_SENSORS];
    uint32_t timestamp0; // 转子偏移为 0（0 度）时的时间戳
    uint8_t channel;
} pulseProcessorV2SweepBlock_t;

/**
 * @brief 解码脉冲工作区时使用的块
 *
 */
typedef struct {
    pulseProcessorV2SweepBlock_t blocks[PULSE_PROCRSSOR_N_CONCURRENT_BLOCKS];
} pulseProcessorV2BlockWorkspace_t;

/**
 * @brief 保存 V2 基站解码所需数据
 *
 */
typedef struct {
  pulseProcessorV2PulseWorkspace_t pulseWorkspace;
  pulseProcessorV2BlockWorkspace_t blockWorkspace;

  // 每个基站的最新块，用于配对转子一次旋转中的两个块（扫描）
  pulseProcessorV2SweepBlock_t blocks[PULSE_PROCESSOR_N_BASE_STATIONS];

  // 最新处理的 slowbit 对应的转子零位时间戳
  uint32_t ootxTimestamps[PULSE_PROCESSOR_N_BASE_STATIONS];
} pulseProcessorV2_t;

typedef struct pulseProcessor_s {
  bool receivedBsSweep[PULSE_PROCESSOR_N_BASE_STATIONS];

  union {
    struct {
      pulseProcessorV1_t v1;
    };

    struct {
      pulseProcessorV2_t v2;
    };
  };

  ootxDecoderState_t ootxDecoder[PULSE_PROCESSOR_N_BASE_STATIONS];
  lighthouseCalibration_t bsCalibration[PULSE_PROCESSOR_N_BASE_STATIONS];
  baseStationGeometry_t bsGeometry[PULSE_PROCESSOR_N_BASE_STATIONS];
  baseStationGeometryCache_t bsGeoCache[PULSE_PROCESSOR_N_BASE_STATIONS];

  // 健康检查数据
  uint32_t healthFirstSensorTs;
  uint8_t healthSensorBitField;
  bool healthDetermined;
} pulseProcessor_t;

typedef struct {
  float angles[PULSE_PROCESSOR_N_SWEEPS];
  float correctedAngles[PULSE_PROCESSOR_N_SWEEPS];
  int validCount;
} pulseProcessorBaseStationMeasuremnt_t;

typedef struct {
  pulseProcessorBaseStationMeasuremnt_t baseStatonMeasurements[PULSE_PROCESSOR_N_BASE_STATIONS];
} pulseProcessorSensorMeasurement_t;

typedef struct {
  pulseProcessorSensorMeasurement_t sensorMeasurementsLh1[PULSE_PROCESSOR_N_SENSORS];
  pulseProcessorSensorMeasurement_t sensorMeasurementsLh2[PULSE_PROCESSOR_N_SENSORS];
  lighthouseBaseStationType_t measurementType;
} pulseProcessorResult_t;

/**
 * @brief 处理 Lighthouse 脉冲数据的接口
 *
 * @param state 状态
 * @param frameData 帧数据
 * @param baseStation 基站
 * @param axis 轴
 * @return true 已写入角度、基站和轴
 * @return false 无有效结果
 */
typedef bool (*pulseProcessorProcessPulse_t)(pulseProcessor_t *state, const pulseProcessorFrame_t* frameData, pulseProcessorResult_t* angles, int *baseStation, int *axis);

/**
 * @brief 对指定基站的所有传感器角度应用标定修正
 *
 * @param state 状态
 * @param angles 角度结果
 * @param baseStation 基站
 */
void pulseProcessorApplyCalibration(pulseProcessor_t *state, pulseProcessorResult_t* angles, int baseStation);

void pulseProcessorClearOutdated(pulseProcessor_t *appState, pulseProcessorResult_t* angles, int basestation);

/**
 * @brief 当数据处理并转换为测量值后，清空某个基站的结果结构体
 *
 * @param angles 需要清空的结果结构体
 * @param baseStation 基站
 */
void pulseProcessorProcessed(pulseProcessorResult_t* angles, int baseStation);

/**
 * @brief 当传感器数据无效时，清空某个基站的结果结构体
 *
 * @param angles 需要清空的结果结构体
 * @param baseStation 基站
 */
void pulseProcessorClear(pulseProcessorResult_t* angles, int baseStation);

/**
 * @brief 当传感器数据无效时清空结果结构体
 *
 * @param angles 结果结构体
 */
void pulseProcessorAllClear(pulseProcessorResult_t* angles);

/**
 * 获取基站角度接收质量。
 * 0 表示没有角度，255 表示所有基站所有轴的角度均已接收。
 */
uint8_t pulseProcessorAnglesQuality();
