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
 * pulse_processor_v1.c - pulse decoding for lighthouse V1 base stations
 *
 */

#include "pulse_processor_v1.h"

#include <string.h>
#include <math.h>
#include "test_support.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 解码常量
// 时间以 24 MHz 时钟表示
#define FRAME_LENGTH 200000    // 8.333ms
#define SWEEP_MAX_WIDTH 512    // 20us

#define SYNC_DIVIDER 250
#define SYNC_BASE_WIDTH (1250 + (SYNC_DIVIDER / 2))
#define SYNC_MIN_WIDTH (1250 - SYNC_DIVIDER)
#define SYNC_MAX_WIDTH (1250 + SYNC_DIVIDER * 8)

#define SYNC_SEPARATION 10000
#define SYNC_SEPARATION_MAX_DISPERSION 2500
#define SYNC_MIN_SEPARATION (SYNC_SEPARATION - SYNC_SEPARATION_MAX_DISPERSION)
#define SYNC_MAX_SEPARATION (SYNC_SEPARATION + SYNC_SEPARATION_MAX_DISPERSION)

#define SENSOR_MAX_DISPERTION 10
#define MAX_FRAME_LENGTH_NOISE 400

#define FRAME_WIDTH_MIN 395000
#define FRAME_WIDTH_MAX 405000


TESTABLE_STATIC int findSyncTime(const pulseProcessorPulse_t pulseHistory[], uint32_t *sync0Time);
TESTABLE_STATIC bool getSystemSyncTime(const uint32_t syncTimes[], size_t nSyncTimes, uint32_t *syncTime);

static void resetPulseHistory(pulseProcessor_t *state) {
  memset(state->pulseHistoryIdx, 0, sizeof(state->pulseHistoryIdx));
}

static void synchronize(pulseProcessor_t *state, int sensor, uint32_t timestamp, uint32_t width)
{
  state->pulseHistory[sensor][state->pulseHistoryIdx[sensor]].timestamp = timestamp;
  state->pulseHistory[sensor][state->pulseHistoryIdx[sensor]].width = width;

  state->pulseHistoryIdx[sensor] += 1;

  // 一旦某个历史缓冲区满了，就运行同步算法！
  if (state->pulseHistoryIdx[sensor] >= PULSE_PROCESSOR_HISTORY_LENGTH) {
    static uint32_t sync0Times[PULSE_PROCESSOR_N_SENSORS];
    size_t syncTimeCount = 0;

    for (int i=0; i<PULSE_PROCESSOR_N_SENSORS; i++) {
      const int bsSyncsFound = findSyncTime(state->pulseHistory[i], &sync0Times[syncTimeCount]);
      if (bsSyncsFound >= state->basestationsSynchronizedCount) {
        state->basestationsSynchronizedCount = bsSyncsFound;
        syncTimeCount += 1;
      }

      uint32_t validSync0Time = 0;
      if (getSystemSyncTime(sync0Times, syncTimeCount, &validSync0Time)) {
        state->synchronized = true;
        state->currentSync0 = validSync0Time;
        state->lastSync = validSync0Time;
        break;
      }
    }

    resetPulseHistory(state);
  }
}

static bool isSweep(pulseProcessor_t *state, unsigned int timestamp, int width)
{
  uint32_t delta = TS_DIFF(timestamp, state->lastSync);
  return ((delta > SYNC_MAX_SEPARATION) && (delta < (FRAME_LENGTH - (2*SYNC_MAX_SEPARATION)))) || (width < SWEEP_MAX_WIDTH);
}

TESTABLE_STATIC bool isSync(pulseProcessor_t *state, unsigned int timestamp)
{
  uint32_t delta = TS_DIFF(timestamp, state->currentSync0);
  int deltaModulo = delta % FRAME_LENGTH;

  // 期望模值接近 0，检测并处理绕过 FRAME_LENGTH 的情况
  if (deltaModulo > (FRAME_LENGTH/2)) {
    deltaModulo -= FRAME_LENGTH;
  }

  if ((deltaModulo > -1*MAX_FRAME_LENGTH_NOISE) && (deltaModulo < SYNC_MAX_SEPARATION)) {
    return true;
  }
  return false;
}

/**
 * @brief 获取基站 ID
 *
 * 该函数通过脉冲相对于最新 Sync0 的位置对 FRAME_LENGTH 取模来判断。
 * 近似 0 表示 Sync0 脉冲，近似 19200 表示 Sync1。
 *
 * @param state 脉冲处理器状态
 * @param timestamp 需要处理的同步时间戳
 * @return Sync0 返回 0，Sync1 返回 1
 */
TESTABLE_STATIC int getBaseStationId(pulseProcessor_t *state, unsigned int timestamp) {
  int baseStation = 0;

  uint32_t delta = TS_DIFF(timestamp, state->currentSync0);
  int deltaModulo = delta % FRAME_LENGTH;

  // 期望模值接近 0，检测并处理绕过 FRAME_LENGTH 的情况
  if (deltaModulo > (FRAME_LENGTH/2)) {
    deltaModulo -= FRAME_LENGTH;
  }

  if (deltaModulo > (SYNC_SEPARATION/2)) {
    baseStation = 1;
  }

  return baseStation;
}

static SweepDirection getAxis(int width) {
  SweepDirection result = sweepDirection_x;

  if ((((width-SYNC_BASE_WIDTH)/SYNC_DIVIDER)&0x01) != 0) {
    result = sweepDirection_y;
  }

  return result;
}

TESTABLE_STATIC bool isSweepActiveThisFrame(int width) {
  return (((width-SYNC_BASE_WIDTH)/SYNC_DIVIDER)&0x04) == 0;
}

static int getOotxDataBit(int width)
{
  return (((width-SYNC_BASE_WIDTH)/SYNC_DIVIDER)&0x02) >> 1;
}

static void storeSweepData(pulseProcessor_t *state, int sensor, unsigned int timestamp) {
  if (state->sweeps[sensor].state == sweepStorageStateWaiting) {
    state->sweeps[sensor].timestamp = timestamp;
    state->sweeps[sensor].state = sweepStorageStateValid;
  } else {
    state->sweeps[sensor].state = sweepStorageStateError;
  }

  state->sweepDataStored = true;
}

static void resetSweepData(pulseProcessor_t *state) {
  for (size_t sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
    state->sweeps[sensor].state = sweepStorageStateWaiting;
  }
  state->sweepDataStored = false;
}

static void resetSynchronization(pulseProcessor_t *state)
{
  resetSweepData(state);
  state->synchronized = false;
}

static bool processPreviousFrame(pulseProcessor_t *state, pulseProcessorResult_t* result, int *baseStation, int *axis) {
  bool anglesMeasured = false;

  if (state->sweepDataStored) {
    for (size_t sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
      if (state->sweeps[sensor].state == sweepStorageStateValid) {
        int delta = TS_DIFF(state->sweeps[sensor].timestamp, state->currentSync);
        if (delta < FRAME_LENGTH) {
          float frameWidth = state->frameWidth[state->currentBaseStation][state->currentAxis];

          if ((frameWidth < FRAME_WIDTH_MIN) || (frameWidth > FRAME_WIDTH_MAX)) {
            return false;
          }

          float center = frameWidth/4.0f;
          float angle = (delta - center)*2*(float)M_PI/frameWidth;

          *baseStation = state->currentBaseStation;
          *axis = state->currentAxis;

          pulseProcessorBaseStationMeasuremnt_t* bsMeasurement = &result->sensorMeasurements[sensor].baseStatonMeasurements[state->currentBaseStation];
          bsMeasurement->angles[state->currentAxis] = angle;
          bsMeasurement->validCount++;

          anglesMeasured = true;
        }
      }
    }

    resetSweepData(state);
  }

  return anglesMeasured;
}

static void storeSyncData(pulseProcessor_t *state, int baseStation, unsigned int timestamp, unsigned int width) {
  if (0 == baseStation) {
    state->currentSync0 = timestamp;
    state->currentSync0Width = width;
    if (getAxis(width) == sweepDirection_x) {
      uint32_t prevSync0X = state->currentSync0X;
      state->currentSync0X = timestamp;
      state->frameWidth[0][0] = TS_DIFF(state->currentSync0X, prevSync0X);
    } else {
      uint32_t prevSync0Y = state->currentSync0Y;
      state->currentSync0Y = timestamp;
      state->frameWidth[0][1] = TS_DIFF(state->currentSync0Y, prevSync0Y);
    }
  } else {
    state->currentSync1Width = width;
    if (getAxis(width) == sweepDirection_x) {
      uint32_t prevSync1X = state->currentSync1X;
      state->currentSync1X = timestamp;
      state->frameWidth[1][0] = TS_DIFF(state->currentSync1X, prevSync1X);
    } else {
      uint32_t prevSync1Y = state->currentSync1Y;
      state->currentSync1Y = timestamp;
      state->frameWidth[1][1] = TS_DIFF(state->currentSync1Y, prevSync1Y);
    }

    state->currentSync0 = TS_DIFF(timestamp, SYNC_SEPARATION);
  }

  state->lastSync = timestamp;

  if (isSweepActiveThisFrame(width)) {
    state->currentBaseStation = baseStation;
    state->currentAxis = getAxis(width);
    state->currentSync = timestamp;
  }
}

TESTABLE_STATIC bool isNewSync(uint32_t timestamp, uint32_t lastSync) {
  const uint32_t min = (1 << PULSE_PROCESSOR_TIMESTAMP_BITWIDTH) - SENSOR_MAX_DISPERTION;
  const uint32_t max = SENSOR_MAX_DISPERTION;

  uint32_t diff = TS_DIFF(timestamp, lastSync);
  return (diff > max) && (diff < min);
}

#ifndef UNIT_TEST_MODE
#include "debug_cf.h"
#else
#include <stdio.h>
#define DEBUG_PRINT printf
#endif

static void printBSInfo(struct ootxDataFrame_s *frame)
{
  DEBUG_PRINT("Got calibration from %08X\n", (unsigned int)frame->id);
  DEBUG_PRINT("  phase0: %f\n", (double)frame->phase0);
  DEBUG_PRINT("  phase1: %f\n", (double)frame->phase1);
}

static void decodeAndApplyBaseStationCalibrationData(pulseProcessor_t *state) {
  if (!state->bsCalibration[0].valid &&
      ootxDecoderProcessBit(&state->ootxDecoder0, getOotxDataBit(state->currentSync0Width))) {
    printBSInfo(&state->ootxDecoder0.frame);
    lighthouseCalibrationInitFromFrame(&state->bsCalibration[0], &state->ootxDecoder0.frame);
  }
  if (!state->bsCalibration[1].valid &&
      ootxDecoderProcessBit(&state->ootxDecoder1, getOotxDataBit(state->currentSync1Width))) {
    printBSInfo(&state->ootxDecoder1.frame);
    lighthouseCalibrationInitFromFrame(&state->bsCalibration[1], &state->ootxDecoder1.frame);
  }
}

static bool processSync(pulseProcessor_t *state, unsigned int timestamp, unsigned int width, pulseProcessorResult_t* angles, int *baseStation, int *axis) {
  bool anglesMeasured = false;

  if (isNewSync(timestamp, state->lastSync)) {
    if (isSync(state, timestamp)) {
      anglesMeasured = processPreviousFrame(state, angles, baseStation, axis);

      if (anglesMeasured) {
        decodeAndApplyBaseStationCalibrationData(state);
      }

      int baseStation = getBaseStationId(state, timestamp);

      if (baseStation == 1 && state->basestationsSynchronizedCount < 2) {
        resetSynchronization(state);
      } else {
        storeSyncData(state, baseStation, timestamp, width);
      }
    } else {
      // 期望为同步脉冲但出现异常，重新同步。
      resetSynchronization(state);
    }
  }

  return anglesMeasured;
}

static bool processWhenSynchronized(pulseProcessor_t *state, int sensor, unsigned int timestamp, unsigned int width, pulseProcessorResult_t* angles, int *baseStation, int *axis) {
  bool anglesMeasured = false;

  if (isSweep(state, timestamp, width)) {
    storeSweepData(state, sensor, timestamp);
  } else {
    anglesMeasured = processSync(state, timestamp, width, angles, baseStation, axis);
  }

  return anglesMeasured;
}


bool pulseProcessorV1ProcessPulse(pulseProcessor_t *state, const pulseProcessorFrame_t* frameData, pulseProcessorResult_t* angles, int *baseStation, int *axis)
{
  bool anglesMeasured = false;

  if (!state->synchronized) {
    synchronize(state, frameData->sensor, frameData->timestamp, frameData->width);
  } else {
    anglesMeasured = processWhenSynchronized(state, frameData->sensor, frameData->timestamp, frameData->width, angles, baseStation, axis);
  }

  return anglesMeasured;
}

static bool isPulseASync(const pulseProcessorPulse_t* pulse) {
  return (pulse->width > SYNC_MIN_WIDTH) && (pulse->width < SYNC_MAX_WIDTH);
}

static bool isPulseASweep(const pulseProcessorPulse_t* pulse) {
  return pulse->width < SWEEP_MAX_WIDTH;
}


/**
 * @brief 在 pulseHistory 中查找第一个 Sync0 脉冲的时间戳，支持一个或两个基站。
 *
 * @param pulseHistory PULSE_PROCESSOR_HISTORY_LENGTH 个脉冲
 * @param[out] sync0Time 写入第一个 Sync0 的时间戳
 * @return 找到同步脉冲的基站数量；若未检测到同步则为 0。
 */
TESTABLE_STATIC int findSyncTime(const pulseProcessorPulse_t pulseHistory[], uint32_t *sync0Time)
{
  int nFoundSync0 = 0;
  int nFoundSync1 = 0;
  uint32_t foundTimes[2];

  bool wasPreviousPulseASweep = false;
  if (isPulseASync(&pulseHistory[0])) {
    wasPreviousPulseASweep = false;
  } else if (isPulseASweep(&pulseHistory[0])) {
    wasPreviousPulseASweep = true;
  } else {
    return 0;
  }

  for (int i=1; i<PULSE_PROCESSOR_HISTORY_LENGTH; i++) {
    if (isPulseASync(&pulseHistory[i])) {
      if (wasPreviousPulseASweep) {
        foundTimes[nFoundSync0] = pulseHistory[i].timestamp;
        nFoundSync0++;

        if (nFoundSync0 == 2) {
          break;
        }
      } else {
        if (nFoundSync0 > 0) {
          uint32_t syncSeparation = TS_DIFF(pulseHistory[i].timestamp, foundTimes[nFoundSync0 - 1]);
          if (syncSeparation > SYNC_MIN_SEPARATION && syncSeparation < SYNC_MAX_SEPARATION) {
            nFoundSync1++;
          } else {
            return 0;
          }
        }
      }

      wasPreviousPulseASweep = false;
    } else if (isPulseASweep(&pulseHistory[i])) {
      wasPreviousPulseASweep = true;
    } else {
      return 0;
    }
  }

  int result = 0;
  if (nFoundSync0 == 2) {
    uint32_t frameLength = TS_DIFF(foundTimes[1], foundTimes[0]);
    bool isFrameLengthWithinBoundaries = (frameLength < (FRAME_LENGTH + MAX_FRAME_LENGTH_NOISE) && frameLength > (FRAME_LENGTH - MAX_FRAME_LENGTH_NOISE));
    if (isFrameLengthWithinBoundaries) {
      *sync0Time = foundTimes[0];
      result = 1;
      if (nFoundSync1 > 0) {
        result = 2;
      }
    }
  }

  return result;
}

/**
 * @brief 根据多个传感器采样到的 Sync0 时间戳计算系统同步时间
 *
 * 该函数将 Sync0 时间戳对 Lighthouse V1 帧长取模，
 * 以估计它们是否可能来自同一套 Lighthouse 系统。
 * 这用于检查多个接收传感器的采样是否一致。
 *
 * @param syncTimes Sync0 时间戳数组
 * @param nSyncTimes syncTimes 数组中的时间戳数量
 * @param[out] syncTime 写入结果同步时间的指针
 * @return true 可计算出可接受的同步时间
 * @return false 采样到的 Sync0 时间戳不合理
 */
TESTABLE_STATIC bool getSystemSyncTime(const uint32_t syncTimes[], size_t nSyncTimes, uint32_t *syncTime)
{
  if (nSyncTimes == 0) {
    return false;
  }

  // 检测采样是否发生回绕
  // 若发生回绕，所有小于 PULSE_PROCESSOR_TIMESTAMP_MAX/2 的样本将加上
  // (1<<TIMESTAMP_BITWIDTH) 以修正回绕
  bool isWrapping = false;
  int wref = syncTimes[0];
  for (size_t i=0; i<nSyncTimes; i++) {
    if (abs(wref - (int)syncTimes[i]) > (PULSE_PROCESSOR_TIMESTAMP_MAX/2)) {
      isWrapping = true;
    }
  }

  int32_t differenceSum = 0;
  int32_t reference = syncTimes[0] % FRAME_LENGTH;
  if (isWrapping && syncTimes[0] < (PULSE_PROCESSOR_TIMESTAMP_MAX/2)) {
    reference = (syncTimes[0] + (1<<PULSE_PROCESSOR_TIMESTAMP_BITWIDTH)) % FRAME_LENGTH;
  }

  int minDiff = INT32_MAX;
  int maxDiff = INT32_MIN;

  for (size_t i=1; i<nSyncTimes; i++) {
    int diff;

    if (isWrapping && (syncTimes[i] < (PULSE_PROCESSOR_TIMESTAMP_MAX/2))) {
      diff = ((syncTimes[i] + (1<<PULSE_PROCESSOR_TIMESTAMP_BITWIDTH)) % FRAME_LENGTH) - reference;
    } else {
      diff = (syncTimes[i] % FRAME_LENGTH) - reference;
    }


    if (diff < minDiff) {
      minDiff = diff;
    }
    if (diff > maxDiff) {
      maxDiff = diff;
    }

    differenceSum += diff;
  }

  if ((maxDiff - minDiff) > MAX_FRAME_LENGTH_NOISE) {
    return false;
  }

  *syncTime = ((int)syncTimes[0] + ((int)differenceSum / (int)nSyncTimes)) & (PULSE_PROCESSOR_TIMESTAMP_MAX);


  return true;
}
