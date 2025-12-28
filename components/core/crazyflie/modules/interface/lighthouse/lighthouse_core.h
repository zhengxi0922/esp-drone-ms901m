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
 * lighthouse_core.h - central part of the lighthouse positioning system
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "pulse_processor.h"

typedef struct {
  bool isSyncFrame;
  pulseProcessorFrame_t data;
} lighthouseUartFrame_t;

typedef struct {
  int sampleCount;
  int hitCount;
} lighthouseBsIdentificationData_t;

void lighthouseCoreInit();
void lighthouseCoreTask(void *param);

/**
 * @brief 为系统中的单个基站设置标定数据
 *
 * @param baseStation   基站 ID
 * @param calibration   标定数据指针
 */
void lighthouseCoreSetCalibrationData(const uint8_t baseStation, const lighthouseCalibration_t* calibration);

/**
 * @brief 将 RAM 中的当前数据复制到永久存储
 *
 * @param baseStation  需要存储数据的基站 ID
 * @param geoData      为 true 时写入基站几何数据
 * @param calibData    为 true 时写入基站标定数据
 * @return true 表示写入成功
 */
bool lighthouseCorePersistData(const uint8_t baseStation, const bool geoData, const bool calibData);
