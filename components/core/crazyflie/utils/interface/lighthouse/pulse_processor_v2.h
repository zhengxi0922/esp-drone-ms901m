/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2020 Bitcraze AB
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
 * pulse_processor_v2.h - pulse decoding for lighthouse V2 base stations
 *
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "pulse_processor.h"


/**
 * @brief 处理 Lighthouse V2 系统的脉冲数据
 *
 * @param state 状态
 * @param frameData 帧数据
 * @param baseStation 基站
 * @param axis 轴
 * @return true 已写入角度、基站和轴
 * @return false 无有效结果
 */
bool pulseProcessorV2ProcessPulse(pulseProcessor_t *state, const pulseProcessorFrame_t* frameData, pulseProcessorResult_t* angles, int *baseStation, int *axis);

/**
 * @brief 将 Lighthouse V2 角度转换为 Lighthouse V1 角度
 *
 * @param v2Angle1 第一个 LH V2 角度
 * @param v2Angle2 第二个 LH V2 角度
 * @param v1Angles 输出的 V1 角度
 */
void pulseProcessorV2ConvertToV1Angles(const float v2Angle1, const float v2Angle2, float* v1Angles);

uint8_t pulseProcessorV2AnglesQuality();
