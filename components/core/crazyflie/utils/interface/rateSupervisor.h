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
 * rateSupervisor.h - functionality to supervise the rate of modules
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t count;
    uint32_t expectedMin;
    uint32_t expectedMax;
    uint32_t nextEvaluationTimeMs;
    uint32_t evaluationIntervalMs;
    uint32_t latestCount;
    uint8_t skip;
} rateSupervisor_t;

/**
 * @brief 初始化 rateSupervisor_t 结构体用于速率测量
 *
 * @param context 要初始化的结构体
 * @param osTimeMs 当前操作系统时间（毫秒）
 * @param evaluationIntervalMs 评估速率的频率（毫秒）
 * @param minCount 每个评估周期期望的最小校验次数
 * @param maxCount 每个评估周期期望的最大校验次数
 * @param skip 需要跳过的初始评估次数，用于系统预热后再报告问题
 */
void rateSupervisorInit(rateSupervisor_t* context, const uint32_t osTimeMs, const uint32_t evaluationIntervalMs, const uint32_t minCount, const uint32_t maxCount, const uint8_t skip);

/**
 * @brief 校验某个流程的速率。该函数应由需要监督速率的流程调用。
 * 调用时会增加计数器，若评估周期已到则进行速率评估。
 *
 * @param context rateSupervisor_t
 * @param osTimeMs 当前操作系统时间（毫秒）
 * @return true 测得速率在范围内，或尚未到达下一次评估时间
 * @return false 测得速率过低或过高
 */
bool rateSupervisorValidate(rateSupervisor_t* context, const uint32_t osTimeMs);

/**
 * @brief 获取最新计数。用于在校验失败后显示计数值。
 *
 * @param context rateSupervisor_t
 * @return uint32_t 最新评估时的计数
 */
uint32_t rateSupervisorLatestCount(rateSupervisor_t* context);
