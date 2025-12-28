/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
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
 * statsCnt.h - utitlity for logging rates
 */

#pragma once

#include <stdint.h>
#include "log.h"

/**
 * @brief 用于跟踪事件速率的结构体
 */
typedef struct {
    uint32_t count;
    uint32_t latestCount;
    uint32_t latestAveragingMs;
    float latestRate;
    uint32_t intervalMs;
} statsCntRateCounter_t;

/**
 * @brief 初始化 statsCntRateLogger_t 结构体。
 *
 * @param logger 要初始化的计数器
 * @param averagingIntervalMs 速率计算的时间间隔（毫秒）
 */
void statsCntRateCounterInit(statsCntRateCounter_t* counter, uint32_t averagingIntervalMs);

/**
 * @brief 当距离上次计算的时间超过配置的间隔时计算新的速率。
 *
 * @param counter 需要更新的速率计数器
 * @param now_ms 当前系统时间（毫秒）
 * @return float 最新计算的速率
 */
float statsCntRateCounterUpdate(statsCntRateCounter_t* counter, uint32_t now_ms);


// 日志模块集成 -------------------------------------------------------

/**
 * @brief 用于与日志 USB 系统一起使用速率计数器的结构体。
 */
typedef struct {
    // logByFunction_t 必须是该结构体的第一个元素，
    // 因为该结构体的指针会被转换为 logByFunction_t* 并被日志模块使用
    logByFunction_t logByFunction;
    statsCntRateCounter_t rateCounter;
} statsCntRateLogger_t;

/**
 * @brief 初始化 statsCntRateLogger_t 结构体的宏。
 *
 * @param LOGGER 指向 statsCntRateLogger_t 的指针
 * @param INTERVAL_MS 速率计算的时间间隔（毫秒）
 */
#define STATS_CNT_RATE_INIT(LOGGER, INTERVAL_MS) statsCntRateLoggerInit(LOGGER, INTERVAL_MS)

#define STATS_CNT_RATE_DEFINE(NAME, INTERVAL_MS) statsCntRateLogger_t NAME = {.logByFunction = {.data = &NAME, .aquireFloat = statsCntRateLogHandler}, .rateCounter = {.intervalMs = (INTERVAL_MS), .count = 0, .latestCount = 0, .latestAveragingMs = 0, .latestRate = 0}}

/**
 * @brief 给 statsCntRateLogger_t 添加一次事件，即增加内部计数器。
 *
 * @param LOGGER 指向 statsCntRateLogger_t 的指针
 */
#define STATS_CNT_RATE_EVENT(LOGGER) ((LOGGER)->rateCounter.count++)

/**
 * @brief 给 statsCntRateLogger_t 添加 CNT 次事件，即内部计数器增加 CNT。
 *
 * @param LOGGER 指向 statsCntRateLogger_t 的指针
 * @param CNT    增加的计数数量
 */
#define STATS_CNT_RATE_MULTI_EVENT(LOGGER, CNT) ((LOGGER)->rateCounter.count += CNT)

/**
 * @brief 将 statsCntRateLogger_t 作为速率日志添加的宏。
 * 用法类似于 LOG_GROUP_START() - LOG_GROUP_STOP() 块中的 LOG_ADD()。
 *
 * @param LOGGER 指向 statsCntRateLogger_t 的指针
 */
#define STATS_CNT_RATE_LOG_ADD(NAME, LOGGER) LOG_ADD_BY_FUNCTION(LOG_FLOAT, NAME, LOGGER)

/**
 * @brief 初始化 statsCntRateLogger_t 结构体。
 *
 * @param logger 要初始化的计数器
 * @param averagingIntervalMs 速率计算的时间间隔（毫秒）
 */
void statsCntRateLoggerInit(statsCntRateLogger_t* logger, uint32_t averagingIntervalMs);

/**
 * @brief 日志模块用于获取日志值的处理函数。
 * 调用 statsCntRateCounterUpdate() 更新记录器并获取速率。
 *
 * @param timestamp 当前系统时间（毫秒）
 * @param data 指向 statsCntRateLogger_t 的指针
 * @return float 最新计算的速率
 */
float statsCntRateLogHandler(uint32_t timestamp, void* data);
