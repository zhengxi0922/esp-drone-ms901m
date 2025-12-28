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
 * tdoaEngineInstance.c - holds the tdoa engine state instance
 */

#include "tdoaEngineInstance.h"
#include "log.h"
#include "param.h"
#include "static_mem.h"

NO_DMA_CCM_SAFE_ZERO_INIT tdoaEngineState_t tdoaEngineState;

LOG_GROUP_START(tdoaEngine)
STATS_CNT_RATE_LOG_ADD(stRx, &tdoaEngineState.stats.packetsReceived)
STATS_CNT_RATE_LOG_ADD(stEst, &tdoaEngineState.stats.packetsToEstimator)
STATS_CNT_RATE_LOG_ADD(stTime, &tdoaEngineState.stats.timeIsGood)
STATS_CNT_RATE_LOG_ADD(stFound, &tdoaEngineState.stats.suitableDataFound)
STATS_CNT_RATE_LOG_ADD(stCc, &tdoaEngineState.stats.clockCorrectionCount)
STATS_CNT_RATE_LOG_ADD(stHit, &tdoaEngineState.stats.contextHitCount)
STATS_CNT_RATE_LOG_ADD(stMiss, &tdoaEngineState.stats.contextMissCount)

LOG_ADD(LOG_FLOAT, cc, &tdoaEngineState.stats.clockCorrection)
LOG_ADD(LOG_UINT16, tof, &tdoaEngineState.stats.tof)
LOG_ADD(LOG_FLOAT, tdoa, &tdoaEngineState.stats.tdoa)
LOG_GROUP_STOP(tdoaEngine)


PARAM_GROUP_START(tdoaEngine)
PARAM_ADD(PARAM_UINT8, logId, &tdoaEngineState.stats.newAnchorId)
PARAM_ADD(PARAM_UINT8, logOthrId, &tdoaEngineState.stats.newRemoteAnchorId)

// 该变量本应不作为参数暴露，因为它会在 CF 固件内部被修改。
// 只有在 LPS 系统模式切换到 TDoA2 或 TDoA3 时才会发生，
// 且这种操作不频繁，因此仍选择将其暴露。
PARAM_ADD(PARAM_UINT8, matchAlgo, &tdoaEngineState.matchingAlgorithm)
PARAM_GROUP_STOP(tdoaEngine)
