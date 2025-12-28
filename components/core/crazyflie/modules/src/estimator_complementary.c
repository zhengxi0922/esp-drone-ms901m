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
 * estimator_complementary.c - a complementary estimator
 */



#include "FreeRTOS.h"
#include "queue.h"
#include "stabilizer.h"
#include "estimator_complementary.h"
#include "sensfusion6.h"
#include "position_estimator.h"
#include "sensors.h"
#include "stabilizer_types.h"
#include "static_mem.h"

#define ATTITUDE_UPDATE_RATE RATE_250_HZ
#define ATTITUDE_UPDATE_DT 1.0/ATTITUDE_UPDATE_RATE

#define POS_UPDATE_RATE RATE_100_HZ
#define POS_UPDATE_DT 1.0/POS_UPDATE_RATE

static bool latestTofMeasurement(tofMeasurement_t* tofMeasurement);

// 激光传感器的 TOF 测量
#define TOF_QUEUE_LENGTH (1)
static xQueueHandle tofDataQueue;
STATIC_MEM_QUEUE_ALLOC(tofDataQueue, TOF_QUEUE_LENGTH, sizeof(tofMeasurement_t));


void estimatorComplementaryInit(void)
{
  tofDataQueue = STATIC_MEM_QUEUE_CREATE(tofDataQueue);

  sensfusion6Init();
}

bool estimatorComplementaryTest(void)
{
  bool pass = true;

  pass &= sensfusion6Test();

  return pass;
}

void estimatorComplementary(state_t *state, sensorData_t *sensorData, control_t *control, const uint32_t tick)
{
  sensorsAcquire(sensorData, tick); // 以全速（1000Hz）读取传感器
  if (RATE_DO_EXECUTE(ATTITUDE_UPDATE_RATE, tick)) {
    sensfusion6UpdateQ(sensorData->gyro.x, sensorData->gyro.y, sensorData->gyro.z,
                       sensorData->acc.x, sensorData->acc.y, sensorData->acc.z,
                       ATTITUDE_UPDATE_DT);

    // 保存姿态，并按旧版 CF2 机体坐标系调整
    sensfusion6GetEulerRPY(&state->attitude.roll, &state->attitude.pitch, &state->attitude.yaw);

    // 保存四元数，希望未来可用于更好的控制器。
    // 注意：此处未按旧版坐标系调整
    sensfusion6GetQuaternion(
      &state->attitudeQuaternion.x,
      &state->attitudeQuaternion.y,
      &state->attitudeQuaternion.z,
      &state->attitudeQuaternion.w);

    state->acc.z = sensfusion6GetAccZWithoutGravity(sensorData->acc.x,
                                                    sensorData->acc.y,
                                                    sensorData->acc.z);

    positionUpdateVelocity(state->acc.z, ATTITUDE_UPDATE_DT);
  }

  if (RATE_DO_EXECUTE(POS_UPDATE_RATE, tick)) {
    tofMeasurement_t tofMeasurement;

    latestTofMeasurement(&tofMeasurement);
    positionEstimate(state, sensorData, &tofMeasurement, POS_UPDATE_DT, tick);
  }
}

static bool latestTofMeasurement(tofMeasurement_t* tofMeasurement) {
  return xQueuePeek(tofDataQueue, tofMeasurement, 0) == pdTRUE;
}

static bool overwriteMeasurement(xQueueHandle queue, void *measurement)
{
  portBASE_TYPE result;
  bool isInInterrupt = false;//(SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;

  if (isInInterrupt) {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    result = xQueueOverwriteFromISR(queue, measurement, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken == pdTRUE)
    {
      portYIELD();
    }
  } else {
    result = xQueueOverwrite(queue, measurement);
  }
  return (result==pdTRUE);
}


bool estimatorComplementaryEnqueueTOF(const tofMeasurement_t *tof)
{
  // 沿 z_B 轴的对地距离（米）
  return overwriteMeasurement(tofDataQueue, (void *)tof);
}
