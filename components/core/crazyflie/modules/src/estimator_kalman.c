/**
 * Authored by Michael Hamer (http://www.mikehamer.info), June 2016
 * Thank you to Mark Mueller (www.mwm.im) for advice during implementation,
 * and for derivation of the original filter in the below-cited paper.
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
 * ============================================================================
 *
 * The Kalman filter implemented in this file is based on the papers:
 *
 * "Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation"
 * http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=7139421
 *
 * and
 *
 * "Covariance Correction Step for Kalman Filtering with an Attitude"
 * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
 *
 * Academic citation would be appreciated.
 *
 * BIBTEX ENTRIES:
      @INPROCEEDINGS{MuellerHamerUWB2015,
      author  = {Mueller, Mark W and Hamer, Michael and D’Andrea, Raffaello},
      title   = {Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation},
      booktitle = {2015 IEEE International Conference on Robotics and Automation (ICRA)},
      year    = {2015},
      month   = {May},
      pages   = {1730-1736},
      doi     = {10.1109/ICRA.2015.7139421},
      ISSN    = {1050-4729}}

      @ARTICLE{MuellerCovariance2016,
      author={Mueller, Mark W and Hehn, Markus and D’Andrea, Raffaello},
      title={Covariance Correction Step for Kalman Filtering with an Attitude},
      journal={Journal of Guidance, Control, and Dynamics},
      pages={1--7},
      year={2016},
      publisher={American Institute of Aeronautics and Astronautics}}
 *
 * ============================================================================
 *
 * MAJOR CHANGELOG:
 * 2016.06.28, Mike Hamer: Initial version
 * 2019.04.12, Kristoffer Richardsson: Refactored, separated kalman implementation from OS related functionality
 *
 */
#include <inttypes.h>
#include "kalman_core.h"
#include "estimator_kalman.h"
#include "kalman_supervisor.h"


#include "stm32_legacy.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "sensors.h"
#include "static_mem.h"

#include "system.h"
#include "log.h"
#include "param.h"
#include "physicalConstants.h"

#include "statsCnt.h"
#include "rateSupervisor.h"
#include "config.h"

#define DEBUG_MODULE "ESTKALMAN"
#include "debug_cf.h"


// #define KALMAN_USE_BARO_UPDATE


/**
 * 另外，滤波器支持将额外传感器并入状态估计
 *
 * 可通过以下外部函数实现：
 * - bool estimatorKalmanEnqueueUWBPacket(uwbPacket_t *uwb)
 * - bool estimatorKalmanEnqueuePosition(positionMeasurement_t *pos)
 * - bool estimatorKalmanEnqueueDistance(distanceMeasurement_t *dist)
 *
 * 同时也包含下列内部函数与数据类型
 */

// 到点距离测量
static xQueueHandle distDataQueue;
STATIC_MEM_QUEUE_ALLOC(distDataQueue, 10, sizeof(distanceMeasurement_t));

static inline bool stateEstimatorHasDistanceMeasurement(distanceMeasurement_t *dist) {
  return (pdTRUE == xQueueReceive(distDataQueue, dist, 0));
}

// Crazyflie 位置直接测量
static xQueueHandle posDataQueue;
STATIC_MEM_QUEUE_ALLOC(posDataQueue, 10, sizeof(positionMeasurement_t));

static inline bool stateEstimatorHasPositionMeasurement(positionMeasurement_t *pos) {
  return (pdTRUE == xQueueReceive(posDataQueue, pos, 0));
}

// Crazyflie 位姿直接测量
static xQueueHandle poseDataQueue;
STATIC_MEM_QUEUE_ALLOC(poseDataQueue, 10, sizeof(poseMeasurement_t));

static inline bool stateEstimatorHasPoseMeasurement(poseMeasurement_t *pose) {
  return (pdTRUE == xQueueReceive(poseDataQueue, pose, 0));
}

// UWB 收发测量
static xQueueHandle tdoaDataQueue;
STATIC_MEM_QUEUE_ALLOC(tdoaDataQueue, 10, sizeof(tdoaMeasurement_t));

static inline bool stateEstimatorHasTDOAPacket(tdoaMeasurement_t *uwb) {
  return (pdTRUE == xQueueReceive(tdoaDataQueue, uwb, 0));
}

// 光流测量（dnx, dny）
static xQueueHandle flowDataQueue;
STATIC_MEM_QUEUE_ALLOC(flowDataQueue, 10, sizeof(flowMeasurement_t));

static inline bool stateEstimatorHasFlowPacket(flowMeasurement_t *flow) {
  return (pdTRUE == xQueueReceive(flowDataQueue, flow, 0));
}

// 激光传感器 TOF 测量
static xQueueHandle tofDataQueue;
STATIC_MEM_QUEUE_ALLOC(tofDataQueue, 10, sizeof(tofMeasurement_t));

static inline bool stateEstimatorHasTOFPacket(tofMeasurement_t *tof) {
  return (pdTRUE == xQueueReceive(tofDataQueue, tof, 0));
}

// 房间 Z 方向的绝对高度测量
static xQueueHandle heightDataQueue;
STATIC_MEM_QUEUE_ALLOC(heightDataQueue, 10, sizeof(heightMeasurement_t));

static inline bool stateEstimatorHasHeightPacket(heightMeasurement_t *height) {
  return (pdTRUE == xQueueReceive(heightDataQueue, height, 0));
}


static xQueueHandle yawErrorDataQueue;
STATIC_MEM_QUEUE_ALLOC(yawErrorDataQueue, 10, sizeof(yawErrorMeasurement_t));

static inline bool stateEstimatorHasYawErrorPacket(yawErrorMeasurement_t *error)
{
  return (pdTRUE == xQueueReceive(yawErrorDataQueue, error, 0));
}

// static xQueueHandle sweepAnglesDataQueue;
// STATIC_MEM_QUEUE_ALLOC(sweepAnglesDataQueue, 10, sizeof(sweepAngleMeasurement_t));

// static inline bool stateEstimatorHasSweepAnglesPacket(sweepAngleMeasurement_t *angles)
// {
//   return (pdTRUE == xQueueReceive(sweepAnglesDataQueue, angles, 0));
// }

// 用于指示从稳定器循环获取到数据的信号量
static SemaphoreHandle_t runTaskSemaphore;

// 保护任务与稳定器循环函数间共享数据的互斥量
static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;


/**
 * 估计器使用的常量
 */

// 推力映射：65536 对应 60 克
#define CONTROL_TO_ACC (GRAVITY_MAGNITUDE*60.0f/(CF_MASS*1000.0f)/65536.0f)


/**
 * 调参参数
 */
#define PREDICT_RATE RATE_100_HZ // 比 IMU 500Hz 更新率更慢
#define BARO_RATE RATE_25_HZ

// 动力学从静止转为飞行的阈值
#define IN_FLIGHT_THRUST_THRESHOLD (GRAVITY_MAGNITUDE*0.1f)
#define IN_FLIGHT_TIME_THRESHOLD (500)

// 协方差边界，按理不应触发，但有时会触发... 为什么？
#define MAX_COVARIANCE (100)
#define MIN_COVARIANCE (1e-6f)



/**
 * 四旋翼状态
 *
 * 内部估计状态包括：
 * - X, Y, Z：全局坐标系下的位置
 * - PX, PY, PZ：机体系下的速度
 * - D0, D1, D2：姿态误差
 *
 * 更多信息请参考论文
 */

NO_DMA_CCM_SAFE_ZERO_INIT static kalmanCoreData_t coreData;

/**
 * 内部变量。注意 static 声明会进行默认初始化（为 0）
 */

static bool isInit = false;

static Axis3f accAccumulator;
static float thrustAccumulator;
static Axis3f gyroAccumulator;
static float baroAslAccumulator;
static uint32_t accAccumulatorCount;
static uint32_t thrustAccumulatorCount;
static uint32_t gyroAccumulatorCount;
static uint32_t baroAccumulatorCount;
static bool quadIsFlying = false;
static uint32_t lastFlightCmd;
static uint32_t takeoffTime;

// 用于在尽量少加锁的情况下运行任务与稳定器循环的数据
static state_t taskEstimatorState; // 任务产生的估计器状态，必要时复制给稳定器
static Axis3f gyroSnapshot; // 最新陀螺数据快照，供任务使用
static Axis3f accSnapshot; // 最新加速度数据快照，供任务使用

// 统计
#define ONE_SECOND 1000
static STATS_CNT_RATE_DEFINE(updateCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(predictionCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(baroUpdateCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(finalizeCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(measurementAppendedCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(measurementNotAppendedCounter, ONE_SECOND);

static rateSupervisor_t rateSupervisorContext;

#define WARNING_HOLD_BACK_TIME M2T(2000)
static uint32_t warningBlockTime = 0;

#ifdef KALMAN_USE_BARO_UPDATE
static const bool useBaroUpdate = true;
#else
static const bool useBaroUpdate = false;
#endif

static void kalmanTask(void* parameters);
static bool predictStateForward(uint32_t osTick, float dt);
static bool updateQueuedMeasurments(const Axis3f *gyro, const uint32_t tick);

STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(kalmanTask, KALMAN_TASK_STACKSIZE);

// --------------------------------------------------

// 系统启动时调用一次
void estimatorKalmanTaskInit() {
  distDataQueue = STATIC_MEM_QUEUE_CREATE(distDataQueue);
  posDataQueue = STATIC_MEM_QUEUE_CREATE(posDataQueue);
  poseDataQueue = STATIC_MEM_QUEUE_CREATE(poseDataQueue);
  tdoaDataQueue = STATIC_MEM_QUEUE_CREATE(tdoaDataQueue);
  flowDataQueue = STATIC_MEM_QUEUE_CREATE(flowDataQueue);
  tofDataQueue = STATIC_MEM_QUEUE_CREATE(tofDataQueue);
  heightDataQueue = STATIC_MEM_QUEUE_CREATE(heightDataQueue);
  yawErrorDataQueue = STATIC_MEM_QUEUE_CREATE(yawErrorDataQueue);
  //sweepAnglesDataQueue = STATIC_MEM_QUEUE_CREATE(sweepAnglesDataQueue);

  vSemaphoreCreateBinary(runTaskSemaphore);

  dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);

  STATIC_MEM_TASK_CREATE(kalmanTask, kalmanTask, KALMAN_TASK_NAME, NULL, KALMAN_TASK_PRI);

  isInit = true;
}

bool estimatorKalmanTaskTest() {
  return isInit;
}

static void kalmanTask(void* parameters) {
  systemWaitStart();

  uint32_t lastPrediction = xTaskGetTickCount();
  uint32_t nextPrediction = xTaskGetTickCount();
  uint32_t lastPNUpdate = xTaskGetTickCount();
  uint32_t nextBaroUpdate = xTaskGetTickCount();

  rateSupervisorInit(&rateSupervisorContext, xTaskGetTickCount(), M2T(1000), 99, 101, 1);

  while (true) {
    xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);

    // 若客户端通过参数更新触发估计器重置
    if (coreData.resetEstimation) {
      estimatorKalmanInit();
      paramSetInt(paramGetVarId("kalman", "resetEstimation"), 0);
    }

    // 跟踪是否已更新状态，从而决定是否需要最终化
    bool doneUpdate = false;

    uint32_t osTick = xTaskGetTickCount(); // 若精度能高于 1ms 会更好...

  #ifdef KALMAN_DECOUPLE_XY
    kalmanCoreDecoupleXY(&coreData);
  #endif

    // 运行系统动力学进行状态预测
    if (osTick >= nextPrediction) { // 按 PREDICT_RATE 更新
      float dt = T2S(osTick - lastPrediction);
      if (predictStateForward(osTick, dt)) {
        lastPrediction = osTick;
        doneUpdate = true;
        STATS_CNT_RATE_EVENT(&predictionCounter);
      }

      nextPrediction = osTick + S2T(1.0f / PREDICT_RATE);

      if (!rateSupervisorValidate(&rateSupervisorContext, T2M(osTick))) {
        DEBUG_PRINT("WARNING: Kalman prediction rate low (%"PRIu32")\n", rateSupervisorLatestCount(&rateSupervisorContext));
      }
    }

    /**
     * 每个循环添加过程噪声，而不是每次预测时添加
     */
    {
      float dt = T2S(osTick - lastPNUpdate);
      if (dt > 0.0f) {
        kalmanCoreAddProcessNoise(&coreData, dt);
        lastPNUpdate = osTick;
      }
    }

    /**
     * 使用气压计测量更新状态估计
     */
    // 累积气压计测量
    if (useBaroUpdate) {
      if (osTick > nextBaroUpdate // 按 BARO_RATE 更新
          && baroAccumulatorCount > 0)
      {
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        float baroAslAverage = baroAslAccumulator / baroAccumulatorCount;
        baroAslAccumulator = 0;
        baroAccumulatorCount = 0;
        xSemaphoreGive(dataMutex);

        kalmanCoreUpdateWithBaro(&coreData, baroAslAverage, quadIsFlying);

        nextBaroUpdate = osTick + S2T(1.0f / BARO_RATE);
        doneUpdate = true;

        STATS_CNT_RATE_EVENT(&baroUpdateCounter);
      }
    }

    {
      Axis3f gyro;
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      memcpy(&gyro, &gyroSnapshot, sizeof(gyro));
      xSemaphoreGive(dataMutex);

      if(updateQueuedMeasurments(&gyro, osTick)) {
        doneUpdate = true;
      }
    }

    /**
     * 如果已完成更新，则进行状态最终化：
     * - 将姿态误差并入机体姿态四元数，
     * - 将机体姿态转换为旋转矩阵供下一次预测使用，
     * - 确保协方差矩阵正确
     */

    if (doneUpdate)
    {
      kalmanCoreFinalize(&coreData, osTick);
      STATS_CNT_RATE_EVENT(&finalizeCounter);
      if (! kalmanSupervisorIsStateWithinBounds(&coreData)) {
        coreData.resetEstimation = true;

        if (osTick > warningBlockTime) {
          warningBlockTime = osTick + WARNING_HOLD_BACK_TIME;
          DEBUG_PRINT("State out of bounds, resetting\n");
        }
      }
    }

    /**
     * 最后将内部状态对外输出。
     * 每轮都执行，因为外部状态包含部分传感器数据
     */
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    kalmanCoreExternalizeState(&coreData, &taskEstimatorState, &accSnapshot, osTick);
    xSemaphoreGive(dataMutex);

    STATS_CNT_RATE_EVENT(&updateCounter);
  }
}

void estimatorKalman(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick)
{
  // 该函数由稳定器循环调用，需尽快返回。
  // dataMutex 仅应被任务短时间持有。
  xSemaphoreTake(dataMutex, portMAX_DELAY);

  // 对最近的 IMU 测量求平均。因为预测循环慢于 IMU 循环，
  // 但外部需要更高频率的 IMU 信息（用于机体角速度控制）。
  if (sensorsReadAcc(&sensors->acc)) {
    accAccumulator.x += sensors->acc.x;
    accAccumulator.y += sensors->acc.y;
    accAccumulator.z += sensors->acc.z;
    accAccumulatorCount++;
  }

  if (sensorsReadGyro(&sensors->gyro)) {
    gyroAccumulator.x += sensors->gyro.x;
    gyroAccumulator.y += sensors->gyro.y;
    gyroAccumulator.z += sensors->gyro.z;
    gyroAccumulatorCount++;
  }

  // 对控制器生成的最近推力指令求平均
  thrustAccumulator += control->thrust;
  thrustAccumulatorCount++;

  // 对气压计数据求平均
  if (useBaroUpdate) {
    if (sensorsReadBaro(&sensors->baro)) {
      baroAslAccumulator += sensors->baro.asl;
      baroAccumulatorCount++;
    }
  }

  // 复制传感器数据供任务使用
  memcpy(&gyroSnapshot, &sensors->gyro, sizeof(gyroSnapshot));
  memcpy(&accSnapshot, &sensors->acc, sizeof(accSnapshot));

  // 复制任务计算出的最新状态
  memcpy(state, &taskEstimatorState, sizeof(state_t));
  xSemaphoreGive(dataMutex);

  xSemaphoreGive(runTaskSemaphore);
}

static bool predictStateForward(uint32_t osTick, float dt) {
  if (gyroAccumulatorCount == 0
      || accAccumulatorCount == 0
      || thrustAccumulatorCount == 0)
  {
    return false;
  }

  xSemaphoreTake(dataMutex, portMAX_DELAY);

  // 陀螺单位为度/秒，但估计器需要弧度/秒
  Axis3f gyroAverage;
  gyroAverage.x = gyroAccumulator.x * DEG_TO_RAD / gyroAccumulatorCount;
  gyroAverage.y = gyroAccumulator.y * DEG_TO_RAD / gyroAccumulatorCount;
  gyroAverage.z = gyroAccumulator.z * DEG_TO_RAD / gyroAccumulatorCount;

  // 加速度计单位为 G，但估计器需要 m/s^2
  Axis3f accAverage;
  accAverage.x = accAccumulator.x * GRAVITY_MAGNITUDE / accAccumulatorCount;
  accAverage.y = accAccumulator.y * GRAVITY_MAGNITUDE / accAccumulatorCount;
  accAverage.z = accAccumulator.z * GRAVITY_MAGNITUDE / accAccumulatorCount;

  // 推力单位为克，需要换算为 m/s^2
  float thrustAverage = thrustAccumulator * CONTROL_TO_ACC / thrustAccumulatorCount;

  accAccumulator = (Axis3f){.axis={0}};
  accAccumulatorCount = 0;
  gyroAccumulator = (Axis3f){.axis={0}};
  gyroAccumulatorCount = 0;
  thrustAccumulator = 0;
  thrustAccumulatorCount = 0;

  xSemaphoreGive(dataMutex);

  // TODO：寻找更好的飞行状态判定
  // 当前假设推力足够大即开始飞行，且暂不停止“飞行”状态。
  if (thrustAverage > IN_FLIGHT_THRUST_THRESHOLD) {
    lastFlightCmd = osTick;
    if (!quadIsFlying) {
      takeoffTime = lastFlightCmd;
    }
  }
  quadIsFlying = (osTick-lastFlightCmd) < IN_FLIGHT_TIME_THRESHOLD;

  kalmanCorePredict(&coreData, thrustAverage, &accAverage, &gyroAverage, dt, quadIsFlying);

  return true;
}


static bool updateQueuedMeasurments(const Axis3f *gyro, const uint32_t tick) {
  bool doneUpdate = false;
  /**
   * 传感器测量可能零散且快于稳定器循环频率，
   * 因此我们消耗自上次循环以来的所有测量，而非累积。
   */

  tofMeasurement_t tof;
  while (stateEstimatorHasTOFPacket(&tof))
  {
    kalmanCoreUpdateWithTof(&coreData, &tof);
    doneUpdate = true;
  }

  yawErrorMeasurement_t yawError;
  while (stateEstimatorHasYawErrorPacket(&yawError))
  {
    kalmanCoreUpdateWithYawError(&coreData, &yawError);
    doneUpdate = true;
  }

  heightMeasurement_t height;
  while (stateEstimatorHasHeightPacket(&height))
  {
    kalmanCoreUpdateWithAbsoluteHeight(&coreData, &height);
    doneUpdate = true;
  }

  distanceMeasurement_t dist;
  while (stateEstimatorHasDistanceMeasurement(&dist))
  {
    kalmanCoreUpdateWithDistance(&coreData, &dist);
    doneUpdate = true;
  }

  positionMeasurement_t pos;
  while (stateEstimatorHasPositionMeasurement(&pos))
  {
    kalmanCoreUpdateWithPosition(&coreData, &pos);
    doneUpdate = true;
  }

  poseMeasurement_t pose;
  while (stateEstimatorHasPoseMeasurement(&pose))
  {
    kalmanCoreUpdateWithPose(&coreData, &pose);
    doneUpdate = true;
  }

  tdoaMeasurement_t tdoa;
  while (stateEstimatorHasTDOAPacket(&tdoa))
  {
    kalmanCoreUpdateWithTDOA(&coreData, &tdoa);
    doneUpdate = true;
  }

  flowMeasurement_t flow;
  while (stateEstimatorHasFlowPacket(&flow))
  {
    kalmanCoreUpdateWithFlow(&coreData, &flow, gyro);
    doneUpdate = true;
  }

  // sweepAngleMeasurement_t angles;
  // while (stateEstimatorHasSweepAnglesPacket(&angles))
  // {
  //   kalmanCoreUpdateWithSweepAngles(&coreData, &angles, tick);
  //   doneUpdate = true;
  // }

  return doneUpdate;
}

// 当该估计器被激活时调用
void estimatorKalmanInit(void) {
  xQueueReset(distDataQueue);
  xQueueReset(posDataQueue);
  xQueueReset(poseDataQueue);
  xQueueReset(tdoaDataQueue);
  xQueueReset(flowDataQueue);
  xQueueReset(tofDataQueue);

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  accAccumulator = (Axis3f){.axis={0}};
  gyroAccumulator = (Axis3f){.axis={0}};
  thrustAccumulator = 0;
  baroAslAccumulator = 0;

  accAccumulatorCount = 0;
  gyroAccumulatorCount = 0;
  thrustAccumulatorCount = 0;
  baroAccumulatorCount = 0;
  xSemaphoreGive(dataMutex);

  kalmanCoreInit(&coreData);
}

static bool appendMeasurement(xQueueHandle queue, void *measurement)
{
  portBASE_TYPE result;
  bool isInInterrupt = 0; // TODO：= (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;

  if (isInInterrupt) {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    result = xQueueSendFromISR(queue, measurement, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken == pdTRUE)
    {
      portYIELD();
    }
  } else {
    result = xQueueSend(queue, measurement, 0);
  }

  if (result == pdTRUE) {
    STATS_CNT_RATE_EVENT(&measurementAppendedCounter);
    return true;
  } else {
    STATS_CNT_RATE_EVENT(&measurementNotAppendedCounter);
    return true;
  }
}

bool estimatorKalmanEnqueueTDOA(const tdoaMeasurement_t *uwb)
{
  ASSERT(isInit);
  return appendMeasurement(tdoaDataQueue, (void *)uwb);
}

bool estimatorKalmanEnqueuePosition(const positionMeasurement_t *pos)
{
  ASSERT(isInit);
  return appendMeasurement(posDataQueue, (void *)pos);
}

bool estimatorKalmanEnqueuePose(const poseMeasurement_t *pose)
{
  ASSERT(isInit);
  return appendMeasurement(poseDataQueue, (void *)pose);
}

bool estimatorKalmanEnqueueDistance(const distanceMeasurement_t *dist)
{
  ASSERT(isInit);
  return appendMeasurement(distDataQueue, (void *)dist);
}

bool estimatorKalmanEnqueueFlow(const flowMeasurement_t *flow)
{
  // 光流测量（dnx, dny），单位为累计像素
  ASSERT(isInit);
  return appendMeasurement(flowDataQueue, (void *)flow);
}

bool estimatorKalmanEnqueueTOF(const tofMeasurement_t *tof)
{
  // 沿 z_B 轴的对地距离（米）
  ASSERT(isInit);
  return appendMeasurement(tofDataQueue, (void *)tof);
}

bool estimatorKalmanEnqueueAbsoluteHeight(const heightMeasurement_t *height)
{
  // 沿 z 轴的对地距离（米）
  ASSERT(isInit);
  return appendMeasurement(heightDataQueue, (void *)height);
}

bool estimatorKalmanEnqueueYawError(const yawErrorMeasurement_t* error)
{
  ASSERT(isInit);
  return appendMeasurement(yawErrorDataQueue, (void *)error);
}

// bool estimatorKalmanEnqueueSweepAngles(const sweepAngleMeasurement_t *angles)
// {
//   ASSERT(isInit);
//   return appendMeasurement(sweepAnglesDataQueue, (void *)angles);
// }

bool estimatorKalmanTest(void)
{
  return isInit;
}

void estimatorKalmanGetEstimatedPos(point_t* pos) {
  pos->x = coreData.S[KC_STATE_X];
  pos->y = coreData.S[KC_STATE_Y];
  pos->z = coreData.S[KC_STATE_Z];
}

void estimatorKalmanGetEstimatedRot(float * rotationMatrix) {
  memcpy(rotationMatrix, coreData.R, 9*sizeof(float));
}

// 临时开发日志组
LOG_GROUP_START(kalman_states)
  LOG_ADD(LOG_FLOAT, ox, &coreData.S[KC_STATE_X])
  LOG_ADD(LOG_FLOAT, oy, &coreData.S[KC_STATE_Y])
  LOG_ADD(LOG_FLOAT, vx, &coreData.S[KC_STATE_PX])
  LOG_ADD(LOG_FLOAT, vy, &coreData.S[KC_STATE_PY])
LOG_GROUP_STOP(kalman_states)


// 默认日志组
LOG_GROUP_START(kalman)
  LOG_ADD(LOG_UINT8, inFlight, &quadIsFlying)
  LOG_ADD(LOG_FLOAT, stateX, &coreData.S[KC_STATE_X])
  LOG_ADD(LOG_FLOAT, stateY, &coreData.S[KC_STATE_Y])
  LOG_ADD(LOG_FLOAT, stateZ, &coreData.S[KC_STATE_Z])
  LOG_ADD(LOG_FLOAT, statePX, &coreData.S[KC_STATE_PX])
  LOG_ADD(LOG_FLOAT, statePY, &coreData.S[KC_STATE_PY])
  LOG_ADD(LOG_FLOAT, statePZ, &coreData.S[KC_STATE_PZ])
  LOG_ADD(LOG_FLOAT, stateD0, &coreData.S[KC_STATE_D0])
  LOG_ADD(LOG_FLOAT, stateD1, &coreData.S[KC_STATE_D1])
  LOG_ADD(LOG_FLOAT, stateD2, &coreData.S[KC_STATE_D2])
  LOG_ADD(LOG_FLOAT, varX, &coreData.P[KC_STATE_X][KC_STATE_X])
  LOG_ADD(LOG_FLOAT, varY, &coreData.P[KC_STATE_Y][KC_STATE_Y])
  LOG_ADD(LOG_FLOAT, varZ, &coreData.P[KC_STATE_Z][KC_STATE_Z])
  LOG_ADD(LOG_FLOAT, varPX, &coreData.P[KC_STATE_PX][KC_STATE_PX])
  LOG_ADD(LOG_FLOAT, varPY, &coreData.P[KC_STATE_PY][KC_STATE_PY])
  LOG_ADD(LOG_FLOAT, varPZ, &coreData.P[KC_STATE_PZ][KC_STATE_PZ])
  LOG_ADD(LOG_FLOAT, varD0, &coreData.P[KC_STATE_D0][KC_STATE_D0])
  LOG_ADD(LOG_FLOAT, varD1, &coreData.P[KC_STATE_D1][KC_STATE_D1])
  LOG_ADD(LOG_FLOAT, varD2, &coreData.P[KC_STATE_D2][KC_STATE_D2])
  LOG_ADD(LOG_FLOAT, q0, &coreData.q[0])
  LOG_ADD(LOG_FLOAT, q1, &coreData.q[1])
  LOG_ADD(LOG_FLOAT, q2, &coreData.q[2])
  LOG_ADD(LOG_FLOAT, q3, &coreData.q[3])

  STATS_CNT_RATE_LOG_ADD(rtUpdate, &updateCounter)
  STATS_CNT_RATE_LOG_ADD(rtPred, &predictionCounter)
  STATS_CNT_RATE_LOG_ADD(rtBaro, &baroUpdateCounter)
  STATS_CNT_RATE_LOG_ADD(rtFinal, &finalizeCounter)
  STATS_CNT_RATE_LOG_ADD(rtApnd, &measurementAppendedCounter)
  STATS_CNT_RATE_LOG_ADD(rtRej, &measurementNotAppendedCounter)
LOG_GROUP_STOP(kalman)

PARAM_GROUP_START(kalman)
  PARAM_ADD(PARAM_UINT8, resetEstimation, &coreData.resetEstimation)
  PARAM_ADD(PARAM_UINT8, quadIsFlying, &quadIsFlying)
PARAM_GROUP_STOP(kalman)
