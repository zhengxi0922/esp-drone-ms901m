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
 */

#include "kalman_core.h"
#include "cfassert.h"

#include "outlierFilter.h"
#include "physicalConstants.h"

#include "log.h"
#include "debug_cf.h"
#include "param.h"
#include "math3d.h"
#include "xtensa_math.h"
#include "static_mem.h"

//#include "lighthouse_calibration.h"

// #define DEBUG_STATE_CHECK

// 俯仰与横滚回零系数
#ifdef LPS_2D_POSITION_HEIGHT
#define ROLLPITCH_ZERO_REVERSION (0.0f)
#else
#define ROLLPITCH_ZERO_REVERSION (0.001f)
#endif


/**
 * 支持与工具函数
 */

#ifdef DEBUG_STATE_CHECK
static void assertStateNotNaN(const kalmanCoreData_t* this) {
  if ((isnan(this->S[KC_STATE_X])) ||
      (isnan(this->S[KC_STATE_Y])) ||
      (isnan(this->S[KC_STATE_Z])) ||
      (isnan(this->S[KC_STATE_PX])) ||
      (isnan(this->S[KC_STATE_PY])) ||
      (isnan(this->S[KC_STATE_PZ])) ||
      (isnan(this->S[KC_STATE_D0])) ||
      (isnan(this->S[KC_STATE_D1])) ||
      (isnan(this->S[KC_STATE_D2])) ||
      (isnan(this->q[0])) ||
      (isnan(this->q[1])) ||
      (isnan(this->q[2])) ||
      (isnan(this->q[3])))
  {
    ASSERT(false);
  }

  for(int i=0; i<KC_STATE_DIM; i++) {
    for(int j=0; j<KC_STATE_DIM; j++)
    {
      if (isnan(this->P[i][j]))
      {
        ASSERT(false);
      }
    }
  }
}
#else
static void assertStateNotNaN(const kalmanCoreData_t* this)
{
  return;
}
#endif


// 协方差边界，按理不应触发，但有时会触发... 为什么？
#define MAX_COVARIANCE (100)
#define MIN_COVARIANCE (1e-6f)

// 初始方差：位置不确定，但已知静止且大致水平
static const float stdDevInitialPosition_xy = 100;
static const float stdDevInitialPosition_z = 1;
static const float stdDevInitialVelocity = 0.01;
static const float stdDevInitialAttitude_rollpitch = 0.01;
static const float stdDevInitialAttitude_yaw = 0.01;

static float procNoiseAcc_xy = 0.5f;
static float procNoiseAcc_z = 1.0f;
static float procNoiseVel = 0;
static float procNoisePos = 0;
static float procNoiseAtt = 0;
static float measNoiseBaro = 2.0f; // 单位：米
static float measNoiseGyro_rollpitch = 0.1f; // 单位：弧度/秒
static float measNoiseGyro_yaw = 0.1f; // 单位：弧度/秒

static float initialX = 0.0;
static float initialY = 0.0;
static float initialZ = 0.0;

// 初始偏航角（弧度）
// 0 --- 朝向 X 正方向
// PI / 2 --- 朝向 Y 正方向
// PI --- 朝向 X 负方向
// 3 * PI / 2 --- 朝向 Y 负方向
static float initialYaw = 0.0;

// 初始偏航使用的四元数
static float initialQuaternion[4] = {0.0, 0.0, 0.0, 0.0};

static uint32_t tdoaCount;

static OutlierFilterLhState_t sweepOutlierFilterState;


void kalmanCoreInit(kalmanCoreData_t* this) {
  tdoaCount = 0;

  // 将所有数据重置为 0（类似系统复位）
  memset(this, 0, sizeof(kalmanCoreData_t));

  this->S[KC_STATE_X] = initialX;
  this->S[KC_STATE_Y] = initialY;
  this->S[KC_STATE_Z] = initialZ;
//  this->S[KC_STATE_PX] = 0;
//  this->S[KC_STATE_PY] = 0;
//  this->S[KC_STATE_PZ] = 0;
//  this->S[KC_STATE_D0] = 0;
//  this->S[KC_STATE_D1] = 0;
//  this->S[KC_STATE_D2] = 0;

  // 重置姿态四元数
  initialQuaternion[0] = xtensa_cos_f32(initialYaw / 2);
  initialQuaternion[1] = 0.0;
  initialQuaternion[2] = 0.0;
  initialQuaternion[3] = xtensa_sin_f32(initialYaw / 2);
  for (int i = 0; i < 4; i++) { this->q[i] = initialQuaternion[i]; }

  // 然后将初始旋转矩阵设为单位矩阵。这只影响第一次预测步，
  // 因为在最终化阶段将姿态误差并入姿态状态后，旋转矩阵会更新。
  for(int i=0; i<3; i++) { for(int j=0; j<3; j++) { this->R[i][j] = i==j ? 1 : 0; }}

  for (int i=0; i< KC_STATE_DIM; i++) {
    for (int j=0; j < KC_STATE_DIM; j++) {
      this->P[i][j] = 0; // 将协方差置零（对角线会在下一部分被改写）
    }
  }

  // 初始化状态方差
  this->P[KC_STATE_X][KC_STATE_X]  = powf(stdDevInitialPosition_xy, 2);
  this->P[KC_STATE_Y][KC_STATE_Y]  = powf(stdDevInitialPosition_xy, 2);
  this->P[KC_STATE_Z][KC_STATE_Z]  = powf(stdDevInitialPosition_z, 2);

  this->P[KC_STATE_PX][KC_STATE_PX] = powf(stdDevInitialVelocity, 2);
  this->P[KC_STATE_PY][KC_STATE_PY] = powf(stdDevInitialVelocity, 2);
  this->P[KC_STATE_PZ][KC_STATE_PZ] = powf(stdDevInitialVelocity, 2);

  this->P[KC_STATE_D0][KC_STATE_D0] = powf(stdDevInitialAttitude_rollpitch, 2);
  this->P[KC_STATE_D1][KC_STATE_D1] = powf(stdDevInitialAttitude_rollpitch, 2);
  this->P[KC_STATE_D2][KC_STATE_D2] = powf(stdDevInitialAttitude_yaw, 2);

  this->Pm.numRows = KC_STATE_DIM;
  this->Pm.numCols = KC_STATE_DIM;
  this->Pm.pData = (float*)this->P;

  this->baroReferenceHeight = 0.0;

  outlierFilterReset(&sweepOutlierFilterState, 0);
}

static void scalarUpdate(kalmanCoreData_t* this, xtensa_matrix_instance_f32 *Hm, float error, float stdMeasNoise)
{
  // 卡尔曼增益列向量
  NO_DMA_CCM_SAFE_ZERO_INIT static float K[KC_STATE_DIM];
  static xtensa_matrix_instance_f32 Km = {KC_STATE_DIM, 1, (float *)K};

  // 协方差更新的临时矩阵
  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
  static xtensa_matrix_instance_f32 tmpNN1m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN1d};

  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
  static xtensa_matrix_instance_f32 tmpNN2m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN2d};

  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmpNN3d[KC_STATE_DIM * KC_STATE_DIM];
  static xtensa_matrix_instance_f32 tmpNN3m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN3d};

  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float HTd[KC_STATE_DIM * 1];
  static xtensa_matrix_instance_f32 HTm = {KC_STATE_DIM, 1, HTd};

  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float PHTd[KC_STATE_DIM * 1];
  static xtensa_matrix_instance_f32 PHTm = {KC_STATE_DIM, 1, PHTd};

  ASSERT(Hm->numRows == 1);
  ASSERT(Hm->numCols == KC_STATE_DIM);

  // ====== 创新协方差 ======

  mat_trans(Hm, &HTm);
  mat_mult(&this->Pm, &HTm, &PHTm); // PH'
  float R = stdMeasNoise*stdMeasNoise;
  float HPHR = R; // HPH' + R
  for (int i=0; i<KC_STATE_DIM; i++) { // 将 HPH' 的元素加到上式
    HPHR += Hm->pData[i]*PHTd[i]; // 仅适用于标量更新（本函数情况）
  }
  ASSERT(!isnan(HPHR));

  // ====== 测量更新 ======
  // 计算卡尔曼增益并执行状态更新
  for (int i=0; i<KC_STATE_DIM; i++) {
    K[i] = PHTd[i]/HPHR; // 卡尔曼增益 = (PH' (HPH' + R )^-1)
    this->S[i] = this->S[i] + K[i] * error; // 状态更新
  }
  assertStateNotNaN(this);

  // ====== 协方差更新 ======
  mat_mult(&Km, Hm, &tmpNN1m); // KH
  for (int i=0; i<KC_STATE_DIM; i++) { tmpNN1d[KC_STATE_DIM*i+i] -= 1; } // KH - I
  mat_trans(&tmpNN1m, &tmpNN2m); // (KH - I)'
  mat_mult(&tmpNN1m, &this->Pm, &tmpNN3m); // (KH - I)*P
  mat_mult(&tmpNN3m, &tmpNN2m, &this->Pm); // (KH - I)*P*(KH - I)'
  assertStateNotNaN(this);
  // 加入测量方差并确保有界与对称
  // TODO：为何会触及这些边界？需要进一步调查。
  for (int i=0; i<KC_STATE_DIM; i++) {
    for (int j=i; j<KC_STATE_DIM; j++) {
      float v = K[i] * R * K[j];
      float p = 0.5f*this->P[i][j] + 0.5f*this->P[j][i] + v; // 加入测量噪声
      if (isnan(p) || p > MAX_COVARIANCE) {
        this->P[i][j] = this->P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        this->P[i][j] = this->P[j][i] = MIN_COVARIANCE;
      } else {
        this->P[i][j] = this->P[j][i] = p;
      }
    }
  }

  assertStateNotNaN(this);
}


void kalmanCoreUpdateWithBaro(kalmanCoreData_t* this, float baroAsl, bool quadIsFlying)
{
  float h[KC_STATE_DIM] = {0};
  xtensa_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  h[KC_STATE_Z] = 1;

  if (!quadIsFlying || this->baroReferenceHeight < 1) {
    // TODO：也许可以将零高度作为一个状态进行跟踪。若 UWB 基站带有气压计则特别有用。
    this->baroReferenceHeight = baroAsl;
  }

  float meas = (baroAsl - this->baroReferenceHeight);
  scalarUpdate(this, &H, meas - this->S[KC_STATE_Z], measNoiseBaro);
}

void kalmanCoreUpdateWithAbsoluteHeight(kalmanCoreData_t* this, heightMeasurement_t* height) {
  float h[KC_STATE_DIM] = {0};
  xtensa_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
  h[KC_STATE_Z] = 1;
  scalarUpdate(this, &H, height->height - this->S[KC_STATE_Z], height->stdDev);
}

void kalmanCoreUpdateWithPosition(kalmanCoreData_t* this, positionMeasurement_t *xyz)
{
  // 直接测量状态 x、y、z
  // 对每个状态做标量更新，比整体更新更快
  for (int i=0; i<3; i++) {
    float h[KC_STATE_DIM] = {0};
    xtensa_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
    h[KC_STATE_X+i] = 1;
    scalarUpdate(this, &H, xyz->pos[i] - this->S[KC_STATE_X+i], xyz->stdDev);
  }
}

void kalmanCoreUpdateWithPose(kalmanCoreData_t* this, poseMeasurement_t *pose)
{
  // 直接测量状态 x、y、z 以及姿态
  // 对每个状态做标量更新，比整体更新更快
  for (int i=0; i<3; i++) {
    float h[KC_STATE_DIM] = {0};
    xtensa_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
    h[KC_STATE_X+i] = 1;
    scalarUpdate(this, &H, pose->pos[i] - this->S[KC_STATE_X+i], pose->stdDevPos);
  }

  // 计算姿态误差
  struct quat const q_ekf = mkquat(this->q[1], this->q[2], this->q[3], this->q[0]);
  struct quat const q_measured = mkquat(pose->quat.x, pose->quat.y, pose->quat.z, pose->quat.w);
  struct quat const q_residual = qqmul(qinv(q_ekf), q_measured);
  // 小角度近似，见 http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf 的公式 141
  struct vec const err_quat = vscl(2.0f / q_residual.w, quatimagpart(q_residual));

  // 对每个状态做标量更新
  {
    float h[KC_STATE_DIM] = {0};
    xtensa_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
    h[KC_STATE_D0] = 1;
    scalarUpdate(this, &H, err_quat.x, pose->stdDevQuat);
    h[KC_STATE_D0] = 0;

    h[KC_STATE_D1] = 1;
    scalarUpdate(this, &H, err_quat.y, pose->stdDevQuat);
    h[KC_STATE_D1] = 0;

    h[KC_STATE_D2] = 1;
    scalarUpdate(this, &H, err_quat.z, pose->stdDevQuat);
  }
}

void kalmanCoreUpdateWithDistance(kalmanCoreData_t* this, distanceMeasurement_t *d)
{
  // 到点 (x, y, z) 的距离测量
  float h[KC_STATE_DIM] = {0};
  xtensa_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  float dx = this->S[KC_STATE_X] - d->x;
  float dy = this->S[KC_STATE_Y] - d->y;
  float dz = this->S[KC_STATE_Z] - d->z;

  float measuredDistance = d->distance;

  float predictedDistance = xtensa_sqrt(powf(dx, 2) + powf(dy, 2) + powf(dz, 2));
  if (predictedDistance != 0.0f)
  {
    // 测量方程：z = sqrt(dx^2 + dy^2 + dz^2)，其对 X 的导数给出 h。
    h[KC_STATE_X] = dx/predictedDistance;
    h[KC_STATE_Y] = dy/predictedDistance;
    h[KC_STATE_Z] = dz/predictedDistance;
  }
  else
  {
    // 避免除以零
    h[KC_STATE_X] = 1.0f;
    h[KC_STATE_Y] = 0.0f;
    h[KC_STATE_Z] = 0.0f;
  }

  scalarUpdate(this, &H, measuredDistance-predictedDistance, d->stdDev);
}


void kalmanCoreUpdateWithTDOA(kalmanCoreData_t* this, tdoaMeasurement_t *tdoa)
{
  if (tdoaCount >= 100)
  {
    /**
     * 测量方程：
     * dR = dT + d1 - d0
     */

    float measurement = tdoa->distanceDiff;

    // 基于当前状态进行预测
    float x = this->S[KC_STATE_X];
    float y = this->S[KC_STATE_Y];
    float z = this->S[KC_STATE_Z];

    float x1 = tdoa->anchorPosition[1].x, y1 = tdoa->anchorPosition[1].y, z1 = tdoa->anchorPosition[1].z;
    float x0 = tdoa->anchorPosition[0].x, y0 = tdoa->anchorPosition[0].y, z0 = tdoa->anchorPosition[0].z;

    float dx1 = x - x1;
    float dy1 = y - y1;
    float dz1 = z - z1;

    float dy0 = y - y0;
    float dx0 = x - x0;
    float dz0 = z - z0;

    float d1 = sqrtf(powf(dx1, 2) + powf(dy1, 2) + powf(dz1, 2));
    float d0 = sqrtf(powf(dx0, 2) + powf(dy0, 2) + powf(dz0, 2));

    float predicted = d1 - d0;
    float error = measurement - predicted;

    float h[KC_STATE_DIM] = {0};
    xtensa_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

    if ((d0 != 0.0f) && (d1 != 0.0f)) {
      h[KC_STATE_X] = (dx1 / d1 - dx0 / d0);
      h[KC_STATE_Y] = (dy1 / d1 - dy0 / d0);
      h[KC_STATE_Z] = (dz1 / d1 - dz0 / d0);

      vector_t jacobian = {
        .x = h[KC_STATE_X],
        .y = h[KC_STATE_Y],
        .z = h[KC_STATE_Z],
      };

      point_t estimatedPosition = {
        .x = this->S[KC_STATE_X],
        .y = this->S[KC_STATE_Y],
        .z = this->S[KC_STATE_Z],
      };

      bool sampleIsGood = outlierFilterValidateTdoaSteps(tdoa, error, &jacobian, &estimatedPosition);
      if (sampleIsGood) {
        scalarUpdate(this, &H, error, tdoa->stdDev);
      }
    }
  }

  tdoaCount++;
}



// TODO：移除临时测试变量（用于日志）
static float predictedNX;
static float predictedNY;
static float measuredNX;
static float measuredNY;

void kalmanCoreUpdateWithFlow(kalmanCoreData_t* this, const flowMeasurement_t *flow, const Axis3f *gyro)
{
  // 通过两次标量更新将光流测量纳入 EKF

  // ~~~ 相机常量 ~~~
  // 视场角从原始寄存器数据估计，且看起来是对称的
  float Npix = 30.0;                      // 像素（x、y 相同）
  //float thetapix = DEG_TO_RAD * 4.0f;     // 单位：弧度（x、y 相同）
  float thetapix = DEG_TO_RAD * 4.2f;
  //~~~ 机体角速度 ~~~
  // TODO：检查该做法是否可行，或是否需要滤波
  float omegax_b = gyro->x * DEG_TO_RAD;
  float omegay_b = gyro->y * DEG_TO_RAD;

  // ~~~ 将机体系速度转换到全局坐标系 ~~~
  // [bar{x},bar{y},bar{z}]_G = R*[bar{x},bar{y},bar{z}]_B
  //
  // \dot{x}_G = (R^T*[dot{x}_B,dot{y}_B,dot{z}_B])\dot \hat{x}_G
  // \dot{x}_G = (R^T*[dot{x}_B,dot{y}_B,dot{z}_B])\dot \hat{x}_G
  //
  // 其中 \hat{} 表示基向量，\dot{} 表示导数，
  // _G 与 _B 分别表示全局/机体坐标系。

  // 修改 1
  //dx_g = R[0][0] * S[KC_STATE_PX] + R[0][1] * S[KC_STATE_PY] + R[0][2] * S[KC_STATE_PZ];
  //dy_g = R[1][0] * S[KC_STATE_PX] + R[1][1] * S[KC_STATE_PY] + R[1][2] * S[KC_STATE_PZ];


  float dx_g = this->S[KC_STATE_PX];
  float dy_g = this->S[KC_STATE_PY];
  float z_g = 0.0;
  // 预测与校正中限制高度，避免奇异
  if ( this->S[KC_STATE_Z] < 0.1f ) {
      z_g = 0.1;
  } else {
      z_g = this->S[KC_STATE_Z];
  }

  // ~~~ X 速度预测与更新 ~~~
  // 预测 x 方向累计像素数
  float omegaFactor = 1.25f;
  float hx[KC_STATE_DIM] = {0};
  xtensa_matrix_instance_f32 Hx = {1, KC_STATE_DIM, hx};
  predictedNX = (flow->dt * Npix / thetapix ) * ((dx_g * this->R[2][2] / z_g) - omegaFactor * omegay_b);
  measuredNX = flow->dpixelx;

  // 对 dx（以及 z）求测量方程
  hx[KC_STATE_Z] = (Npix * flow->dt / thetapix) * ((this->R[2][2] * dx_g) / (-z_g * z_g));
  hx[KC_STATE_PX] = (Npix * flow->dt / thetapix) * (this->R[2][2] / z_g);

  // 第一次更新
  scalarUpdate(this, &Hx, measuredNX-predictedNX, flow->stdDevX);

  // ~~~ Y 速度预测与更新 ~~~
  float hy[KC_STATE_DIM] = {0};
  xtensa_matrix_instance_f32 Hy = {1, KC_STATE_DIM, hy};
  predictedNY = (flow->dt * Npix / thetapix ) * ((dy_g * this->R[2][2] / z_g) + omegaFactor * omegax_b);
  measuredNY = flow->dpixely;

  // 对 dy（以及 z）求测量方程
  hy[KC_STATE_Z] = (Npix * flow->dt / thetapix) * ((this->R[2][2] * dy_g) / (-z_g * z_g));
  hy[KC_STATE_PY] = (Npix * flow->dt / thetapix) * (this->R[2][2] / z_g);

  // 第二次更新
  scalarUpdate(this, &Hy, measuredNY-predictedNY, flow->stdDevY);
}


void kalmanCoreUpdateWithTof(kalmanCoreData_t* this, tofMeasurement_t *tof)
{
  // 使用测得的 zb 方向距离更新滤波器
  float h[KC_STATE_DIM] = {0};
  xtensa_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  // 仅在测量可靠时更新滤波器（当 R[2][2] -> 0 时 \hat{h} -> 无穷）
  if (fabs(this->R[2][2]) > 0.1 && this->R[2][2] > 0){
    float angle = fabsf(acosf(this->R[2][2])) - DEG_TO_RAD * (15.0f / 2.0f);
    if (angle < 0.0f) {
      angle = 0.0f;
    }
    //float predictedDistance = S[KC_STATE_Z] / cosf(angle);
    float predictedDistance = this->S[KC_STATE_Z] / this->R[2][2];
    float measuredDistance = tof->distance; // 单位：m

    // 测量方程
    //
    // h = z/((R*z_b)\dot z_b) = z/cos(alpha)
    h[KC_STATE_Z] = 1 / this->R[2][2];
    //h[KC_STATE_Z] = 1 / cosf(angle);

    // 标量更新
    scalarUpdate(this, &H, measuredDistance-predictedDistance, tof->stdDev);
  }
}

void kalmanCoreUpdateWithYawError(kalmanCoreData_t *this, yawErrorMeasurement_t *error)
{
    float h[KC_STATE_DIM] = {0};
    xtensa_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

    h[KC_STATE_D2] = 1;
    scalarUpdate(this, &H, this->S[KC_STATE_D2] - error->yawError, error->stdDev);
}

//void kalmanCoreUpdateWithSweepAngles(kalmanCoreData_t *this, sweepAngleMeasurement_t *sweepInfo, const uint32_t tick) {
  // 使用 CF 旋转矩阵将传感器位置从 CF 参考系旋转到全局参考系
  // vec3d s;
  // xtensa_matrix_instance_f32 Rcf_ = {3, 3, (float32_t *)this->R};
  // xtensa_matrix_instance_f32 scf_ = {3, 1, (float32_t *)*sweepInfo->sensorPos};
  // xtensa_matrix_instance_f32 s_ = {3, 1, s};
  // mat_mult(&Rcf_, &scf_, &s_);

  // // 获取 Crazyflie 当前位置（全局参考系），并加上相对传感器位置
  // vec3d pcf = {this->S[KC_STATE_X] + s[0], this->S[KC_STATE_Y] + s[1], this->S[KC_STATE_Z] + s[2]};

  // // 计算转子与 CF 上传感器之间的差值（全局参考系）
  // const vec3d* pr = sweepInfo->rotorPos;
  // vec3d stmp = {pcf[0] - (*pr)[0], pcf[1] - (*pr)[1], pcf[2] - (*pr)[2]};
  // xtensa_matrix_instance_f32 stmp_ = {3, 1, stmp};

  // // 使用转子逆旋转矩阵将位置差旋转到转子参考系
  // vec3d sr;
  // xtensa_matrix_instance_f32 Rr_inv_ = {3, 3, (float32_t *)(*sweepInfo->rotorRotInv)};
  // xtensa_matrix_instance_f32 sr_ = {3, 1, sr};
  // mat_mult(&Rr_inv_, &stmp_, &sr_);

  // // 以下计算在转子参考系中进行
  // const float x = sr[0];
  // const float y = sr[1];
  // const float z = sr[2];
  // const float t = sweepInfo->t;
  // const float tan_t = tanf(t);

  // const float r2 = x * x + y * y;
  // const float r = xtensa_sqrt(r2);

  // const float predictedSweepAngle = sweepInfo->calibrationMeasurementModel(x, y, z, t, sweepInfo->calib);
  // const float measuredSweepAngle = sweepInfo->measuredSweepAngle;
  // const float error = measuredSweepAngle - predictedSweepAngle;

  // if (outlierFilterValidateLighthouseSweep(&sweepOutlierFilterState, r, error, tick)) {
  //   // 计算 H 向量（转子参考系）
  //   const float z_tan_t = z * tan_t;
  //   const float qNum = r2 - z_tan_t * z_tan_t;
  //   // 避免奇异
  //   if (qNum > 0.0001f) {
  //     const float q = tan_t / xtensa_sqrt(qNum);
  //     vec3d gr = {(-y - x * z * q) / r2, (x - y * z * q) / r2 , q};

  //     // gr 位于转子参考系中，使用转子旋转矩阵旋转回全局参考系
  //     vec3d g;
  //     xtensa_matrix_instance_f32 gr_ = {3, 1, gr};
  //     xtensa_matrix_instance_f32 Rr_ = {3, 3, (float32_t *)(*sweepInfo->rotorRot)};
  //     xtensa_matrix_instance_f32 g_ = {3, 1, g};
  //     mat_mult(&Rr_, &gr_, &g_);

  //     float h[KC_STATE_DIM] = {0};
  //     h[KC_STATE_X] = g[0];
  //     h[KC_STATE_Y] = g[1];
  //     h[KC_STATE_Z] = g[2];

  //     xtensa_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
  //     scalarUpdate(this, &H, error, sweepInfo->stdDev);
  //   }
  // }
//}

void kalmanCorePredict(kalmanCoreData_t* this, float cmdThrust, Axis3f *acc, Axis3f *gyro, float dt, bool quadIsFlying)
{
  /* Here we discretize (euler forward) and linearise the quadrocopter dynamics in order
   * to push the covariance forward.
   *
   * QUADROCOPTER DYNAMICS (see paper):
   *
   * \dot{x} = R(I + [[d]])p
   * \dot{p} = f/m * e3 - [[\omega]]p - g(I - [[d]])R^-1 e3 // 忽略阻力
   * \dot{d} = \omega
   *
   * where [[.]] is the cross-product matrix of .
   *       \omega are the gyro measurements
   *       e3 is the column vector [0 0 1]'
   *       I is the identity
   *       R is the current attitude as a rotation matrix
   *       f/m is the mass-normalized motor force (acceleration in the body's z direction)
   *       g is gravity
   *       x, p, d are the quad's states
   * note that d (attitude error) is zero at the beginning of each iteration,
   * since error information is incorporated into R after each Kalman update.
   */

  // 线性化更新矩阵
  NO_DMA_CCM_SAFE_ZERO_INIT static float A[KC_STATE_DIM][KC_STATE_DIM];
  static __attribute__((aligned(4))) xtensa_matrix_instance_f32 Am = { KC_STATE_DIM, KC_STATE_DIM, (float *)A}; // 用于协方差更新的线性化动力学

  // 协方差更新的临时矩阵
  NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
  static __attribute__((aligned(4))) xtensa_matrix_instance_f32 tmpNN1m = { KC_STATE_DIM, KC_STATE_DIM, tmpNN1d};

  NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
  static __attribute__((aligned(4))) xtensa_matrix_instance_f32 tmpNN2m = { KC_STATE_DIM, KC_STATE_DIM, tmpNN2d};

  float dt2 = dt*dt;

  // ====== 动力学线性化 ======
  // 初始化为单位矩阵
  A[KC_STATE_X][KC_STATE_X] = 1;
  A[KC_STATE_Y][KC_STATE_Y] = 1;
  A[KC_STATE_Z][KC_STATE_Z] = 1;

  A[KC_STATE_PX][KC_STATE_PX] = 1;
  A[KC_STATE_PY][KC_STATE_PY] = 1;
  A[KC_STATE_PZ][KC_STATE_PZ] = 1;

  A[KC_STATE_D0][KC_STATE_D0] = 1;
  A[KC_STATE_D1][KC_STATE_D1] = 1;
  A[KC_STATE_D2][KC_STATE_D2] = 1;

  // 由机体系速度得到位置
  A[KC_STATE_X][KC_STATE_PX] = this->R[0][0]*dt;
  A[KC_STATE_Y][KC_STATE_PX] = this->R[1][0]*dt;
  A[KC_STATE_Z][KC_STATE_PX] = this->R[2][0]*dt;

  A[KC_STATE_X][KC_STATE_PY] = this->R[0][1]*dt;
  A[KC_STATE_Y][KC_STATE_PY] = this->R[1][1]*dt;
  A[KC_STATE_Z][KC_STATE_PY] = this->R[2][1]*dt;

  A[KC_STATE_X][KC_STATE_PZ] = this->R[0][2]*dt;
  A[KC_STATE_Y][KC_STATE_PZ] = this->R[1][2]*dt;
  A[KC_STATE_Z][KC_STATE_PZ] = this->R[2][2]*dt;

  // 由姿态误差得到位置
  A[KC_STATE_X][KC_STATE_D0] = (this->S[KC_STATE_PY]*this->R[0][2] - this->S[KC_STATE_PZ]*this->R[0][1])*dt;
  A[KC_STATE_Y][KC_STATE_D0] = (this->S[KC_STATE_PY]*this->R[1][2] - this->S[KC_STATE_PZ]*this->R[1][1])*dt;
  A[KC_STATE_Z][KC_STATE_D0] = (this->S[KC_STATE_PY]*this->R[2][2] - this->S[KC_STATE_PZ]*this->R[2][1])*dt;

  A[KC_STATE_X][KC_STATE_D1] = (- this->S[KC_STATE_PX]*this->R[0][2] + this->S[KC_STATE_PZ]*this->R[0][0])*dt;
  A[KC_STATE_Y][KC_STATE_D1] = (- this->S[KC_STATE_PX]*this->R[1][2] + this->S[KC_STATE_PZ]*this->R[1][0])*dt;
  A[KC_STATE_Z][KC_STATE_D1] = (- this->S[KC_STATE_PX]*this->R[2][2] + this->S[KC_STATE_PZ]*this->R[2][0])*dt;

  A[KC_STATE_X][KC_STATE_D2] = (this->S[KC_STATE_PX]*this->R[0][1] - this->S[KC_STATE_PY]*this->R[0][0])*dt;
  A[KC_STATE_Y][KC_STATE_D2] = (this->S[KC_STATE_PX]*this->R[1][1] - this->S[KC_STATE_PY]*this->R[1][0])*dt;
  A[KC_STATE_Z][KC_STATE_D2] = (this->S[KC_STATE_PX]*this->R[2][1] - this->S[KC_STATE_PY]*this->R[2][0])*dt;

  // 由机体系速度得到机体系速度
  A[KC_STATE_PX][KC_STATE_PX] = 1; // 忽略阻力
  A[KC_STATE_PY][KC_STATE_PX] =-gyro->z*dt;
  A[KC_STATE_PZ][KC_STATE_PX] = gyro->y*dt;

  A[KC_STATE_PX][KC_STATE_PY] = gyro->z*dt;
  A[KC_STATE_PY][KC_STATE_PY] = 1; // 忽略阻力
  A[KC_STATE_PZ][KC_STATE_PY] =-gyro->x*dt;

  A[KC_STATE_PX][KC_STATE_PZ] =-gyro->y*dt;
  A[KC_STATE_PY][KC_STATE_PZ] = gyro->x*dt;
  A[KC_STATE_PZ][KC_STATE_PZ] = 1; // 忽略阻力

  // 由姿态误差得到机体系速度
  A[KC_STATE_PX][KC_STATE_D0] =  0;
  A[KC_STATE_PY][KC_STATE_D0] = -GRAVITY_MAGNITUDE*this->R[2][2]*dt;
  A[KC_STATE_PZ][KC_STATE_D0] =  GRAVITY_MAGNITUDE*this->R[2][1]*dt;

  A[KC_STATE_PX][KC_STATE_D1] =  GRAVITY_MAGNITUDE*this->R[2][2]*dt;
  A[KC_STATE_PY][KC_STATE_D1] =  0;
  A[KC_STATE_PZ][KC_STATE_D1] = -GRAVITY_MAGNITUDE*this->R[2][0]*dt;

  A[KC_STATE_PX][KC_STATE_D2] = -GRAVITY_MAGNITUDE*this->R[2][1]*dt;
  A[KC_STATE_PY][KC_STATE_D2] =  GRAVITY_MAGNITUDE*this->R[2][0]*dt;
  A[KC_STATE_PZ][KC_STATE_D2] =  0;

  // 姿态误差由姿态误差更新
  /**
   * 乍看之下，下面的数值来源并不明显，因为它们并未直接出现在动力学中。
   * 在该预测步，我们跳过先更新姿态误差再将新误差并入当前姿态的步骤
   *（这需要对姿态误差协方差做旋转）。相反，我们直接更新机体姿态，
   * 但仍需要旋转协方差，即下方所示。
   *
   * 该结果来自以下二阶近似：
   * Sigma_post = exps(-d) Sigma_pre exps(-d)'
   *            ~ (I + [[-d]] + [[-d]]^2 / 2) Sigma_pre (I + [[-d]] + [[-d]]^2 / 2)'
   * 其中 d 为用罗德里格斯参数表示的姿态误差，例如在每个预测步开始时
   * 假设 d = [0,0,0] 且采样周期内 gyro.x 恒定，则 d0 = 1/2*gyro.x*dt
   *
   * 推导见 “Covariance Correction Step for Kalman Filtering with an Attitude”
   * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
   */
  float d0 = gyro->x*dt/2;
  float d1 = gyro->y*dt/2;
  float d2 = gyro->z*dt/2;

  A[KC_STATE_D0][KC_STATE_D0] =  1 - d1*d1/2 - d2*d2/2;
  A[KC_STATE_D0][KC_STATE_D1] =  d2 + d0*d1/2;
  A[KC_STATE_D0][KC_STATE_D2] = -d1 + d0*d2/2;

  A[KC_STATE_D1][KC_STATE_D0] = -d2 + d0*d1/2;
  A[KC_STATE_D1][KC_STATE_D1] =  1 - d0*d0/2 - d2*d2/2;
  A[KC_STATE_D1][KC_STATE_D2] =  d0 + d1*d2/2;

  A[KC_STATE_D2][KC_STATE_D0] =  d1 + d0*d2/2;
  A[KC_STATE_D2][KC_STATE_D1] = -d0 + d1*d2/2;
  A[KC_STATE_D2][KC_STATE_D2] = 1 - d0*d0/2 - d1*d1/2;


  // ====== 协方差更新 ======
  mat_mult(&Am, &this->Pm, &tmpNN1m); // A P
  mat_trans(&Am, &tmpNN2m); // A'
  mat_mult(&tmpNN1m, &tmpNN2m, &this->Pm); // A P A'
  // 过程噪声在预测步返回后再加入

  // ====== 预测步骤 ======
  // 预测取决于是否在地面或飞行状态。
  // 飞行时，加速度计直接测量推力（因此在飞行中不适合用来估计机体角度）

  float dx, dy, dz;
  float tmpSPX, tmpSPY, tmpSPZ;
  float zacc;

  if (quadIsFlying) // 仅 z 方向加速度
  {
    // TODO：下面可以使用 cmdThrust/mass 或 acc->z，需要测试哪个更可靠。
    // cmdThrust 的误差来自质量标定不准以及 cmdThrust -> 推力映射不精确
    // acc->z 的误差来自测量噪声与加速度计标定
    // float zacc = cmdThrust;
    zacc = acc->z;

    // 在机体系更新位置（随后旋转到惯性系）
    dx = this->S[KC_STATE_PX] * dt;
    dy = this->S[KC_STATE_PY] * dt;
    dz = this->S[KC_STATE_PZ] * dt + zacc * dt2 / 2.0f; // 推力只能沿机体 Z 方向产生

    // 位置更新
    this->S[KC_STATE_X] += this->R[0][0] * dx + this->R[0][1] * dy + this->R[0][2] * dz;
    this->S[KC_STATE_Y] += this->R[1][0] * dx + this->R[1][1] * dy + this->R[1][2] * dz;
    this->S[KC_STATE_Z] += this->R[2][0] * dx + this->R[2][1] * dy + this->R[2][2] * dz - GRAVITY_MAGNITUDE * dt2 / 2.0f;

    // 保留上一时刻状态用于更新
    tmpSPX = this->S[KC_STATE_PX];
    tmpSPY = this->S[KC_STATE_PY];
    tmpSPZ = this->S[KC_STATE_PZ];

    // 机体速度更新：加速度计 - 陀螺交叉项 - 机体系重力
    this->S[KC_STATE_PX] += dt * (gyro->z * tmpSPY - gyro->y * tmpSPZ - GRAVITY_MAGNITUDE * this->R[2][0]);
    this->S[KC_STATE_PY] += dt * (-gyro->z * tmpSPX + gyro->x * tmpSPZ - GRAVITY_MAGNITUDE * this->R[2][1]);
    this->S[KC_STATE_PZ] += dt * (zacc + gyro->y * tmpSPX - gyro->x * tmpSPY - GRAVITY_MAGNITUDE * this->R[2][2]);
  }
  else // 加速度可为任意方向（由加速度计测得），例如自由落体或被携带时。
  {
    // 在机体系更新位置（随后旋转到惯性系）
    dx = this->S[KC_STATE_PX] * dt + acc->x * dt2 / 2.0f;
    dy = this->S[KC_STATE_PY] * dt + acc->y * dt2 / 2.0f;
    dz = this->S[KC_STATE_PZ] * dt + acc->z * dt2 / 2.0f; // 推力只能沿机体 Z 方向产生

    // 位置更新
    this->S[KC_STATE_X] += this->R[0][0] * dx + this->R[0][1] * dy + this->R[0][2] * dz;
    this->S[KC_STATE_Y] += this->R[1][0] * dx + this->R[1][1] * dy + this->R[1][2] * dz;
    this->S[KC_STATE_Z] += this->R[2][0] * dx + this->R[2][1] * dy + this->R[2][2] * dz - GRAVITY_MAGNITUDE * dt2 / 2.0f;

    // 保留上一时刻状态用于更新
    tmpSPX = this->S[KC_STATE_PX];
    tmpSPY = this->S[KC_STATE_PY];
    tmpSPZ = this->S[KC_STATE_PZ];

    // 机体速度更新：加速度计 - 陀螺交叉项 - 机体系重力
    this->S[KC_STATE_PX] += dt * (acc->x + gyro->z * tmpSPY - gyro->y * tmpSPZ - GRAVITY_MAGNITUDE * this->R[2][0]);
    this->S[KC_STATE_PY] += dt * (acc->y - gyro->z * tmpSPX + gyro->x * tmpSPZ - GRAVITY_MAGNITUDE * this->R[2][1]);
    this->S[KC_STATE_PZ] += dt * (acc->z + gyro->y * tmpSPX - gyro->x * tmpSPY - GRAVITY_MAGNITUDE * this->R[2][2]);
  }

  // 姿态更新（由陀螺积分得到旋转），使用四元数
  // 这是对陀螺角速度在采样周期内的积分
  float dtwx = dt*gyro->x;
  float dtwy = dt*gyro->y;
  float dtwz = dt*gyro->z;

  // 按 [w,x,y,z] 顺序计算四元数
  float angle = xtensa_sqrt(dtwx*dtwx + dtwy*dtwy + dtwz*dtwz);
  float ca = xtensa_cos_f32(angle/2.0f);
  float sa = xtensa_sin_f32(angle/2.0f);
  float dq[4] = {ca , sa*dtwx/angle , sa*dtwy/angle , sa*dtwz/angle};

  float tmpq0;
  float tmpq1;
  float tmpq2;
  float tmpq3;

  // 使用上面计算的增量四元数旋转机体姿态
  tmpq0 = dq[0]*this->q[0] - dq[1]*this->q[1] - dq[2]*this->q[2] - dq[3]*this->q[3];
  tmpq1 = dq[1]*this->q[0] + dq[0]*this->q[1] + dq[3]*this->q[2] - dq[2]*this->q[3];
  tmpq2 = dq[2]*this->q[0] - dq[3]*this->q[1] + dq[0]*this->q[2] + dq[1]*this->q[3];
  tmpq3 = dq[3]*this->q[0] + dq[2]*this->q[1] - dq[1]*this->q[2] + dq[0]*this->q[3];

  if (! quadIsFlying) {
    float keep = 1.0f - ROLLPITCH_ZERO_REVERSION;

    tmpq0 = keep * tmpq0 + ROLLPITCH_ZERO_REVERSION * initialQuaternion[0];
    tmpq1 = keep * tmpq1 + ROLLPITCH_ZERO_REVERSION * initialQuaternion[1];
    tmpq2 = keep * tmpq2 + ROLLPITCH_ZERO_REVERSION * initialQuaternion[2];
    tmpq3 = keep * tmpq3 + ROLLPITCH_ZERO_REVERSION * initialQuaternion[3];
  }

  // 归一化并保存结果
  float norm = xtensa_sqrt(tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3);
  this->q[0] = tmpq0/norm; this->q[1] = tmpq1/norm; this->q[2] = tmpq2/norm; this->q[3] = tmpq3/norm;
  assertStateNotNaN(this);
}


void kalmanCoreAddProcessNoise(kalmanCoreData_t* this, float dt)
{
  if (dt>0)
  {
    this->P[KC_STATE_X][KC_STATE_X] += powf(procNoiseAcc_xy*dt*dt + procNoiseVel*dt + procNoisePos, 2);  // 位置过程噪声
    this->P[KC_STATE_Y][KC_STATE_Y] += powf(procNoiseAcc_xy*dt*dt + procNoiseVel*dt + procNoisePos, 2);  // 位置过程噪声
    this->P[KC_STATE_Z][KC_STATE_Z] += powf(procNoiseAcc_z*dt*dt + procNoiseVel*dt + procNoisePos, 2);  // 位置过程噪声

    this->P[KC_STATE_PX][KC_STATE_PX] += powf(procNoiseAcc_xy*dt + procNoiseVel, 2); // 速度过程噪声
    this->P[KC_STATE_PY][KC_STATE_PY] += powf(procNoiseAcc_xy*dt + procNoiseVel, 2); // 速度过程噪声
    this->P[KC_STATE_PZ][KC_STATE_PZ] += powf(procNoiseAcc_z*dt + procNoiseVel, 2); // 速度过程噪声

    this->P[KC_STATE_D0][KC_STATE_D0] += powf(measNoiseGyro_rollpitch * dt + procNoiseAtt, 2);
    this->P[KC_STATE_D1][KC_STATE_D1] += powf(measNoiseGyro_rollpitch * dt + procNoiseAtt, 2);
    this->P[KC_STATE_D2][KC_STATE_D2] += powf(measNoiseGyro_yaw * dt + procNoiseAtt, 2);
  }

  for (int i=0; i<KC_STATE_DIM; i++) {
    for (int j=i; j<KC_STATE_DIM; j++) {
      float p = 0.5f*this->P[i][j] + 0.5f*this->P[j][i];
      if (isnan(p) || p > MAX_COVARIANCE) {
        this->P[i][j] = this->P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        this->P[i][j] = this->P[j][i] = MIN_COVARIANCE;
      } else {
        this->P[i][j] = this->P[j][i] = p;
      }
    }
  }

  assertStateNotNaN(this);
}



void kalmanCoreFinalize(kalmanCoreData_t* this, uint32_t tick)
{
  // 姿态协方差更新后的旋转矩阵
  NO_DMA_CCM_SAFE_ZERO_INIT static float A[KC_STATE_DIM][KC_STATE_DIM];
  static xtensa_matrix_instance_f32 Am = {KC_STATE_DIM, KC_STATE_DIM, (float *)A};

  // 协方差更新的临时矩阵
  NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
  static xtensa_matrix_instance_f32 tmpNN1m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN1d};

  NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
  static xtensa_matrix_instance_f32 tmpNN2m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN2d};

  // 将姿态误差（卡尔曼滤波状态）并入姿态
  float v0 = this->S[KC_STATE_D0];
  float v1 = this->S[KC_STATE_D1];
  float v2 = this->S[KC_STATE_D2];

  // 当角度误差足够大时，将姿态误差并入姿态
  if ((fabsf(v0) > 0.1e-3f || fabsf(v1) > 0.1e-3f || fabsf(v2) > 0.1e-3f) && (fabsf(v0) < 10 && fabsf(v1) < 10 && fabsf(v2) < 10))
  {
    float angle = xtensa_sqrt(v0*v0 + v1*v1 + v2*v2);
    float ca = xtensa_cos_f32(angle / 2.0f);
    float sa = xtensa_sin_f32(angle / 2.0f);
    float dq[4] = {ca, sa * v0 / angle, sa * v1 / angle, sa * v2 / angle};

    // 使用上面计算的增量四元数旋转机体姿态
    float tmpq0 = dq[0] * this->q[0] - dq[1] * this->q[1] - dq[2] * this->q[2] - dq[3] * this->q[3];
    float tmpq1 = dq[1] * this->q[0] + dq[0] * this->q[1] + dq[3] * this->q[2] - dq[2] * this->q[3];
    float tmpq2 = dq[2] * this->q[0] - dq[3] * this->q[1] + dq[0] * this->q[2] + dq[1] * this->q[3];
    float tmpq3 = dq[3] * this->q[0] + dq[2] * this->q[1] - dq[1] * this->q[2] + dq[0] * this->q[3];

    // 归一化并保存结果
    float norm = xtensa_sqrt(tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + tmpq3 * tmpq3);
    this->q[0] = tmpq0 / norm;
    this->q[1] = tmpq1 / norm;
    this->q[2] = tmpq2 / norm;
    this->q[3] = tmpq3 / norm;

    /** 由于机体姿态已旋转，因此需要旋转协方差
     *
     * 该结果来自以下二阶近似：
     * Sigma_post = exps(-d) Sigma_pre exps(-d)'
     *            ~ (I + [[-d]] + [[-d]]^2 / 2) Sigma_pre (I + [[-d]] + [[-d]]^2 / 2)'
     * 其中 d 为用罗德里格斯参数表示的姿态误差，即 d = tan(|v|/2)*v/|v|
     *
     * 推导见 “Covariance Correction Step for Kalman Filtering with an Attitude”
     * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
     */

    float d0 = v0/2; // 姿态误差向量 (v0,v1,v2) 很小，
    float d1 = v1/2; // 因此使用一阶近似 d0 = tan(|v0|/2)*v0/|v0|
    float d2 = v2/2;

    A[KC_STATE_X][KC_STATE_X] = 1;
    A[KC_STATE_Y][KC_STATE_Y] = 1;
    A[KC_STATE_Z][KC_STATE_Z] = 1;

    A[KC_STATE_PX][KC_STATE_PX] = 1;
    A[KC_STATE_PY][KC_STATE_PY] = 1;
    A[KC_STATE_PZ][KC_STATE_PZ] = 1;

    A[KC_STATE_D0][KC_STATE_D0] =  1 - d1*d1/2 - d2*d2/2;
    A[KC_STATE_D0][KC_STATE_D1] =  d2 + d0*d1/2;
    A[KC_STATE_D0][KC_STATE_D2] = -d1 + d0*d2/2;

    A[KC_STATE_D1][KC_STATE_D0] = -d2 + d0*d1/2;
    A[KC_STATE_D1][KC_STATE_D1] =  1 - d0*d0/2 - d2*d2/2;
    A[KC_STATE_D1][KC_STATE_D2] =  d0 + d1*d2/2;

    A[KC_STATE_D2][KC_STATE_D0] =  d1 + d0*d2/2;
    A[KC_STATE_D2][KC_STATE_D1] = -d0 + d1*d2/2;
    A[KC_STATE_D2][KC_STATE_D2] = 1 - d0*d0/2 - d1*d1/2;

    mat_trans(&Am, &tmpNN1m); // A'
    mat_mult(&Am, &this->Pm, &tmpNN2m); // AP
    mat_mult(&tmpNN2m, &tmpNN1m, &this->Pm); //APA'
  }

  // 将新的姿态转换为旋转矩阵，以便旋转机体系速度与加速度
  this->R[0][0] = this->q[0] * this->q[0] + this->q[1] * this->q[1] - this->q[2] * this->q[2] - this->q[3] * this->q[3];
  this->R[0][1] = 2 * this->q[1] * this->q[2] - 2 * this->q[0] * this->q[3];
  this->R[0][2] = 2 * this->q[1] * this->q[3] + 2 * this->q[0] * this->q[2];

  this->R[1][0] = 2 * this->q[1] * this->q[2] + 2 * this->q[0] * this->q[3];
  this->R[1][1] = this->q[0] * this->q[0] - this->q[1] * this->q[1] + this->q[2] * this->q[2] - this->q[3] * this->q[3];
  this->R[1][2] = 2 * this->q[2] * this->q[3] - 2 * this->q[0] * this->q[1];

  this->R[2][0] = 2 * this->q[1] * this->q[3] - 2 * this->q[0] * this->q[2];
  this->R[2][1] = 2 * this->q[2] * this->q[3] + 2 * this->q[0] * this->q[1];
  this->R[2][2] = this->q[0] * this->q[0] - this->q[1] * this->q[1] - this->q[2] * this->q[2] + this->q[3] * this->q[3];

  // 重置姿态误差
  this->S[KC_STATE_D0] = 0;
  this->S[KC_STATE_D1] = 0;
  this->S[KC_STATE_D2] = 0;

  // 保持协方差矩阵对称，并确保数值有界
  for (int i=0; i<KC_STATE_DIM; i++) {
    for (int j=i; j<KC_STATE_DIM; j++) {
      float p = 0.5f*this->P[i][j] + 0.5f*this->P[j][i];
      if (isnan(p) || p > MAX_COVARIANCE) {
        this->P[i][j] = this->P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        this->P[i][j] = this->P[j][i] = MIN_COVARIANCE;
      } else {
        this->P[i][j] = this->P[j][i] = p;
      }
    }
  }

  assertStateNotNaN(this);
}

void kalmanCoreExternalizeState(const kalmanCoreData_t* this, state_t *state, const Axis3f *acc, uint32_t tick)
{
  // 位置状态已在世界坐标系
  state->position = (point_t){
      .timestamp = tick,
      .x = this->S[KC_STATE_X],
      .y = this->S[KC_STATE_Y],
      .z = this->S[KC_STATE_Z]
  };

  // 速度在机体系中，需要旋转到世界坐标系
  state->velocity = (velocity_t){
      .timestamp = tick,
      .x = this->R[0][0]*this->S[KC_STATE_PX] + this->R[0][1]*this->S[KC_STATE_PY] + this->R[0][2]*this->S[KC_STATE_PZ],
      .y = this->R[1][0]*this->S[KC_STATE_PX] + this->R[1][1]*this->S[KC_STATE_PY] + this->R[1][2]*this->S[KC_STATE_PZ],
      .z = this->R[2][0]*this->S[KC_STATE_PX] + this->R[2][1]*this->S[KC_STATE_PY] + this->R[2][2]*this->S[KC_STATE_PZ]
  };

  // 加速度计测量在机体系中，需要旋转到世界坐标系。
  // 此外，历史代码要求 acc.z 为去除重力后的加速度。
  // 注意这些加速度单位为 G，而非 m/s^2，因此需要减去 1 去除重力
  state->acc = (acc_t){
      .timestamp = tick,
      .x = this->R[0][0]*acc->x + this->R[0][1]*acc->y + this->R[0][2]*acc->z,
      .y = this->R[1][0]*acc->x + this->R[1][1]*acc->y + this->R[1][2]*acc->z,
      .z = this->R[2][0]*acc->x + this->R[2][1]*acc->y + this->R[2][2]*acc->z - 1
  };

  // 将新姿态转换为欧拉角 YPR
  float yaw = atan2f(2*(this->q[1]*this->q[2]+this->q[0]*this->q[3]) , this->q[0]*this->q[0] + this->q[1]*this->q[1] - this->q[2]*this->q[2] - this->q[3]*this->q[3]);
  float pitch = asinf(-2*(this->q[1]*this->q[3] - this->q[0]*this->q[2]));
  float roll = atan2f(2*(this->q[2]*this->q[3]+this->q[0]*this->q[1]) , this->q[0]*this->q[0] - this->q[1]*this->q[1] - this->q[2]*this->q[2] + this->q[3]*this->q[3]);

  // 保存姿态，并按旧版 CF2 机体坐标系调整
  state->attitude = (attitude_t){
      .timestamp = tick,
      .roll = roll*RAD_TO_DEG,
      .pitch = -pitch*RAD_TO_DEG,
      .yaw = yaw*RAD_TO_DEG
  };

  // 保存四元数，希望未来可用于更好的控制器。
  // 注意：此处未按旧版坐标系调整
  state->attitudeQuaternion = (quaternion_t){
      .timestamp = tick,
      .w = this->q[0],
      .x = this->q[1],
      .y = this->q[2],
      .z = this->q[3]
  };

  assertStateNotNaN(this);
}

// 将某个状态重置为 0 并设置最大协方差
// 若频繁调用，会使该状态与滤波器其他部分解耦
static void decoupleState(kalmanCoreData_t* this, kalmanCoreStateIdx_t state)
{
  // 将所有协方差置 0
  for(int i=0; i<KC_STATE_DIM; i++) {
    this->P[state][i] = 0;
    this->P[i][state] = 0;
  }
  // 将状态方差设为最大
  this->P[state][state] = MAX_COVARIANCE;
  // 将状态置零
  this->S[state] = 0;
}

void kalmanCoreDecoupleXY(kalmanCoreData_t* this)
{
  decoupleState(this, KC_STATE_X);
  decoupleState(this, KC_STATE_PX);
  decoupleState(this, KC_STATE_Y);
  decoupleState(this, KC_STATE_PY);
}

// 默认日志组
LOG_GROUP_START(kalman_pred)
  LOG_ADD(LOG_FLOAT, predNX, &predictedNX)
  LOG_ADD(LOG_FLOAT, predNY, &predictedNY)
  LOG_ADD(LOG_FLOAT, measNX, &measuredNX)
  LOG_ADD(LOG_FLOAT, measNY, &measuredNY)
LOG_GROUP_STOP(kalman_pred)

LOG_GROUP_START(outlierf)
  LOG_ADD(LOG_INT32, lhWin, &sweepOutlierFilterState.openingWindow)
LOG_GROUP_STOP(outlierf)

PARAM_GROUP_START(kalman)
  PARAM_ADD(PARAM_FLOAT, pNAcc_xy, &procNoiseAcc_xy)
  PARAM_ADD(PARAM_FLOAT, pNAcc_z, &procNoiseAcc_z)
  PARAM_ADD(PARAM_FLOAT, pNVel, &procNoiseVel)
  PARAM_ADD(PARAM_FLOAT, pNPos, &procNoisePos)
  PARAM_ADD(PARAM_FLOAT, pNAtt, &procNoiseAtt)
  PARAM_ADD(PARAM_FLOAT, mNBaro, &measNoiseBaro)
  PARAM_ADD(PARAM_FLOAT, mNGyro_rollpitch, &measNoiseGyro_rollpitch)
  PARAM_ADD(PARAM_FLOAT, mNGyro_yaw, &measNoiseGyro_yaw)
  PARAM_ADD(PARAM_FLOAT, initialX, &initialX)
  PARAM_ADD(PARAM_FLOAT, initialY, &initialY)
  PARAM_ADD(PARAM_FLOAT, initialZ, &initialZ)
  PARAM_ADD(PARAM_FLOAT, initialYaw, &initialYaw)
PARAM_GROUP_STOP(kalman)
