#pragma once

/*
The MIT License (MIT)

cmath3d
Copyright (c) 2016-2018 James Alan Preiss

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef CMATH3D_ASSERTS
#include <assert.h>
#endif

#ifndef M_PI_F
#define M_PI_F   (3.14159265358979323846f)
#define M_1_PI_F (0.31830988618379067154f)
#define M_PI_2_F (1.57079632679f)
#endif


// ----------------------------- 标量 --------------------------------

static inline float fsqr(float x) { return x * x; }
static inline float radians(float degrees) { return (M_PI_F / 180.0f) * degrees; }
static inline float degrees(float radians) { return (180.0f / M_PI_F) * radians; }
static inline float clamp(float value, float min, float max) {
  if (value < min) return min;
  if (value > max) return max;
  return value;
}
// 使用“相邻浮点数具有相邻位表示”的特性来近似比较两个浮点数相等性。
// 参数 `ulps` 表示允许的步数。该方法对接近零的数效果不佳。
// 参见 https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
static inline bool fcloseulps(float a, float b, int ulps) {
	if ((a < 0.0f) != (b < 0.0f)) {
		// 处理负零。
		if (a == b) {
			return true;
		}
		return false;
	}
	int ia = *((int *)&a);
	int ib = *((int *)&b);
	return fabsf(ia - ib) <= ulps;
}


// ---------------------------- 3D 向量 ------------------------------

struct vec {
	float x; float y; float z;
};

//
// 构造函数
//

// 由 3 个浮点数构造向量。
static inline struct vec mkvec(float x, float y, float z) {
	struct vec v;
	v.x = x; v.y = y; v.z = z;
	return v;
}
// 构造向量：x、y、z 都为同一数值。
static inline struct vec vrepeat(float x) {
	return mkvec(x, x, x);
}
// 构造零向量。
static inline struct vec vzero(void) {
	return vrepeat(0.0f);
}
// 构造第 i 个基向量，例如 vbasis(0) == (1, 0, 0)。
static inline struct vec vbasis(int i) {
	float a[3] = {0.0f, 0.0f, 0.0f};
	a[i] = 1.0f;
	return mkvec(a[0], a[1], a[2]);
}

//
// 运算符
//

// 向量与标量相乘。
static inline struct vec vscl(float s, struct vec v) {
	return mkvec(s * v.x , s * v.y, s * v.z);
}
// 向量取负。
static inline struct vec vneg(struct vec v) {
	return mkvec(-v.x, -v.y, -v.z);
}
// 向量与标量相除。
// 不进行除零检查。
static inline struct vec vdiv(struct vec v, float s) {
	return vscl(1.0f/s, v);
}
// 两个向量相加。
static inline struct vec vadd(struct vec a, struct vec b) {
	return mkvec(a.x + b.x, a.y + b.y, a.z + b.z);
}
// 向量相减。
static inline struct vec vsub(struct vec a, struct vec b) {
	return vadd(a, vneg(b));
}
// 向量点积。
static inline float vdot(struct vec a, struct vec b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}
// 向量逐元素相乘。
static inline struct vec veltmul(struct vec a, struct vec b) {
	return mkvec(a.x * b.x, a.y * b.y, a.z * b.z);
}
// 向量逐元素相除。
static inline struct vec veltdiv(struct vec a, struct vec b) {
	return mkvec(a.x / b.x, a.y / b.y, a.z / b.z);
}
// 向量逐元素求倒数。
static inline struct vec veltrecip(struct vec a) {
	return mkvec(1.0f / a.x, 1.0f / a.y, 1.0f / a.z);
}
// 向量模长平方。
static inline float vmag2(struct vec v) {
	return vdot(v, v);
}
// 向量模长。
static inline float vmag(struct vec v) {
	return sqrtf(vmag2(v));
}
// 向量欧氏距离平方。
static inline float vdist2(struct vec a, struct vec b) {
  return vmag2(vsub(a, b));
}
// 向量欧氏距离。
static inline float vdist(struct vec a, struct vec b) {
  return sqrtf(vdist2(a, b));
}
// 向量归一化（单位向量）。
static inline struct vec vnormalize(struct vec v) {
	return vdiv(v, vmag(v));
}
// 限制向量欧氏范数的最大值。
static inline struct vec vclampnorm(struct vec v, float maxnorm) {
	float const norm = vmag(v);
	if (norm > maxnorm) {
		return vscl(maxnorm / norm, v);
	}
	return v;
}
// 向量叉积。
static inline struct vec vcross(struct vec a, struct vec b) {
	return mkvec(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}
// 将 a 投影到 b 上（b 为单位向量）。
static inline struct vec vprojectunit(struct vec a, struct vec b_unit) {
	return vscl(vdot(a, b_unit), b_unit);
}
// a 在 b 的正交分量（b 为单位向量）。
static inline struct vec vorthunit(struct vec a, struct vec b_unit) {
	return vsub(a, vprojectunit(a, b_unit));
}
// 向量逐元素取绝对值。
static inline struct vec vabs(struct vec v) {
	return mkvec(fabsf(v.x), fabsf(v.y), fabsf(v.z));
}
// 向量逐元素取最小值。
static inline struct vec vmin(struct vec a, struct vec b) {
	return mkvec(fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z));
}
// 向量逐元素取最大值。
static inline struct vec vmax(struct vec a, struct vec b) {
	return mkvec(fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z));
}
// 向量逐元素限幅。
static inline struct vec vclamp(struct vec v, struct vec lower, struct vec upper) {
	return vmin(upper, vmax(v, lower));
}
// 向量逐元素按零为中心的范围限幅。
static inline struct vec vclampabs(struct vec v, struct vec abs_upper) {
	return vclamp(v, vneg(abs_upper), abs_upper);
}
// 向量最大元素。
static inline float vmaxelt(struct vec v) {
	return fmax(fmax(v.x, v.y), v.z);
}
// 向量最小元素（最负）。
static inline float vminelt(struct vec v) {
	return fmin(fmin(v.x, v.y), v.z);
}
// 向量的 L1 范数（又称 Minkowski/出租车/曼哈顿范数）。
static inline float vnorm1(struct vec v) {
	return fabsf(v.x) + fabsf(v.y) + fabsf(v.z);
}

//
// 比较（含偏序）
//

// 比较两个向量是否完全相等。
static inline bool veq(struct vec a, struct vec b) {
	return (a.x == b.x) && (a.y == b.y) && (a.z == b.z);
}
// 比较两个向量是否完全不相等。
static inline bool vneq(struct vec a, struct vec b) {
	return !veq(a, b);
}
// 使用用户定义阈值比较两个向量的近似相等。
static inline bool veqepsilon(struct vec a, struct vec b, float epsilon) {
	struct vec diffs = vabs(vsub(a, b));
	return diffs.x < epsilon && diffs.y < epsilon && diffs.z < epsilon;
}
// 全部元素小于（逐元素）
static inline bool vless(struct vec a, struct vec b) {
	return (a.x < b.x) && (a.y < b.y) && (a.z < b.z);
}
// 全部元素小于等于（逐元素）
static inline bool vleq(struct vec a, struct vec b) {
	return (a.x <= b.x) && (a.y <= b.y) && (a.z <= b.z);
}
// 全部元素大于（逐元素）
static inline bool vgreater(struct vec a, struct vec b) {
	return (a.x > b.x) && (a.y > b.y) && (a.z > b.z);
}
// 全部元素大于等于（逐元素）
static inline bool vgeq(struct vec a, struct vec b) {
	return (a.x >= b.x) && (a.y >= b.y) && (a.z >= b.z);
}
// 检测向量是否包含 NaN 元素。
static inline bool visnan(struct vec v) {
	return isnan(v.x) || isnan(v.y) || isnan(v.z);
}

//
// 简化 C 语言向量运算的辅助函数。
//

// 三个向量相加。
static inline struct vec vadd3(struct vec a, struct vec b, struct vec c) {
	return vadd(vadd(a, b), c);
}
// 四个向量相加。
static inline struct vec vadd4(struct vec a, struct vec b, struct vec c, struct vec d) {
	// TODO：确认编译后为最优代码
	return vadd(vadd(a, b), vadd(c, d));
}
// a 减去 b 和 c。
static inline struct vec vsub2(struct vec a, struct vec b, struct vec c) {
	return vadd3(a, vneg(b), vneg(c));
}

//
// 与原始 float/double 数组互转；提供数组式访问。
//

// 从 double 数组加载向量。
static inline struct vec vload(double const *d) {
	return mkvec(d[0], d[1], d[2]);
}
// 将向量存入 double 数组。
static inline void vstore(struct vec v, double *d) {
	d[0] = (double)v.x; d[1] = (double)v.y; d[2] = (double)v.z;
}
// 从 float 数组加载向量。
static inline struct vec vloadf(float const *f) {
	return mkvec(f[0], f[1], f[2]);
}
// 将向量存入 float 数组。
static inline void vstoref(struct vec v, float *f) {
	f[0] = v.x; f[1] = v.y; f[2] = v.z;
}
// 按 3 元素数组方式索引向量。
static inline float vindex(struct vec v, int i) {
	return ((float const *)&v.x)[i];
}


// ---------------------------- 3x3 矩阵 ------------------------------

struct mat33 {
	float m[3][3];
};

//
// 构造函数
//

// 构造零矩阵。
static inline struct mat33 mzero(void) {
	struct mat33 m;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			m.m[i][j] = 0;
		}
	}
	return m;
}
// 构造指定对角线的矩阵。
static inline struct mat33 mdiag(float a, float b, float c) {
	struct mat33 m = mzero();
	m.m[0][0] = a;
	m.m[1][1] = b;
	m.m[2][2] = c;
	return m;
}
// 构造标量 a 的倍数单位矩阵 a*I。
static inline struct mat33 meyescl(float a) {
	return mdiag(a, a, a);
}
// 构造单位矩阵。
static inline struct mat33 meye(void) {
	return meyescl(1.0f);
}
// 由三个列向量构造矩阵。
static inline struct mat33 mcolumns(struct vec a, struct vec b, struct vec c) {
	struct mat33 m;
	m.m[0][0] = a.x;
	m.m[1][0] = a.y;
	m.m[2][0] = a.z;

	m.m[0][1] = b.x;
	m.m[1][1] = b.y;
	m.m[2][1] = b.z;

	m.m[0][2] = c.x;
	m.m[1][2] = c.y;
	m.m[2][2] = c.z;

	return m;
}
// 由三个行向量构造矩阵。
static inline struct mat33 mrows(struct vec a, struct vec b, struct vec c) {
	struct mat33 m;
	vstoref(a, m.m[0]);
	vstoref(b, m.m[1]);
	vstoref(c, m.m[2]);
	return m;
}
// 由向量 v 构造矩阵 A，使得 Ax = cross(v, x)。
static inline struct mat33 mcrossmat(struct vec v) {
	struct mat33 m;
	m.m[0][0] = 0;
	m.m[0][1] = -v.z;
	m.m[0][2] = v.y;
	m.m[1][0] = v.z;
	m.m[1][1] = 0;
	m.m[1][2] = -v.x;
	m.m[2][0] = -v.y;
	m.m[2][1] = v.x;
	m.m[2][2] = 0;
	return m;
}

//
// 访问器
//

// 以向量形式返回矩阵的一列。
static inline struct vec mcolumn(struct mat33 m, int col) {
	return mkvec(m.m[0][col], m.m[1][col], m.m[2][col]);
}
// 以向量形式返回矩阵的一行。
static inline struct vec mrow(struct mat33 m, int row) {
	return vloadf(m.m[row]);
}

//
// 运算符
//

// 矩阵转置。
static inline struct mat33 mtranspose(struct mat33 m) {
	struct mat33 mt;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			mt.m[i][j] = m.m[j][i];
		}
	}
	return mt;
}
// 矩阵与标量相乘。
static inline struct mat33 mscl(float s, struct mat33 a) {
	struct mat33 sa;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			sa.m[i][j] = s * a.m[i][j];
		}
	}
	return sa;
}
// 矩阵取负。
static inline struct mat33 mneg(struct mat33 a) {
	return mscl(-1.0, a);
}
// 矩阵相加。
static inline struct mat33 madd(struct mat33 a, struct mat33 b) {
	struct mat33 c;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			c.m[i][j] = a.m[i][j] + b.m[i][j];
		}
	}
	return c;
}
// 矩阵相减。
static inline struct mat33 msub(struct mat33 a, struct mat33 b) {
	struct mat33 c;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			c.m[i][j] = a.m[i][j] - b.m[i][j];
		}
	}
	return c;
}
// 矩阵乘向量。
static inline struct vec mvmul(struct mat33 a, struct vec v) {
	float x = a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z;
	float y = a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z;
	float z = a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z;
	return mkvec(x, y, z);
}
// 矩阵相乘。
static inline struct mat33 mmul(struct mat33 a, struct mat33 b) {
	struct mat33 ab;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			float accum = 0;
			for (int k = 0; k < 3; ++k) {
				accum += a.m[i][k] * b.m[k][j];
			}
			ab.m[i][j] = accum;
		}
	}
	return ab;
}
// 对角线加上标量，即 a + dI。
static inline struct mat33 maddridge(struct mat33 a, float d) {
	a.m[0][0] += d;
	a.m[1][1] += d;
	a.m[2][2] += d;
	return a;
}
// 检测矩阵是否包含 NaN 元素。
static inline bool misnan(struct mat33 m) {
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			if (isnan(m.m[i][j])) {
				return true;
			}
		}
	}
	return false;
}

// 在大型行优先矩阵内设置一个 3x3 块。
// block: 指向大矩阵中该块左上角元素的指针。
// stride: 大矩阵的列数。
static inline void set_block33_rowmaj(float *block, int stride, struct mat33 const *m) {
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			block[j] = m->m[i][j];
		}
		block += stride;
	}
}

//
// 简化 C 语言矩阵运算的辅助函数。
//

// 三个矩阵相加。
static inline struct mat33 madd3(struct mat33 a, struct mat33 b, struct mat33 c) {
	return madd(madd(a, b), c);
}

//
// 3D 旋转构造与运算
//

// 由轴-角旋转构造等效旋转矩阵。
// 假设输入轴已归一化，角度单位为弧度。
static inline struct mat33 maxisangle(struct vec axis, float angle) {
	// Rodrigues 公式
	struct mat33 const K = mcrossmat(axis);
	return madd3(
		meye(),
		mscl(sinf(angle), K),
		mscl(1.0f - cosf(angle), mmul(K, K))
	);
}
// 绕 x 轴旋转给定角度（弧度）
static inline struct mat33 mrotx(float angle) {
	return maxisangle(mkvec(1.0f, 0.0f, 0.0f), angle);
}
// 绕 y 轴旋转给定角度（弧度）
static inline struct mat33 mroty(float angle) {
	return maxisangle(mkvec(0.0f, 1.0f, 0.0f), angle);
}
// 绕 z 轴旋转给定角度（弧度）
static inline struct mat33 mrotz(float angle) {
	return maxisangle(mkvec(0.0f, 0.0f, 1.0f), angle);
}
// TODO：手写实现可能更快，
// 但当前实现正确且三角函数可能才是主要开销


// Matrix TODO：逆矩阵、求解、特征值、9 浮点构造、轴对齐旋转


// ---------------------------- 四元数 ------------------------------

struct quat {
	float x;
	float y;
	float z;
	float w;
};

//
// 构造函数
//

// 由 x、y、z、w 分量构造四元数。
static inline struct quat mkquat(float x, float y, float z, float w) {
	struct quat q;
	q.x = x; q.y = y; q.z = z; q.w = w;
	return q;
}
// 由包含 (x, y, z) 的向量和标量 w 构造四元数。
// 注意：这不是轴-角构造。
static inline struct quat quatvw(struct vec v, float w) {
	struct quat q;
	q.x = v.x; q.y = v.y; q.z = v.z;
	q.w = w;
	return q;
}
// 构造单位四元数。
static inline struct quat qeye(void) {
	return mkquat(0, 0, 0, 1);
}
// 由旋转轴和角度构造四元数。
// 不假设轴已归一化。
static inline struct quat qaxisangle(struct vec axis, float angle) {
	float scale = sinf(angle / 2) / vmag(axis);
	struct quat q;
	q.x = scale * axis.x;
	q.y = scale * axis.y;
	q.z = scale * axis.z;
	q.w = cosf(angle/2);
	return q;
}

// 前置声明，某些构造函数需要
static inline struct quat qnormalize(struct quat q);

// 构造四元数，使得 q * a = b，
// 且旋转轴与 a、b 所定义的平面正交，
// 旋转角小于 180 度。
// 假设 a 与 b 为单位向量。
// 不处理 a = -b 的退化情形，返回全零四元数
static inline struct quat qvectovec(struct vec a, struct vec b) {
	struct vec const cross = vcross(a, b);
	float const sinangle = vmag(cross);
	float const cosangle = vdot(a, b);
	// 避免因浮点误差对负数开方。
	// TODO：寻找更紧的精确界限
	float const EPS_ANGLE = 1e-6;
	if (sinangle < EPS_ANGLE) {
		if (cosangle > 0.0f) return qeye();
		else return mkquat(0.0f, 0.0f, 0.0f, 0.0f); // degenerate
	}
	float const halfcos = 0.5f * cosangle;
	// 由于角度 < 180 度，取正平方根即可
	float const sinhalfangle = sqrtf(fmax(0.5f - halfcos, 0.0f));
	float const coshalfangle = sqrtf(fmax(0.5f + halfcos, 0.0f));
	struct vec const qimag = vscl(sinhalfangle / sinangle, cross);
	float const qreal = coshalfangle;
	return quatvw(qimag, qreal);
}
// 使用 Tait-Bryan 约定由 (roll, pitch, yaw) 欧拉角构造
//（先 yaw，再绕新的 pitch 轴旋转，最后绕新的 roll 轴旋转）
static inline struct quat rpy2quat(struct vec rpy) {
	// 来源：https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	float r = rpy.x;
	float p = rpy.y;
	float y = rpy.z;
	float cr = cosf(r / 2.0f); float sr = sinf(r / 2.0f);
	float cp = cosf(p / 2.0f); float sp = sinf(p / 2.0f);
	float cy = cosf(y / 2.0f); float sy = sinf(y / 2.0f);

	float qx = sr * cp * cy -  cr * sp * sy;
	float qy = cr * sp * cy +  sr * cp * sy;
	float qz = cr * cp * sy -  sr * sp * cy;
	float qw = cr * cp * cy +  sr * sp * sy;

	return mkquat(qx, qy, qz, qw);
}
// 由小角度 (roll, pitch, yaw) 欧拉角近似构造四元数，
// 无需计算三角函数；仅对小角度时有效。
// 典型应用：物体角速度相对采样频率较小时的陀螺积分。
static inline struct quat rpy2quat_small(struct vec rpy) {
	// TODO：补充引用，可由 rpy2quat 的一阶近似推导：
	// sin(epsilon) = epsilon, cos(epsilon) = 1, epsilon^2 = 0
	float q2 = vmag2(rpy) / 4.0f;
	if (q2 < 1) {
		return quatvw(vdiv(rpy, 2), sqrtf(1.0f - q2));
	}
	else {
		float w = 1.0f / sqrtf(1.0f + q2);
		return quatvw(vscl(w/2, rpy), w);
	}
}
// 由正交规范矩阵构造四元数。
static inline struct quat mat2quat(struct mat33 m) {
	float w = sqrtf(fmax(0.0f, 1.0f + m.m[0][0] + m.m[1][1] + m.m[2][2])) / 2.0f;
	float x = sqrtf(fmax(0.0f, 1.0f + m.m[0][0] - m.m[1][1] - m.m[2][2])) / 2.0f;
	float y = sqrtf(fmax(0.0f, 1.0f - m.m[0][0] + m.m[1][1] - m.m[2][2])) / 2.0f;
	float z = sqrtf(fmax(0.0f, 1.0f - m.m[0][0] - m.m[1][1] + m.m[2][2])) / 2.0f;
	x = copysign(x, m.m[2][1] - m.m[1][2]);
	y = copysign(y, m.m[0][2] - m.m[2][0]);
	z = copysign(z, m.m[1][0] - m.m[0][1]);
	return mkquat(x, y, z, w);
}

//
// 转换为其他 3D 旋转参数形式
//

// 使用 Tait-Bryan 约定将四元数转换为 (roll, pitch, yaw) 欧拉角
//（先 yaw，再绕新的 pitch 轴旋转，最后绕新的 roll 轴旋转）
static inline struct vec quat2rpy(struct quat q) {
	// 来源：https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	struct vec v;
	v.x = atan2f(2.0f * (q.w * q.x + q.y * q.z), 1 - 2 * (fsqr(q.x) + fsqr(q.y))); // roll
	v.y = asinf(2.0f * (q.w * q.y - q.x * q.z)); // pitch
	v.z = atan2f(2.0f * (q.w * q.z + q.x * q.y), 1 - 2 * (fsqr(q.y) + fsqr(q.z))); // yaw
	return v;
}
// 计算四元数轴-角分解中的旋转轴。
static inline struct vec quat2axis(struct quat q) {
	// TODO：对微小旋转数值稳定性不足
	float s = 1.0f / sqrtf(1.0f - q.w * q.w);
	return vscl(s, mkvec(q.x, q.y, q.z));
}
// 计算四元数轴-角分解的角度。
// 结果位于 (-pi, pi]。
static inline float quat2angle(struct quat q) {
	float angle = 2 * acosf(q.w);
	if (angle > M_PI_F) {
		angle -= 2.0f * M_PI_F;
	}
	return angle;
}
// 返回四元数的虚部向量，即 (x, y, z)。
static inline struct vec quatimagpart(struct quat q) {
	return mkvec(q.x, q.y, q.z);
}
// 将四元数转换为 3x3 旋转矩阵。
static inline struct mat33 quat2rotmat(struct quat q) {
	// 来源：https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	float x = q.x;
	float y = q.y;
	float z = q.z;
	float w = q.w;

	struct mat33 m;
	m.m[0][0] = 1 - 2*y*y - 2*z*z;
	m.m[0][1] = 2*x*y - 2*z*w;
	m.m[0][2] = 2*x*z + 2*y*w,
	m.m[1][0] = 2*x*y + 2*z*w;
	m.m[1][1] = 1 - 2*x*x - 2*z*z;
	m.m[1][2] = 2*y*z - 2*x*w,
	m.m[2][0] = 2*x*z - 2*y*w;
	m.m[2][1] = 2*y*z + 2*x*w;
	m.m[2][2] = 1 - 2*x*x - 2*y*y;
	return m;
}

//
// 运算符
//

// 用四元数旋转向量。
static inline struct vec qvrot(struct quat q, struct vec v) {
	// 来源：http://gamedev.stackexchange.com/a/50545 - TODO：补充正式引用
	struct vec qv = mkvec(q.x, q.y, q.z);
	return vadd3(
		vscl(2.0f * vdot(qv, v), qv),
		vscl(q.w * q.w - vmag2(qv), v),
		vscl(2.0f * q.w, vcross(qv, v))
	);
}
// 四元数相乘（组合），
// 满足 qvrot(qqmul(q, p), v) == qvrot(q, qvrot(p, v))。
static inline struct quat qqmul(struct quat q, struct quat p) {
	float x =  q.w*p.x + q.z*p.y - q.y*p.z + q.x*p.w;
	float y = -q.z*p.x + q.w*p.y + q.x*p.z + q.y*p.w;
	float z =  q.y*p.x - q.x*p.y + q.w*p.z + q.z*p.w;
	float w = -q.x*p.x - q.y*p.y - q.z*p.z + q.w*p.w;
	return mkquat(x, y, z, w);
}
// 四元数求逆。
static inline struct quat qinv(struct quat q) {
	return mkquat(-q.x, -q.y, -q.z, q.w);
}
// 四元数取负。
// 表示相同旋转，但仍有时有用。
static inline struct quat qneg(struct quat q) {
	return mkquat(-q.x, -q.y, -q.z, -q.w);
}
// 返回表示相同旋转、且实部 (q.w) 为正的四元数。
// 可用于消除四元数对 SO(3) 的双覆盖。
static inline struct quat qposreal(struct quat q) {
	if (q.w < 0) return qneg(q);
	return q;
}
// 四元数点积（为它们夹角余弦）。
static inline float qdot(struct quat a, struct quat b) {
	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}
static inline float qanglebetween(struct quat a, struct quat b) {
	float const dot = qdot(qposreal(a), qposreal(b));
	// 避免 acos 的定义域问题
	if (dot > 1.0f - 1e9f) return 0.0f;
	if (dot < -1.0f + 1e9f) return M_PI_F;
	return acosf(dot);
}
static inline bool qeq(struct quat a, struct quat b) {
	return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}
// 四元数归一化。
// 通常用于减小精度误差。
static inline struct quat qnormalize(struct quat q) {
	float s = 1.0f / sqrtf(qdot(q, q));
	return mkquat(s*q.x, s*q.y, s*q.z, s*q.w);
}
// 使用陀螺仪读数在 dt 时间内更新姿态四元数。
// 假设陀螺仪输出为 (roll, pitch, yaw) 角速度，单位为 rad/s。
static inline struct quat quat_gyro_update(struct quat quat, struct vec gyro, float const dt) {
	// 来源："Indirect Kalman Filter for 3D Attitude Estimation", N. Trawny, 2005
	struct quat q1;
	float const r = (dt / 2) * gyro.x;
	float const p = (dt / 2) * gyro.y;
	float const y = (dt / 2) * gyro.z;

	q1.x =    quat.x + y*quat.y - p*quat.z + r*quat.w;
	q1.y = -y*quat.x +   quat.y + r*quat.z + p*quat.w;
	q1.z =  p*quat.x - r*quat.y +   quat.z + y*quat.w;
	q1.w = -r*quat.x - p*quat.y - y*quat.z +   quat.w;
	return q1;
}
// 归一化线性插值，s 取值应在 0 到 1 之间。
static inline struct quat qnlerp(struct quat a, struct quat b, float t) {
	float s = 1.0f - t;
	return qnormalize(mkquat(
		s*a.x + t*b.x, s*a.y + t*b.y, s*a.z + t*b.z, s*a.w + t*b.w));
}
// 球面线性插值，s 取值应在 0 到 1 之间。
static inline struct quat qslerp(struct quat a, struct quat b, float t)
{
	// 来源："Animating Rotation with Quaternion Curves", Ken Shoemake, 1985
	float dp = qdot(a, b);
	if (dp < 0) {
		dp = -dp;
		b = qneg(b);
	}

	if (dp > 0.99f) {
		// 回退到线性插值以避免除零
		return qnlerp(a, b, t);
	}
	else {
		float theta = acosf(dp);
		float s = sinf(theta * (1 - t)) / sinf(theta);
		t = sinf(theta * t) / sinf(theta);
		return mkquat(
			s*a.x + t*b.x, s*a.y + t*b.y, s*a.z + t*b.z, s*a.w + t*b.w);
	}
}

//
// 与原始 float/double 数组互转。
//

// 从 double 数组加载四元数。
static inline struct quat qload(double const *d) {
	return mkquat(d[0], d[1], d[2], d[3]);
}
// 将四元数存入 double 数组。
static inline void qstore(struct quat q, double *d) {
	d[0] = (double)q.x; d[1] = (double)q.y; d[2] = (double)q.z; d[3] = (double)q.w;
}
// 从 float 数组加载四元数。
static inline struct quat qloadf(float const *f) {
	return mkquat(f[0], f[1], f[2], f[3]);
}
// 将四元数存入 float 数组。
static inline void qstoref(struct quat q, float *f) {
	f[0] = q.x; f[1] = q.y; f[2] = q.z; f[3] = q.w;
}


// ------------------------ R^3 中的凸集 ---------------------------

// 将 v 投影到半空间 H = {x : a^T x <= b}，其中 a 为单位向量。
// 若 v 在 H 内，返回 v；否则返回最近点，该点最小化 |x - v|_2，并满足 a^T x = b。
static inline struct vec vprojecthalfspace(struct vec x, struct vec a_unit, float b) {
	float ax = vdot(a_unit, x);
	if (ax <= b) {
		return x;
	}
	return vadd(x, vscl(b - ax, a_unit));
}

// 判断 v 是否位于由线性不等式 Ax <= b 定义的凸多面体内。
// A: n x 3 矩阵，行优先。每行 L2 范数为 1。
// b: n 维向量。
// tolerance: 允许最多 Ax <= b + tolerance 的违约。
static inline bool vinpolytope(struct vec v, float const A[], float const b[], int n, float tolerance)
{
	for (int i = 0; i < n; ++i) {
		struct vec a = vloadf(A + 3 * i);
		if (vdot(a, v) > b[i] + tolerance) {
			return false;
		}
	}
	return true;
}

// 求射线与凸多面体边界的交点。
// 多面体由线性不等式 Ax <= b 定义。
// 射线必须起始于多面体内部。
//
// 参数：
//   origin: 射线起点，必须位于多面体内部。
//   direction: 射线方向。
//   A: n x 3 矩阵，行优先。每行 L2 范数为 1。
//   b: n 维向量。
//   n: 不等式数量（A 的行数）。
//
// 返回：
//   s: 正数，使得 (origin + s * direction) 位于多面体边界。若射线不与
//     多面体相交（例如多面体无界），返回 INFINITY。若多面体为空，
//     返回负数。若 `origin` 不在多面体内，返回值未定义。
//   active_row: 输出参数，A 中与射线相交的行（多面体面）。点
//     (origin + s * direction) 在该行满足等式。若交点位于多个面的交线上，
//     active_row 将为相交集合中的任意成员。可选，允许为 NULL。
//
static inline float rayintersectpolytope(struct vec origin, struct vec direction, float const A[], float const b[], int n, int *active_row)
{
	#ifdef CMATH3D_ASSERTS
	// 检查输入是否已归一化。
	for (int i = 0; i < n; ++i) {
		struct vec a = vloadf(A + 3 * i);
		assert(fabsf(vmag2(a) - 1.0f) < 1e-6f);
	}
	#endif

	float min_s = INFINITY;
	int min_row = -1;

	for (int i = 0; i < n; ++i) {
		struct vec a = vloadf(A + 3 * i);
		float a_dir = vdot(a, direction);
		if (a_dir <= 0.0f) {
			// 射线指向远离或与该面平行，因此不会相交。
			continue;
		}
		// 用代数方式求该半空间的交点参数。
		float s = (b[i] - vdot(a, origin)) / a_dir;
		if (s < min_s) {
			min_s = s;
			min_row = i;
		}
	}

	if (active_row != NULL) {
		*active_row = min_row;
	}
	return min_s;
}

// 将 v 投影到由线性不等式 Ax <= b 定义的凸多面体。
// 返回 argmin_{x: Ax <= b} |x - v|_2。使用 Dykstra（非 Dijkstra！）
// 投影算法 [1] 以及稳健停止准则 [2]。
//
// 参数：
//   v: 需要投影的向量。
//   A: n x 3 矩阵，行优先。每行 L2 范数为 1。
//   b: n 维向量。
//   work: n x 3 矩阵，会被覆盖，输入值不使用。
//   tolerance: 当 *近似* 违反约束不超过该值时停止；并非严格精确，
//     需要保守时请取更小值。
//   maxiters: 无论是否收敛，迭代到该次数即停止。
//
// 返回：
//   v 在多面体内的投影。
//
// 参考文献：
//   [1] Boyle, J. P., and Dykstra, R. L. (1986). A Method for Finding
//       Projections onto the Intersection of Convex Sets in Hilbert Spaces.
//       Lecture Notes in Statistics, 28–47. doi:10.1007/978-1-4613-9940-7_3
//   [2] Birgin, E. G., and Raydan, M. (2005). Robust Stopping Criteria for
//       Dykstra's Algorithm. SIAM J. Scientific Computing 26(4): 1405-1414.
//       doi:10.1137/03060062X
//
static inline struct vec vprojectpolytope(struct vec v, float const A[], float const b[], float work[], int n, float tolerance, int maxiters)
{
	// 快速返回。
	if (vinpolytope(v, A, b, n, tolerance)) {
		return v;
	}

	#ifdef CMATH3D_ASSERTS
	// 检查输入是否已归一化。
	for (int i = 0; i < n; ++i) {
		struct vec a = vloadf(A + 3 * i);
		assert(fabsf(vmag2(a) - 1.0f) < 1e-6f);
	}
	#endif

	float *z = work;
	for (int i = 0; i < 3 * n; ++i) {
		z[i] = 0.0f;
	}

	// 为方便使用，tolerance 采用多面体约束违约的欧氏量级表述。
	// 但我们使用的停止准则是 [2] 中更稳健的形式——
	// 具体为 c_I^k 表达式，基于投影残差平方和。
	// 这里做了一个粗略折算，得到大致等价的容差量级。
	float const tolerance2 = n * fsqr(tolerance) / 10.0f;
	struct vec x = v;

	for (int iter = 0; iter < maxiters; ++iter) {
		float c = 0.0f;
		for (int i = 0; i < n; ++i) {
			struct vec x_old = x;
			struct vec ai = vloadf(A + 3 * i);
			struct vec zi_old = vloadf(z + 3 * i);
			x = vprojecthalfspace(vsub(x_old, zi_old), ai, b[i]);
			struct vec zi = vadd3(x, vneg(x_old), zi_old);
			vstoref(zi, z + 3 * i);
			c += vdist2(zi_old, zi);
		}
		if (c < tolerance2) {
			return x;
		}
	}
	return x;
}


// 总体 TODO：直线？线段？平面？轴对齐盒？球？
