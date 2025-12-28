// I2Cdev library collection - HMC5883L I2C device class
// Based on Honeywell HMC5883L datasheet, 10/2010 (Form #900405 Rev B)
// 6/12/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

/* ============================================
 I2Cdev device library code is placed under the MIT license
 Copyright (c) 2012 Jeff Rowberg
 Adapted to Crazyflie FW by Bitcraze

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ===============================================
 */


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "eprintf.h"
#include "hmc5883l.h"
#include "i2cdev.h"
#define DEBUG_MODULE "HMC5883L"
#include "debug_cf.h"

static uint8_t devAddr;
static uint8_t buffer[6];
static uint8_t mode;
static I2C_Dev *I2Cx;
static bool isInit;

/** 上电并进行通用初始化。
 * 该函数将磁力计设置为默认参数，适用于单次测量模式
 *（功耗很低）。默认设置包括 8 次采样平均、15 Hz 输出速率、
 * 正常测量偏置，以及 1090 增益（单位 LSB/Gauss）。
 * 初始化后可按需调整设置，尤其是增益设置；若出现大量 -4096
 * 值，请参考数据手册。
 */
void hmc5883lInit(I2C_Dev *i2cPort)
{
    if (isInit) {
        return;
    }

    I2Cx = i2cPort;
    devAddr = HMC5883L_ADDRESS;

    // 写 CONFIG_A 寄存器
    i2cdevWriteByte(I2Cx, devAddr, HMC5883L_RA_CONFIG_A,
                    (HMC5883L_AVERAGING_8 << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1)) |
                    (HMC5883L_RATE_15 << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1)) |
                    (HMC5883L_BIAS_NORMAL << (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1))); //配置信息：采样率15hz，每次结果来自8次采样，不进行校准

    // 写 CONFIG_B 寄存器
    hmc5883lSetGain(HMC5883L_GAIN_660); // 测量范围 ±2.5Ga

    isInit = true;
}

/** 验证 I2C 连接。
 * 确保设备已连接并能按预期响应。
 * @return 连接有效返回 true，否则返回 false
 */
bool hmc5883lTestConnection()
{
    if (i2cdevReadReg8(I2Cx, devAddr, HMC5883L_RA_ID_A, 3, buffer)) {
        return (buffer[0] == 'H' && buffer[1] == '4' && buffer[2] == '3');
    }

    return false;
}

/** 执行自检。
 * @return 自检通过返回 true，否则返回 false
 */
bool hmc5883lSelfTest()
{
    bool testStatus = true;
    int16_t mxp, myp, mzp;  // 正向磁场测量
    int16_t mxn, myn, mzn;  // 反向磁场测量
    struct {
        uint8_t configA;
        uint8_t configB;
        uint8_t mode;
    } regSave;

    // 保存寄存器值
    if (i2cdevReadReg8(I2Cx, devAddr, HMC5883L_RA_CONFIG_A, sizeof(regSave), (uint8_t *)&regSave) == false) {
        // TODO：错误处理
        return false;
    }

    // 设置增益（灵敏度）
    hmc5883lSetGain(HMC5883L_ST_GAIN);

    // 写 CONFIG_A 寄存器并进行正向测试
    i2cdevWriteByte(I2Cx, devAddr, HMC5883L_RA_CONFIG_A,
                    (HMC5883L_AVERAGING_1 << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1)) |
                    (HMC5883L_RATE_15 << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1)) |
                    (HMC5883L_BIAS_POSITIVE << (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1)));

    /* 执行测试测量并检查结果 */
    hmc5883lSetMode(HMC5883L_MODE_SINGLE);
    vTaskDelay(M2T(HMC5883L_ST_DELAY_MS));
    hmc5883lGetHeading(&mxp, &myp, &mzp);

    // 写 CONFIG_A 寄存器并进行反向测试
    i2cdevWriteByte(I2Cx, devAddr, HMC5883L_RA_CONFIG_A,
                    (HMC5883L_AVERAGING_1 << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1)) |
                    (HMC5883L_RATE_15 << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1)) |
                    (HMC5883L_BIAS_NEGATIVE << (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1)));

    /* 执行测试测量并检查结果 */
    hmc5883lSetMode(HMC5883L_MODE_SINGLE);
    vTaskDelay(M2T(HMC5883L_ST_DELAY_MS));
    hmc5883lGetHeading(&mxn, &myn, &mzn);

    if (hmc5883lEvaluateSelfTest(HMC5883L_ST_X_MIN, HMC5883L_ST_X_MAX, mxp, "pos X") &&
            hmc5883lEvaluateSelfTest(HMC5883L_ST_Y_MIN, HMC5883L_ST_Y_MAX, myp, "pos Y") &&
            hmc5883lEvaluateSelfTest(HMC5883L_ST_Z_MIN, HMC5883L_ST_Z_MAX, mzp, "pos Z") &&
            hmc5883lEvaluateSelfTest(-HMC5883L_ST_X_MAX, -HMC5883L_ST_X_MIN, mxn, "neg X") &&
            hmc5883lEvaluateSelfTest(-HMC5883L_ST_Y_MAX, -HMC5883L_ST_Y_MIN, myn, "neg Y") &&
            hmc5883lEvaluateSelfTest(-HMC5883L_ST_Z_MAX, -HMC5883L_ST_Z_MIN, mzn, "neg Z")) {
        DEBUG_PRINTD("hmc5883l Self test [OK].\n");
    } else {
        testStatus = false;
    }

    // 恢复寄存器
    if (i2cdevWriteReg8(I2Cx, devAddr, HMC5883L_RA_CONFIG_A, sizeof(regSave), (uint8_t *)&regSave) == false) {
        // TODO：错误处理
        return false;
    }

    return testStatus;
}

/** 评估 HMC8335L 自检结果。
 * @param min 自检最小阈值
 * @param max 自检最大阈值
 * @param value 待比较的值
 * @param string 描述该值的字符串
 * @return 自检值在阈值范围内返回 true，否则返回 false
 */
bool hmc5883lEvaluateSelfTest(int16_t min, int16_t max, int16_t value, char *string)
{
    if (value < min || value > max) {
        DEBUG_PRINTD("Self test %s [FAIL]. low: %d, high: %d, measured: %d\n",
                     string, min, max, value);
        return false;
    }

    return true;
}

// CONFIG_A 寄存器

/** 获取每次测量的平均采样数。
 * @return 当前每次测量的平均采样数（0-3 分别对应 1/2/4/8）
 * @see HMC5883L_AVERAGING_8
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_AVERAGE_BIT
 * @see HMC5883L_CRA_AVERAGE_LENGTH
 */
uint8_t hmc5883lGetSampleAveraging()
{
    i2cdevReadBits(I2Cx, devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_AVERAGE_BIT, HMC5883L_CRA_AVERAGE_LENGTH, buffer);
    return buffer[0];
}
/** 设置每次测量的平均采样数。
 * @param averaging 新的平均采样数设置（0-3 分别对应 1/2/4/8）
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_AVERAGE_BIT
 * @see HMC5883L_CRA_AVERAGE_LENGTH
 */
void hmc5883lSetSampleAveraging(uint8_t averaging)
{
    i2cdevWriteBits(I2Cx, devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_AVERAGE_BIT, HMC5883L_CRA_AVERAGE_LENGTH, averaging);
}
/** 获取数据输出速率。
 * 下表为连续测量模式下可选的输出速率。三个通道会在给定输出速率内完成测量。
 * 通过单次测量模式下监测 DRDY 中断引脚，可实现最高 160 Hz 的输出速率。
 *
 * 数值 | 典型输出速率 (Hz)
 * -----+-------------------
 * 0     | 0.75
 * 1     | 1.5
 * 2     | 3
 * 3     | 7.5
 * 4     | 15（默认）
 * 5     | 30
 * 6     | 75
 * 7     | 未使用
 *
 * @return 当前写入寄存器的输出速率
 * @see HMC5883L_RATE_15
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_RATE_BIT
 * @see HMC5883L_CRA_RATE_LENGTH
 */
uint8_t hmc5883lGetDataRate()
{
    i2cdevReadBits(I2Cx, devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_RATE_BIT, HMC5883L_CRA_RATE_LENGTH, buffer);
    return buffer[0];
}
/** 设置数据输出速率。
 * @param rate 写入寄存器的输出速率
 * @see getDataRate()
 * @see HMC5883L_RATE_15
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_RATE_BIT
 * @see HMC5883L_CRA_RATE_LENGTH
 */
void hmc5883lSetDataRate(uint8_t rate)
{
    i2cdevWriteBits(I2Cx, devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_RATE_BIT, HMC5883L_CRA_RATE_LENGTH, rate);
}
/** 获取测量偏置值。
 * @return 当前偏置值（0-2 分别对应 正常/正/负）
 * @see HMC5883L_BIAS_NORMAL
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_BIAS_BIT
 * @see HMC5883L_CRA_BIAS_LENGTH
 */
uint8_t hmc5883lGetMeasurementBias()
{
    i2cdevReadBits(I2Cx, devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_BIAS_BIT, HMC5883L_CRA_BIAS_LENGTH, buffer);
    return buffer[0];
}
/** 设置测量偏置值。
 * @param bias 新的偏置值（0-2 分别对应 正常/正/负）
 * @see HMC5883L_BIAS_NORMAL
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_BIAS_BIT
 * @see HMC5883L_CRA_BIAS_LENGTH
 */
void hmc5883lSetMeasurementBias(uint8_t bias)
{
    i2cdevWriteBits(I2Cx, devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_BIAS_BIT, HMC5883L_CRA_BIAS_LENGTH, bias);
}

// CONFIG_B 寄存器

/** 获取磁场增益值。
 * 下表为名义增益设置。使用 “Gain” 列将计数转换为高斯。
 * 当总磁场强度导致数据寄存器溢出（饱和）时，选择更低增益值（更高 GN#）。
 * 所有设置的数据输出范围为 0xF800-0x07FF（-2048 - 2047）。
 *
 * 数值 | 磁场范围 | 增益 (LSB/Gauss)
 * -----+----------+-----------------
 * 0     | +/- 0.88 Ga | 1370
 * 1     | +/- 1.3 Ga  | 1090（默认）
 * 2     | +/- 1.9 Ga  | 820
 * 3     | +/- 2.5 Ga  | 660
 * 4     | +/- 4.0 Ga  | 440
 * 5     | +/- 4.7 Ga  | 390
 * 6     | +/- 5.6 Ga  | 330
 * 7     | +/- 8.1 Ga  | 230
 *
 * @return 当前磁场增益值
 * @see HMC5883L_GAIN_1090
 * @see HMC5883L_RA_CONFIG_B
 * @see HMC5883L_CRB_GAIN_BIT
 * @see HMC5883L_CRB_GAIN_LENGTH
 */
uint8_t hmc5883lGetGain()
{
    i2cdevReadBits(I2Cx, devAddr, HMC5883L_RA_CONFIG_B, HMC5883L_CRB_GAIN_BIT, HMC5883L_CRB_GAIN_LENGTH, buffer);
    return buffer[0];
}
/** 设置磁场增益值。
 * @param gain 新的磁场增益值
 * @see getGain()
 * @see HMC5883L_RA_CONFIG_B
 * @see HMC5883L_CRB_GAIN_BIT
 * @see HMC5883L_CRB_GAIN_LENGTH
 */
void hmc5883lSetGain(uint8_t gain)
{
    // 使用该方法可确保位 4-0 清零，这是数据手册的要求；
    // 也比 I2Cdev.writeBits 方法更高效
    i2cdevWriteByte(I2Cx, devAddr, HMC5883L_RA_CONFIG_B, gain << (HMC5883L_CRB_GAIN_BIT - HMC5883L_CRB_GAIN_LENGTH + 1));
}

// MODE 寄存器

/** 获取测量模式。
 * 在连续测量模式下，设备持续进行测量并将结果写入数据寄存器。
 * 当三轴寄存器都有新数据时 RDY 置高。上电或写入模式/配置寄存器后，
 * 第一组测量数据在 2/fDO 时间后可用，随后以 fDO 频率输出，其中 fDO 为输出数据频率。
 *
 * 在单次测量模式（默认）下，设备进行一次测量，置 RDY 高并返回空闲模式。
 * 模式寄存器会恢复为空闲模式位值。测量结果保留在数据输出寄存器中，
 * RDY 保持为高直到读取数据寄存器或执行下一次测量。
 *
 * @return 当前测量模式
 * @see HMC5883L_MODE_CONTINUOUS
 * @see HMC5883L_MODE_SINGLE
 * @see HMC5883L_MODE_IDLE
 * @see HMC5883L_RA_MODE
 * @see HMC5883L_MODEREG_BIT
 * @see HMC5883L_MODEREG_LENGTH
 */
uint8_t hmc5883lGetMode()
{
    i2cdevReadBits(I2Cx, devAddr, HMC5883L_RA_MODE, HMC5883L_MODEREG_BIT, HMC5883L_MODEREG_LENGTH, buffer);
    return buffer[0];
}
/** 设置测量模式。
 * @param newMode 新的测量模式
 * @see getMode()
 * @see HMC5883L_MODE_CONTINUOUS
 * @see HMC5883L_MODE_SINGLE
 * @see HMC5883L_MODE_IDLE
 * @see HMC5883L_RA_MODE
 * @see HMC5883L_MODEREG_BIT
 * @see HMC5883L_MODEREG_LENGTH
 */
void hmc5883lSetMode(uint8_t newMode)
{
    // 使用该方法可确保位 7-2 清零，这是数据手册要求；
    // 也比 I2Cdev.writeBits 方法更高效
    i2cdevWriteByte(I2Cx, devAddr, HMC5883L_RA_MODE, mode << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
    mode = newMode; // 记录当前模式，用于判断读后是否需要清除位 7
}

// DATA* 寄存器

/** 获取三轴航向测量值。
 * 当某通道 ADC 读数溢出/下溢，或偏置测量期间发生数学溢出时，
 * 数据寄存器将包含 -4096。该值会在下一次有效测量后清除。
 * 注意：当处于单次测量模式时，本函数会自动清除 MODE 寄存器中的相应位。
 * @param x 16 位有符号整数容器，保存 X 轴航向
 * @param y 16 位有符号整数容器，保存 Y 轴航向
 * @param z 16 位有符号整数容器，保存 Z 轴航向
 * @see HMC5883L_RA_DATAX_H
 */
void hmc5883lGetHeading(int16_t *x, int16_t *y, int16_t *z)
{
    i2cdevReadReg8(I2Cx, devAddr, HMC5883L_RA_DATAX_H, 6, buffer);

    if (mode == HMC5883L_MODE_SINGLE) {
        i2cdevWriteByte(I2Cx, devAddr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
    }

    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[4]) << 8) | buffer[5];
    *z = (((int16_t)buffer[2]) << 8) | buffer[3];
}
/** 获取 X 轴航向测量值。
 * @return 16 位有符号整数形式的 X 轴航向
 * @see HMC5883L_RA_DATAX_H
 */
int16_t hmc5883lGetHeadingX()
{
    // 读取任一轴都必须读取全部轴寄存器，即便只使用其中一轴；
    // 这并非低效实现，而是器件要求
    i2cdevReadReg8(I2Cx, devAddr, HMC5883L_RA_DATAX_H, 6, buffer);

    if (mode == HMC5883L_MODE_SINGLE) {
        i2cdevWriteByte(I2Cx, devAddr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
    }

    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
/** 获取 Y 轴航向测量值。
 * @return 16 位有符号整数形式的 Y 轴航向
 * @see HMC5883L_RA_DATAY_H
 */
int16_t hmc5883lGetHeadingY()
{
    // 读取任一轴都必须读取全部轴寄存器，即便只使用其中一轴；
    // 这并非低效实现，而是器件要求
    i2cdevReadReg8(I2Cx, devAddr, HMC5883L_RA_DATAX_H, 6, buffer);

    if (mode == HMC5883L_MODE_SINGLE) {
        i2cdevWriteByte(I2Cx, devAddr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
    }

    return (((int16_t)buffer[4]) << 8) | buffer[5];
}
/** 获取 Z 轴航向测量值。
 * @return 16 位有符号整数形式的 Z 轴航向
 * @see HMC5883L_RA_DATAZ_H
 */
int16_t hmc5883lGetHeadingZ()
{
    // 读取任一轴都必须读取全部轴寄存器，即便只使用其中一轴；
    // 这并非低效实现，而是器件要求
    i2cdevReadReg8(I2Cx, devAddr, HMC5883L_RA_DATAX_H, 6, buffer);

    if (mode == HMC5883L_MODE_SINGLE) {
        i2cdevWriteByte(I2Cx, devAddr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
    }

    return (((int16_t)buffer[2]) << 8) | buffer[3];
}

// STATUS 寄存器

/** 获取数据输出寄存器锁定状态。
 * 当 6 个数据输出寄存器中只有部分被读取时，该位被置位。
 * 置位后，这 6 个寄存器会被锁定，新数据不会写入，直到满足以下条件之一：
 * 1）读完 6 个字节或模式改变；2）模式改变；3）测量配置改变。
 * @return 数据输出寄存器锁定状态
 * @see HMC5883L_RA_STATUS
 * @see HMC5883L_STATUS_LOCK_BIT
 */
bool hmc5883lGetLockStatus()
{
    i2cdevReadBit(I2Cx, devAddr, HMC5883L_RA_STATUS, HMC5883L_STATUS_LOCK_BIT, buffer);
    return buffer[0];
}
/** 获取数据就绪状态。
 * 当数据写入 6 个数据寄存器时该位被置位；
 * 当设备开始写数据输出寄存器，以及写入一个或多个数据输出寄存器后该位被清除。
 * RDY 清零后会保持 250 us。DRDY 引脚可作为监测测量数据的替代方式。
 * @return 数据就绪状态
 * @see HMC5883L_RA_STATUS
 * @see HMC5883L_STATUS_READY_BIT
 */
bool hmc5883lGetReadyStatus()
{
    i2cdevReadBit(I2Cx, devAddr, HMC5883L_RA_STATUS, HMC5883L_STATUS_READY_BIT, buffer);
    return buffer[0];
}

// ID_* 寄存器

/** 获取标识字节 A
 * @return ID_A 字节（应为 01001000，ASCII 字符 'H'）
 */
uint8_t hmc5883lGetIDA()
{
    i2cdevReadByte(I2Cx, devAddr, HMC5883L_RA_ID_A, buffer);
    return buffer[0];
}
/** 获取标识字节 B
 * @return ID_B 字节（应为 00110100，ASCII 字符 '4'）
 */
uint8_t hmc5883lGetIDB()
{
    i2cdevReadByte(I2Cx, devAddr, HMC5883L_RA_ID_B, buffer);
    return buffer[0];
}
/** 获取标识字节 C
 * @return ID_C 字节（应为 00110011，ASCII 字符 '3'）
 */
uint8_t hmc5883lGetIDC()
{
    i2cdevReadByte(I2Cx, devAddr, HMC5883L_RA_ID_C, buffer);
    return buffer[0];
}
