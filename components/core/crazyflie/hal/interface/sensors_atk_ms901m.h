/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2018 Bitcraze AB
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
 */

#ifndef __SENSORS_ATK_MS901M_H__
#define __SENSORS_ATK_MS901M_H__

#include "sensors.h"

void sensorsAtkMs901mInit(void);
bool sensorsAtkMs901mTest(void);
bool sensorsAtkMs901mAreCalibrated(void);
bool sensorsAtkMs901mManufacturingTest(void);
void sensorsAtkMs901mAcquire(sensorData_t *sensors, const uint32_t tick);
void sensorsAtkMs901mWaitDataReady(void);
bool sensorsAtkMs901mReadGyro(Axis3f *gyro);
bool sensorsAtkMs901mReadAcc(Axis3f *acc);
bool sensorsAtkMs901mReadMag(Axis3f *mag);
bool sensorsAtkMs901mReadBaro(baro_t *baro);
void sensorsAtkMs901mSetAccMode(accModes accMode);
bool sensorsAtkMs901mGetAttitude(attitude_t *attitude);
bool sensorsAtkMs901mGetQuaternion(quaternion_t *quaternion);
bool sensorsAtkMs901mReadReg(uint8_t id, uint8_t *data, uint8_t *len, uint32_t timeout_ms);
bool sensorsAtkMs901mWriteReg(uint8_t id, const uint8_t *data, uint8_t len);
int sensorsAtkMs901mRawTransfer(const uint8_t *tx, uint8_t tx_len, uint8_t *rx, uint8_t rx_len, uint32_t timeout_ms);

#endif // __SENSORS_ATK_MS901M_H__
