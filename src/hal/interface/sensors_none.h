/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
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

#ifndef __SENSORS_NONE_H__
#define __SENSORS_NONE_H__

#include "sensors.h"

void sensorsNoneInit(void);
bool sensorsNoneTest(void);
bool sensorsNoneAreCalibrated(void);
bool sensorsNoneManufacturingTest(void);
void sensorsNoneAcquire(sensorData_t *sensors, const uint32_t tick);
void sensorsNoneWaitDataReady(void);
bool sensorsNoneReadGyro(Axis3f *gyro);
bool sensorsNoneReadAcc(Axis3f *acc);
bool sensorsNoneReadMag(Axis3f *mag);
bool sensorsNoneReadBaro(baro_t *baro);
void sensorsNoneSetAccMode(accModes accMode);
void sensorsNoneDataAvailableCallback(void);

#endif // __SENSORS_NONE_H__