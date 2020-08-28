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

#ifndef __SENSORS_UWB_H__
#define __SENSORS_UWB_H__

#include "sensors.h"

void sensorsUwbInit(void);
bool sensorsUwbTest(void);
bool sensorsUwbAreCalibrated(void);
bool sensorsUwbManufacturingTest(void);
void sensorsUwbAcquire(sensorData_t *sensors, const uint32_t tick);
void sensorsUwbWaitDataReady(void);
bool sensorsUwbReadGyro(Axis3f *gyro);
bool sensorsUwbReadAcc(Axis3f *acc);
bool sensorsUwbReadMag(Axis3f *mag);
bool sensorsUwbReadBaro(baro_t *baro);
void sensorsUwbSetAccMode(accModes accMode);
void sensorsUwbDataAvailableCallback(void);

#endif // __SENSORS_UWB_H__