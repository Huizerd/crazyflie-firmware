/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2018 Bitcraze AB
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
 * sensors_none.c: Placeholder functions for implementations without IMU
 */

#define DEBUG_MODULE "IMU"

#include <math.h>

#include "sensors_none.h"
#include "stm32fxxx.h"

static bool isInit = false;


bool sensorsNoneReadGyro(Axis3f *gyro)
{
  return false;
}

bool sensorsNoneReadAcc(Axis3f *acc)
{
  return false;
}

bool sensorsNoneReadMag(Axis3f *mag)
{
  return false;
}

bool sensorsNoneReadBaro(baro_t *baro)
{
  return false;
}

void sensorsNoneAcquire(sensorData_t *sensors, const uint32_t tick)
{

}

bool sensorsNoneAreCalibrated()
{
  return true;
}


void sensorsNoneWaitDataReady(void)
{

}

void sensorsNoneInit(void)
{
  isInit = true;
}

bool sensorsNoneTest(void)
{
  return true;
}

bool sensorsNoneManufacturingTest(void)
{
  return true;
}

void sensorsNoneSetAccMode(accModes accMode)
{
  switch (accMode)
  {
    case ACC_MODE_PROPTEST:
      break;
    case ACC_MODE_FLIGHT:
    default:
      break;
  }
}
