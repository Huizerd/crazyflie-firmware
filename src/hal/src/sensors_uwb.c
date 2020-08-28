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
 * sensors_uwb.c: Sensor Implementation of the UWB Board Platform
 */

#define DEBUG_MODULE "IMU"

#include <math.h>

#include "sensors_uwb.h"
#include "stm32fxxx.h"

#include "imu.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "system.h"
#include "configblock.h"
#include "param.h"
#include "log.h"
#include "debug.h"
#include "nvicconf.h"
#include "ledseq.h"
#include "sound.h"
#include "filter.h"
#include "i2cdev.h"
#include "bstdr_types.h"
#include "static_mem.h"
#include "deck_core.h"

#define SENSORS_READ_RATE_HZ            1000

static xQueueHandle barometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(barometerDataQueue, 1, sizeof(baro_t));

static DeckInfo empty_info;  
static const DeckDriver* uwb_driver;
static const DeckDriver* mavic_driver;

static bool isInit = false;
static sensorData_t sensorData;
static volatile uint64_t imuIntTimestamp;

static bool isBarometerPresent = false;

bool sensorsUwbReadGyro(Axis3f *gyro)
{
  return false;
}

bool sensorsUwbReadAcc(Axis3f *acc)
{
  return false;
}

bool sensorsUwbReadMag(Axis3f *mag)
{
  return false;
}

bool sensorsUwbReadBaro(baro_t *baro)
{
  return (pdTRUE == xQueueReceive(barometerDataQueue, baro, 0));
}

void sensorsUwbAcquire(sensorData_t *sensors, const uint32_t tick)
{
  sensorsReadBaro(&sensors->baro);
  sensors->interruptTimestamp = sensorData.interruptTimestamp;
}

bool sensorsUwbAreCalibrated()
{
  return true;
}

void sensorsUwbWaitDataReady(void)
{
  vTaskDelay(M2T(100));
}

static void sensorsTaskInit(void)
{
  barometerDataQueue = STATIC_MEM_QUEUE_CREATE(barometerDataQueue);
}

void sensorsUwbInit(void)
{
  if (isInit)
    {
      return;
    }

  // i2cdevInit(I2C3_DEV);
  // Initialize required deck drivers
  uwb_driver = deckFindDriverByName("bcDWM1000");
  mavic_driver = deckFindDriverByName("mavic");
  uwb_driver->init(&empty_info);
  mavic_driver->init(&empty_info);

  sensorsTaskInit();
  isInit = true;
}


bool sensorsUwbTest(void)
{
  bool testStatus = true;

  if (!isInit)
  {
    DEBUG_PRINT("Uninitialized\n");
    testStatus = false;
  }

  testStatus &= uwb_driver->test();
  testStatus &= mavic_driver->test();

  return testStatus;
}

bool sensorsUwbManufacturingTest(void)
{
  bool testStatus = true;
  return testStatus;
}

void sensorsUwbSetAccMode(accModes accMode)
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


PARAM_GROUP_START(uwb_board)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, Barometer, &isBarometerPresent)
PARAM_GROUP_STOP(uwb_board)
