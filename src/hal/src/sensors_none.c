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
#include "bmi088.h"
#include "bmp3.h"
#include "bstdr_types.h"
#include "static_mem.h"
#include "serial_commands.h"

#define SENSORS_READ_RATE_HZ            1000


static xQueueHandle accelerometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(accelerometerDataQueue, 1, sizeof(Axis3f));
static xQueueHandle gyroDataQueue;
STATIC_MEM_QUEUE_ALLOC(gyroDataQueue, 1, sizeof(Axis3f));
static xQueueHandle magnetometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(magnetometerDataQueue, 1, sizeof(Axis3f));
static xQueueHandle barometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(barometerDataQueue, 1, sizeof(baro_t));


static bool isInit = false;
static sensorData_t sensorData;
static volatile uint64_t imuIntTimestamp;

#if defined(SENSORS_GYRO_BIAS_CALCULATE_STDDEV) && defined (GYRO_BIAS_LIGHT_WEIGHT)
static Axis3f gyroBiasStdDev;
#endif

// Low Pass filtering
#define GYRO_LPF_CUTOFF_FREQ  80
#define ACCEL_LPF_CUTOFF_FREQ 30

static bool isBarometerPresent = false;

bool sensorsNoneReadGyro(Axis3f *gyro)
{
  return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}

bool sensorsNoneReadAcc(Axis3f *acc)
{
  return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}

bool sensorsNoneReadMag(Axis3f *mag)
{
  return (pdTRUE == xQueueReceive(magnetometerDataQueue, mag, 0));
}

bool sensorsNoneReadBaro(baro_t *baro)
{
  return (pdTRUE == xQueueReceive(barometerDataQueue, baro, 0));
}

void sensorsNoneAcquire(sensorData_t *sensors, const uint32_t tick)
{
  sensorsReadGyro(&sensors->gyro);
  sensorsReadAcc(&sensors->acc);
  sensorsReadMag(&sensors->mag);
  sensorsReadBaro(&sensors->baro);
  sensors->interruptTimestamp = sensorData.interruptTimestamp;
}

bool sensorsNoneAreCalibrated()
{
  return true;
}

void sensorsNoneWaitDataReady(void)
{
  sendSerialVelocity(3,9);
  vTaskDelay(M2T(100));
}

static void sensorsTaskInit(void)
{
  accelerometerDataQueue = STATIC_MEM_QUEUE_CREATE(accelerometerDataQueue);
  gyroDataQueue = STATIC_MEM_QUEUE_CREATE(gyroDataQueue);
  magnetometerDataQueue = STATIC_MEM_QUEUE_CREATE(magnetometerDataQueue);
  barometerDataQueue = STATIC_MEM_QUEUE_CREATE(barometerDataQueue);
}

void sensorsNoneInit(void)
{
  if (isInit)
    {
      return;
    }

  //i2cdevInit(I2C3_DEV);
  
  sensorsTaskInit();
  isInit = true;
}


bool sensorsNoneTest(void)
{
  bool testStatus = true;

  if (!isInit)
  {
    DEBUG_PRINT("Uninitialized\n");
    testStatus = false;
  }
  return testStatus;
}

bool sensorsNoneManufacturingTest(void)
{
  bool testStatus = true;
  return testStatus;
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

void sensorsNoneDataAvailableCallback(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  imuIntTimestamp = usecTimestamp();
  //xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken)
  {
    portYIELD();
  }
}

PARAM_GROUP_START(imu_sensors)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, BMP388, &isBarometerPresent)
PARAM_GROUP_STOP(imu_sensors)
