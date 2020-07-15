/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
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
 * Copyright (C) 2020 Bitcraze AB, Sven Pfeiffer, Huizerd
 *
 * estimator_mhe.c - Moving horizon estimator
 */


/**
 * Includes
 */

// Own
#include "estimator_mhe.h"
// Core functions for the moving horizon estimator
#include "mhe_core.h"
// Supervising function for the moving horizon estimator
#include "mhe_supervisor.h"

// Chip
#include "stm32f4xx.h"
// Real-time OS
#include "FreeRTOS.h"
// Data queues
#include "queue.h"
// Tasks
#include "task.h"
// Static memory allocation (for task and queues)
#include "static_mem.h"
// Semaphores
#include "semphr.h"
// System
#include "system.h"

// Heart of the autopilot
#include "stabilizer.h"
// AHRS sensor fusion
#include "sensfusion6.h"
// Position estimation from UWB
#include "uwb2pos.h"
// Update vertical velocity for UWB
#include "uwb2pos_core.h"
// Various sensors (gyro, accelerometers)
#include "sensors.h"
// Important types
#include "stabilizer_types.h"

// Logging
#include "log.h"
// Adjustable parameters
#include "param.h"
// Statistics
#include "statsCnt.h"

// Debugging
#define DEBUG_MODULE "MHE"
#include "debug.h"

// memcpy
#include <string.h>


/**
 * File constants
 */

// Update rates
// Attitude (+ vertical velocity) update: sensor fusion
#define ATTITUDE_UPDATE_RATE RATE_500_HZ
#define ATTITUDE_UPDATE_DT (1.0f / ATTITUDE_UPDATE_RATE)
// Position update:
// - update position prediction based on attitude
// - update corrector if new UWB measurement
// - combine prediction and corrector (finalization)
#define POSITION_UPDATE_RATE RATE_100_HZ

// One second in ticks
#define ONE_SECOND (1000)


/**
 * File statics
 */

// Quadcopter state for logging / storing
static mheCoreData_t coreData;

// Data used to enable the task and stabilizer loop to run with minimal locking:
// - estimator state produced by task, copied to stabilizer when needed
static state_t taskEstimatorState;

// Snapshots:
// - of latest position from UWB queue
// - of latest attitude in case averaging (below) fails
static point_t uwbQueueSnapshot;
static attitude_t attSnapshot;

// Accumulator + counter for attitude
static attitude_t attAccumulator;
static int attCount = 0;

// Semaphores:
// - task semaphore to signal that we got data from stabilizer loop
// - mutex to protect data shared between task and stabilizer loop
static SemaphoreHandle_t runTaskSemaphore;
static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;

// Check for initialization
static bool isInit = false;


/**
 * Static function prototypes
 */

// Task
static void estimatorMheTask(void* parameters);

// Update position prediction based on attitude
static void updatePrediction(float dt);
// Update corrector with UWB measurement
static bool updateCorrector(uint32_t osTick);


/**
 * Task memory allocation
 */

STATIC_MEM_TASK_ALLOC(estimatorMheTask, MHE_TASK_STACKSIZE);


/**
 * Statistics (updates per second)
 */

// Task loops
static STATS_CNT_RATE_DEFINE(loopCounter, ONE_SECOND);
// Updates to position (prediction + finalization)
static STATS_CNT_RATE_DEFINE(updateCounter, ONE_SECOND);
// Updates to corrector
static STATS_CNT_RATE_DEFINE(correctorCounter, ONE_SECOND);
// Calls from stabilizer loop
static STATS_CNT_RATE_DEFINE(stabCallCounter, ONE_SECOND);


/**
 * Estimator task functions
 */


void estimatorMheTaskInit(void)
{
  // Create binary semaphore for task handling
  vSemaphoreCreateBinary(runTaskSemaphore);

  // Create mutex for sharing data
  dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);

  // Create task
  STATIC_MEM_TASK_CREATE(estimatorMheTask, estimatorMheTask, MHE_TASK_NAME, NULL, MHE_TASK_PRI);

  // Did we do the above?
  isInit = true;
}


// Trivial test that checks for task init
bool estimatorMheTaskTest(void)
{
  return isInit;
}


// Main task function
static void estimatorMheTask(void* parameters)
{
  // Wait for startup
  systemWaitStart();

  // Counters
  // Position update
  uint32_t lastUpdate = xTaskGetTickCount();
  uint32_t nextUpdate = xTaskGetTickCount();

  // Task loop
  while (true)
  {
    // Take semaphore
    xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);

    // Reset estimator if triggered
    if (coreData.resetEstimation)
    {
      uwb2posReset();
      estimatorMheInit();
      coreData.resetEstimation = false;
    }

    // Current time
    uint32_t osTick = xTaskGetTickCount();

    // Position update:
    // - update position prediction based on attitude
    // - update corrector if new UWB measurement
    // - combine prediction and corrector (finalization)
    if (osTick >= nextUpdate)
    {
      // Get dt
      float dt = T2S(osTick - lastUpdate);

      // Update position prediction
      updatePrediction(dt);

      // Update corrector if new UWB measurement
      if (updateCorrector(osTick))
        STATS_CNT_RATE_EVENT(&correctorCounter);
      
      // Finalization: combine prediction and corrector
      mheCoreFinalize(&coreData, dt);

      // Check if state within bounds
      // If so, externalize to stabilizer / other modules
      if (mheSupervisorIsStateWithinBounds(&coreData))
      {
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        mheCoreExternalize(&coreData, &taskEstimatorState, osTick);
        xSemaphoreGive(dataMutex);

        // Update counters
        lastUpdate = osTick;
        STATS_CNT_RATE_EVENT(&updateCounter);
      }
      else
      {
        coreData.resetEstimation = true;
        DEBUG_PRINT("MHE estimate out of bounds, resetting\n");
      }

      // Next update
      nextUpdate = osTick + S2T(1.0f / POSITION_UPDATE_RATE);
    }

    // Counter for loops
    STATS_CNT_RATE_EVENT(&loopCounter);
  }
}


/**
 * Estimator task subfunctions
 */


// Update position prediction
static void updatePrediction(float dt)
{
  // Get averaged attitudes from accumulators
  // Needs check for nan
  if (attCount > 0)
  {
    coreData.att[0] = attAccumulator.roll / attCount;
    coreData.att[1] = attAccumulator.pitch / attCount;
    coreData.att[2] = attAccumulator.yaw / attCount;
    memset(&attAccumulator, 0, sizeof(attitude_t));
    attCount = 0;
  }
  else
  {
    coreData.att[0] = attSnapshot.roll;
    coreData.att[1] = attSnapshot.pitch;
    coreData.att[2] = attSnapshot.yaw;
  }

  // Update prediction
  mheCoreUpdatePrediction(&coreData, dt);
}


// Update corrector if new UWB measurement
static bool updateCorrector(uint32_t osTick)
{
  // Check if new UWB measurement
  if (latestPosMeasurement(&uwbQueueSnapshot))
  {
    // Update corrector
    mheCoreUpdateCorrector(&coreData, &uwbQueueSnapshot, osTick);
    return true;
  }
  else
  {
    // No update
    return false;
  }
}


/**
 * Main estimator functions
 */


// Init / reset
void estimatorMheInit(void)
{
  // Initialize sensor fusion
  sensfusion6Init();

  // Initialize moving horizon estimator
  mheCoreInit(&coreData);

  // Reset snapshots
  memset(&attSnapshot, 0, sizeof(attitude_t));

  // Reset accumulators / counts
  memset(&attAccumulator, 0, sizeof(attitude_t));
  attCount = 0;
}


// Some trivial test, only tests sensor fusion and initialization
bool estimatorMheTest(void)
{
  // Check sensor fusion
  bool pass = true;
  pass &= sensfusion6Test();

  // Check initialization has occurred
  pass &= isInit;

  return pass;
}


// Estimator function called from stabilizer loop
// Largely based on complementary estimator
void estimatorMhe(state_t* state, sensorData_t* sensorData, control_t* control, uint32_t tick)
{
  // Call should return as quickly as possible
  xSemaphoreTake(dataMutex, portMAX_DELAY);

  // Read sensors at full rate (1000 Hz)
  sensorsAcquire(sensorData, tick);

  // Copy position and velocity estimated here to stabilizer loop
  // dest, src
  memcpy(state, &taskEstimatorState, sizeof(state_t));

  // Update with sensor fusion:
  // - attitude
  // - vertical acceleration + velocity
  if (RATE_DO_EXECUTE(ATTITUDE_UPDATE_RATE, tick)) {

    // Update sensors
    sensfusion6UpdateQ(sensorData->gyro.x, sensorData->gyro.y, sensorData->gyro.z,
                       sensorData->acc.x, sensorData->acc.y, sensorData->acc.z,
                       ATTITUDE_UPDATE_DT);

    // Save attitude, adjusted for the legacy CF2 body coordinate system
    sensfusion6GetEulerRPY(&state->attitude.roll,
                           &state->attitude.pitch,
                           &state->attitude.yaw);

    // Save quaternion, hopefully one day this could be used in a better controller.
    // Note that this is not adjusted for the legacy coordinate system
    sensfusion6GetQuaternion(
      &state->attitudeQuaternion.x,
      &state->attitudeQuaternion.y,
      &state->attitudeQuaternion.z,
      &state->attitudeQuaternion.w);

    // Save vertical acceleration
    state->acc.z = sensfusion6GetAccZWithoutGravity(sensorData->acc.x,
                                                    sensorData->acc.y,
                                                    sensorData->acc.z);

    // Update vertical velocity for laser height estimation
    laserVelocity(state->acc.z, ATTITUDE_UPDATE_DT);

    // Save to snapshot
    attSnapshot.roll = state->attitude.roll;
    attSnapshot.pitch = state->attitude.pitch;
    attSnapshot.yaw = state->attitude.yaw;

    // Add to attitude accumulator
    attAccumulator.roll += state->attitude.roll;
    attAccumulator.pitch += state->attitude.pitch;
    attAccumulator.yaw += state->attitude.yaw;
    attCount++;
  }

  // Give back semaphores
  xSemaphoreGive(dataMutex);
  xSemaphoreGive(runTaskSemaphore);

  // Counter for calls
  STATS_CNT_RATE_EVENT(&stabCallCounter);
}


/**
 * Logging and adjustable parameters
 */

// Stock group
LOG_GROUP_START(MHE)
  // Complete state
  LOG_ADD(LOG_FLOAT, predX, &coreData.S[MHC_STATE_X])
  LOG_ADD(LOG_FLOAT, predY, &coreData.S[MHC_STATE_Y])
  LOG_ADD(LOG_FLOAT, predZ, &coreData.S[MHC_STATE_Z])
  LOG_ADD(LOG_FLOAT, predVX, &coreData.S[MHC_STATE_VX])
  LOG_ADD(LOG_FLOAT, predVY, &coreData.S[MHC_STATE_VY])
  LOG_ADD(LOG_FLOAT, predVZ, &coreData.S[MHC_STATE_VZ])
  LOG_ADD(LOG_FLOAT, corrX, &coreData.C[MHC_STATE_X])
  LOG_ADD(LOG_FLOAT, corrY, &coreData.C[MHC_STATE_Y])
  LOG_ADD(LOG_FLOAT, corrZ, &coreData.C[MHC_STATE_Z])
  LOG_ADD(LOG_FLOAT, corrVX, &coreData.C[MHC_STATE_VX])
  LOG_ADD(LOG_FLOAT, corrVY, &coreData.C[MHC_STATE_VY])
  LOG_ADD(LOG_FLOAT, corrVZ, &coreData.C[MHC_STATE_VZ])
  LOG_ADD(LOG_FLOAT, finX, &coreData.F[MHC_STATE_X])
  LOG_ADD(LOG_FLOAT, finY, &coreData.F[MHC_STATE_Y])
  LOG_ADD(LOG_FLOAT, finZ, &coreData.F[MHC_STATE_Z])
  LOG_ADD(LOG_FLOAT, finVX, &coreData.F[MHC_STATE_VX])
  LOG_ADD(LOG_FLOAT, finVY, &coreData.F[MHC_STATE_VY])
  LOG_ADD(LOG_FLOAT, finVZ, &coreData.F[MHC_STATE_VZ])
  LOG_ADD(LOG_FLOAT, roll, &coreData.att[0])
  LOG_ADD(LOG_FLOAT, pitch, &coreData.att[1])
  LOG_ADD(LOG_FLOAT, yaw, &coreData.att[2])
  LOG_ADD(LOG_UINT32, attCount, &attCount)

  // Variabes used for importing / exporting
  LOG_ADD(LOG_FLOAT, exX, &taskEstimatorState.position.x)
  LOG_ADD(LOG_FLOAT, exY, &taskEstimatorState.position.y)
  LOG_ADD(LOG_FLOAT, exZ, &taskEstimatorState.position.z)
  LOG_ADD(LOG_FLOAT, uwbQueueX, &uwbQueueSnapshot.x)
  LOG_ADD(LOG_FLOAT, uwbQueueY, &uwbQueueSnapshot.y)
  LOG_ADD(LOG_FLOAT, uwbQueueZ, &uwbQueueSnapshot.z)

  // Statistics
  STATS_CNT_RATE_LOG_ADD(rtLoop, &loopCounter)
  STATS_CNT_RATE_LOG_ADD(rtUpdate, &updateCounter)
  STATS_CNT_RATE_LOG_ADD(rtCorrect, &correctorCounter)
  STATS_CNT_RATE_LOG_ADD(rtCall, &stabCallCounter)
LOG_GROUP_STOP(MHE)

// Parameters
PARAM_GROUP_START(MHE)
  PARAM_ADD(PARAM_UINT8, resetEstimation, &coreData.resetEstimation)
PARAM_GROUP_STOP(MHE)
