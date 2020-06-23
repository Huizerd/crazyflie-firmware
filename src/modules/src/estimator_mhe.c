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
 * TODO: tune the rates in Cyberzoo
 */

// Update rates
// Attitude + vertical velocity (sensor fusion)
#define ATTITUDE_UPDATE_RATE RATE_250_HZ
#define ATTITUDE_UPDATE_DT (1.0f / ATTITUDE_UPDATE_RATE)
// Estimation (moving horizon)
#define PREDICTION_UPDATE_RATE RATE_100_HZ
// Position (from uwb2pos)
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
// - snapshot of latest position from position estimation, used by state estimation
static state_t taskEstimatorState;
static point_t posSnapshot;

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

// Predict state forward
static bool predictStateForward(uint32_t osTick, float dt);
// Get position from uwb2pos
static bool updatePosition(void);


/**
 * Task memory allocation
 */

STATIC_MEM_TASK_ALLOC(estimatorMheTask, MHE_TASK_STACKSIZE);


/**
 * Statistics (updates per second)
 */

// Task loops
static STATS_CNT_RATE_DEFINE(loopCounter, ONE_SECOND);
// States predicted forward
static STATS_CNT_RATE_DEFINE(predictionCounter, ONE_SECOND);
// Position updates
static STATS_CNT_RATE_DEFINE(positionCounter, ONE_SECOND);
// State finalizations
static STATS_CNT_RATE_DEFINE(finalCounter, ONE_SECOND);
// Calls from stabilizer loop: state copied + attitude update
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
  // State prediction
  uint32_t lastPrediction = xTaskGetTickCount();
  uint32_t nextPrediction = xTaskGetTickCount();
  // Position estimation
  uint32_t nextPosition = xTaskGetTickCount();
  // Finalization
  uint32_t lastFinal = xTaskGetTickCount();

  // Task loop
  while (true)
  {
    // Take semaphore
    xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);

    // Reset estimator if triggered
    if (coreData.resetEstimation)
    {
      estimatorMheInit();
      coreData.resetEstimation = false;
    }

    // Tracks whether state has been updated and thus needs finalization
    bool doneUpdate = false;

    // Current time
    uint32_t osTick = xTaskGetTickCount();

    // Predict state forward:
    // - update prediction based on previous position and attitude estimate
    // - apply correction based on new position estimate
    if (osTick >= nextPrediction)
    {
      // Compute dt
      float dt = T2S(osTick - lastPrediction);

      // Predict state forward
      if (predictStateForward(osTick, dt))
      {
        lastPrediction = osTick;
        doneUpdate = true;
        STATS_CNT_RATE_EVENT(&predictionCounter);
      }

      // Prediction rate
      nextPrediction = osTick + S2T(1.0f / PREDICTION_UPDATE_RATE);
    }

    // Position update:
    // - update vertical position from laser ranger (if used)
    // - update position from LPS (TDoA or TWR) using projection / multilateration
    if (osTick >= nextPosition)
    {
      // Update position
      if (updatePosition())
      {
        STATS_CNT_RATE_EVENT(&positionCounter);
      }      

      // Position update rate
      nextPosition = osTick + S2T(1.0f / POSITION_UPDATE_RATE);
    }

    // If an update has been made (state predicted forward), we finalize it:
    // - prediction and correction are combined into a full estimate
    if (doneUpdate)
    {
      // Compute dt since last finalization
      float dt = T2S(osTick - lastFinal);

      // Combine
      if (mheCoreFinalize(&coreData, dt))
      {
        lastFinal = osTick;
        STATS_CNT_RATE_EVENT(&finalCounter);
      }

      // Check if state within bounds
      if (!mheSupervisorIsStateWithinBounds(&coreData))
      {
        coreData.resetEstimation = true;
        DEBUG_PRINT("MHE estimate out of bounds, resetting\n");
      }
    }

    // Finally, internal state is externalized:
    // - for use in other modules / stabilizer loop
    // - done every iteration, regardless of update / out-of-bounds
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    mheCoreExternalize(&coreData, &taskEstimatorState, osTick);
    xSemaphoreGive(dataMutex);

    // Counter for loops
    STATS_CNT_RATE_EVENT(&loopCounter);
  }
}


/**
 * Estimator task subfunctions
 */


// Predict state forward: prediction + correction
static bool predictStateForward(uint32_t osTick, float dt)
{
  // Update prediction
  mheCorePredict(&coreData, dt);

  // Update correction
  mheCoreCorrect(&coreData, &posSnapshot, T2S(osTick));

  // Success
  return true;
}


// Get position from uwb2pos
static bool updatePosition(void)
{
  /**
   * TODO: do we need mutex here? Don't think so,
   * only when a function here gets called externally
   */

  // Call externalize from uwb2pos
  uwb2posExternalize(&posSnapshot);

  // Success
  return true;
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
    /**
     * TODO: this correct? Seemed to work as well without...
     */
    laserVelocity(state->acc.z, ATTITUDE_UPDATE_DT);

    // Get attitudes into coreData for state prediction
    /**
     * TODO: like this or with memcpy? Or copy whole state to taskEstimatorState and
     * use from there?
     */
    coreData.att[0] = state->attitude.roll;
    coreData.att[1] = state->attitude.pitch;
    coreData.att[2] = state->attitude.yaw;
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

  // Statistics
  STATS_CNT_RATE_LOG_ADD(rtLoop, &loopCounter)
  STATS_CNT_RATE_LOG_ADD(rtPred, &predictionCounter)
  STATS_CNT_RATE_LOG_ADD(rtPos, &positionCounter)
  STATS_CNT_RATE_LOG_ADD(rtFin, &finalCounter)
  STATS_CNT_RATE_LOG_ADD(rtCall, &stabCallCounter)
LOG_GROUP_STOP(MHE)

// Parameters
PARAM_GROUP_START(MHE)
  PARAM_ADD(PARAM_UINT8, resetEstimation, &coreData.resetEstimation)
PARAM_GROUP_STOP(MHE)
