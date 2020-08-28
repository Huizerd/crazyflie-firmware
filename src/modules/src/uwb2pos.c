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
 * uwb2pos.c - Position estimation from UWB / laser measurements
 */


/**
 * Includes
 */

// Own
#include "uwb2pos.h"
// Core functionality
#include "uwb2pos_core.h"

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
#define DEBUG_MODULE "UWB2POS"
#include "debug.h"

// memcpy
#include <string.h>


/**
 * File constants
 * TODO: tune the rates and queue lengths in Cyberzoo
 */

// Update rates
// Height from laser ranger
#define LASER_UPDATE_RATE RATE_100_HZ
// Position from UWB measurements
#define UWB_UPDATE_RATE RATE_50_HZ

// Measurement queue lengths
#define TOF_QUEUE_LENGTH (1)
#define TDOA_QUEUE_LENGTH (20)
#define DIST_QUEUE_LENGTH (20)

// One second in ticks
#define ONE_SECOND (1000)


/**
 * File statics
 */

// Position estimated from UWB / laser measurements:
// - one for internal use
// - one for interfacing with other modules
// - one for resetting (that we don't change)
static point_t inPosition;
static point_t exPosition;
static const point_t zeroPosition;
// static point_t prevPosition;

// Snapshot of sensor data for use in height estimation (barometer only)
static sensorData_t sensorSnapshot;

// Measurement queue handles:
// - ToF for height from laser ranger
// - TDoA for position
// - TWR/distance for position
static xQueueHandle tofDataQueue;
static xQueueHandle tdoaDataQueue;
static xQueueHandle distDataQueue;

// Semaphores:
// - task semaphore to signal we can run the task loop
// - mutex to protect data shared between task and other modules
static SemaphoreHandle_t runTaskSemaphore;
static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;

// Check for initialization
static bool isInit = false;

// Reset position in case something went wrong
static bool resetEstimation = false;

// Bounds for position
static float maxPosition = 100.0f;

// Use height estimation from laser ranger in position estimation
static bool forceZ = false;


/**
 * Static function prototypes
 */

// Task
static void uwb2posTask(void* parameters);

// Supervisor: check if state within bounds
static bool uwb2posWithinBounds(void);

// General queue overwrite
// Overwrites item in queue
static bool overwriteMeasurement(xQueueHandle queue, void* measurement);
// General queue append
// Appends item to queue, meaning queue can get full
static bool appendMeasurement(xQueueHandle queue, void* measurement);

// Queue-specific getters
// ToF
static bool latestTofMeasurement(tofMeasurement_t* tof);
// TDoA
static bool latestTdoaMeasurement(tdoaMeasurement_t* tdoa);
// Distance
static bool latestDistanceMeasurement(distanceMeasurement_t* dist);

// Checks whether TDoA or distance measurements
static bool checkTdoa(tdoaMeasurement_t* tdoa);
static bool checkDist(distanceMeasurement_t* dist);


/**
 * Measurement queue memory allocation
 */

// ToF
STATIC_MEM_QUEUE_ALLOC(tofDataQueue, TOF_QUEUE_LENGTH, sizeof(tofMeasurement_t));
// TDoA
STATIC_MEM_QUEUE_ALLOC(tdoaDataQueue, TDOA_QUEUE_LENGTH, sizeof(tdoaMeasurement_t));
// Distance
STATIC_MEM_QUEUE_ALLOC(distDataQueue, DIST_QUEUE_LENGTH, sizeof(distanceMeasurement_t));


/**
 * Task memory allocation
 */

STATIC_MEM_TASK_ALLOC(uwb2posTask, UWB2POS_TASK_STACKSIZE);


/**
 * Statistics (updates per second)
 */

// Task loops
static STATS_CNT_RATE_DEFINE(loopCounter, ONE_SECOND);
// Height from laser ranger
static STATS_CNT_RATE_DEFINE(laserCounter, ONE_SECOND);
// Position from UWB measurements
static STATS_CNT_RATE_DEFINE(uwbCounter, ONE_SECOND);
// Position estimate externalized
static STATS_CNT_RATE_DEFINE(extCounter, ONE_SECOND);
// Appended measurements
static STATS_CNT_RATE_DEFINE(measurementAppendedCounter, ONE_SECOND);
// Non-appended measurements
static STATS_CNT_RATE_DEFINE(measurementNotAppendedCounter, ONE_SECOND);


/**
 * Estimator task functions
 */


// Init
void uwb2posTaskInit(void)
{
  // Create data queues for measurements
  tofDataQueue = STATIC_MEM_QUEUE_CREATE(tofDataQueue);
  tdoaDataQueue = STATIC_MEM_QUEUE_CREATE(tdoaDataQueue);
  distDataQueue = STATIC_MEM_QUEUE_CREATE(distDataQueue);

  // Create binary semaphore for task handling
  vSemaphoreCreateBinary(runTaskSemaphore);

  // Create mutex for sharing data
  dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);

  // Create task
  STATIC_MEM_TASK_CREATE(uwb2posTask, uwb2posTask, UWB2POS_TASK_NAME, NULL, UWB2POS_TASK_PRI);

  // Did we do the above?
  isInit = true;
}


// Trivial test that checks for task init
bool uwb2posTaskTest(void)
{
  return isInit;
}


// Main task function
static void uwb2posTask(void* parameters)
{
  // Wait for startup
  systemWaitStart();

  // Counters
  // Height estimation
  uint32_t lastLaser = xTaskGetTickCount();
  uint32_t nextLaser = xTaskGetTickCount();
  // Position estimation
  // Only next because we don't need dt here
  uint32_t nextUwb = xTaskGetTickCount();

  // Task loop
  while (true)
  {
    // Take semaphore
    xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);

    // Reset estimator if triggered
    if (resetEstimation)
    {
      uwb2posReset();
      resetEstimation = false;
    }

    // Trackers whether position has been updated
    // Either laser or UWB
    bool doneUpdate = false;

    // Current time
    uint32_t osTick = xTaskGetTickCount();

    // Estimate height from laser ranger
    /**
     * TODO: maybe we need a separate variable for the estimated height,
     * so we can switch between using / not using it (and keep it when we 
     * miss an update)
     */
    if (osTick >= nextLaser)
    {
      // Compute dt
      float dt = T2S(osTick - lastLaser);

      // Measurement placeholder
      tofMeasurement_t tof;

      // Check for measurement
      if (latestTofMeasurement(&tof))
      {
        // Update sensor data
        sensorsAcquire(&sensorSnapshot, osTick);

        // Estimate height
        /**
         * TODO: don't forget to call the update function
         * for vertical velocity in the attitude update of the state estimator
         */
        laserHeight(&inPosition, &sensorSnapshot, &tof, dt, osTick);

        // Use the estimated height for UWB position estimation
        forceZ = true;

        // Update counters
        lastLaser = osTick;
        doneUpdate = true;
        STATS_CNT_RATE_EVENT(&laserCounter);

        // Set tick of position estimate (gets overwritten by UWB position update)
        inPosition.timestamp = osTick;
      }
      else
      {
        // Reset forceZ
        forceZ = false;
      }

      // Next update
      nextLaser = osTick + S2T(1.0f / LASER_UPDATE_RATE);
    }

    // Estimate position from UWB measurements
    if (osTick >= nextUwb)
    {
      // Measurement placeholders
      tdoaMeasurement_t tdoa;
      distanceMeasurement_t dist;

      // Check for TDoA / distance measurements:
      // - only allocate resources for which is available
      // - do projection if not enough samples for multilateration
      // - raise error if both available
      /**
       * TODO: include check that ensures we have enough UNIQUE measurements
       * (3 range measurements from the same beacon don't get us anywhere...)
       */
      if (checkTdoa(&tdoa) && checkDist(&dist))
      {
        //DEBUG_PRINT("Both TDoA and distance measurements, doing nothing\n");
      }
      // TDoA
      else if (checkTdoa(&tdoa))
      {
        // Arrays to store measurements
        static float anchorAx[TDOA_QUEUE_LENGTH];
        static float anchorAy[TDOA_QUEUE_LENGTH];
        static float anchorAz[TDOA_QUEUE_LENGTH];
        static float anchorBx[TDOA_QUEUE_LENGTH];
        static float anchorBy[TDOA_QUEUE_LENGTH];
        static float anchorBz[TDOA_QUEUE_LENGTH];
        static float anchorDistDiff[TDOA_QUEUE_LENGTH];
        static uint32_t tdoaTimestamp[TDOA_QUEUE_LENGTH];

        // Start indices
        /**
         * TODO: where are new measurements appended to queue?
         * Does this have consequences for our queue receive?
         */
        static int tdoaStartIdx = TDOA_QUEUE_LENGTH - 1;
        // Sample counters
        static int tdoaSamples = 0;

        // Get latest TDoA measurements
        while(latestTdoaMeasurement(&tdoa))
        {
          // Put in array
          // Anchor A
          anchorAx[tdoaStartIdx] = tdoa.anchorPosition[0].x;
          anchorAy[tdoaStartIdx] = tdoa.anchorPosition[0].y;
          anchorAz[tdoaStartIdx] = tdoa.anchorPosition[0].z;
          // Anchor B
          anchorBx[tdoaStartIdx] = tdoa.anchorPosition[1].x;
          anchorBy[tdoaStartIdx] = tdoa.anchorPosition[1].y;
          anchorBz[tdoaStartIdx] = tdoa.anchorPosition[1].z;
          // Anchor distance difference
          anchorDistDiff[tdoaStartIdx] = tdoa.distanceDiff;
          // Timestamp (both have one, choose first)
          tdoaTimestamp[tdoaStartIdx] = tdoa.anchorPosition[0].timestamp;

          // Decrement / increment start index and counter
          tdoaStartIdx--;
          if (tdoaSamples < TDOA_QUEUE_LENGTH)
            tdoaSamples++;

          // Check index overflow (circular array)
          if (tdoaStartIdx < 0)
            tdoaStartIdx = TDOA_QUEUE_LENGTH - 1;
        }

        // Projection if not enough samples
        /**
         * TODO: for TDoA, measurements seem to be sparse in general,
         * so projection is more important here!
         */
        if (tdoaSamples > 0 && tdoaSamples < 5)
          uwbPosProjectTdoa(&inPosition, anchorAx, anchorAy, anchorAz, anchorBx, anchorBy, anchorBz, anchorDistDiff, tdoaStartIdx, tdoaSamples, TDOA_QUEUE_LENGTH, inPosition.z, forceZ);
        // Multilateration if enough
        else if (tdoaSamples >= 5)
          uwbPosMultilatTdoa(&inPosition, anchorAx, anchorAy, anchorAz, anchorBx, anchorBy, anchorBz, anchorDistDiff, tdoaTimestamp, tdoaStartIdx, tdoaSamples, TDOA_QUEUE_LENGTH, inPosition.z, forceZ);

        // Update counters
        doneUpdate = true;
        STATS_CNT_RATE_EVENT(&uwbCounter);

        // Set tick of position estimate
        inPosition.timestamp = osTick;
      }
      // Distance
      else if (checkDist(&dist))
      {
        // Arrays to store measurements
        static float anchorX[DIST_QUEUE_LENGTH];
        static float anchorY[DIST_QUEUE_LENGTH];
        static float anchorZ[DIST_QUEUE_LENGTH];
        static float anchorDist[DIST_QUEUE_LENGTH];
        static uint32_t distTimestamp[DIST_QUEUE_LENGTH];

        // Start indices
        /**
         * TODO: where are new measurements appended to queue?
         * Does this have consequences for our queue receive?
         */
        static int distStartIdx = DIST_QUEUE_LENGTH - 1;
        // Sample counters
        static int distSamples = 0;

        // Get latest distance measurements
        while(latestDistanceMeasurement(&dist))
        {
          // Put in array
          // Anchor position
          anchorX[distStartIdx] = dist.x;
          anchorY[distStartIdx] = dist.y;
          anchorZ[distStartIdx] = dist.z;
          // Anchor distance
          anchorDist[distStartIdx] = dist.distance;
          // Timestamp
          // Added to distanceMeasurement_t
          distTimestamp[distStartIdx] = dist.timestamp;

          // Decrement / increment start index and counter
          distStartIdx--;
          if (distSamples < DIST_QUEUE_LENGTH)
            distSamples++;

          // Check index overflow (circular array)
          if (distStartIdx < 0)
            distStartIdx = DIST_QUEUE_LENGTH - 1;
        }

        // Projection if not enough samples
        if (distSamples > 0 && distSamples < 5)
          uwbPosProjectTwr(&inPosition, anchorX, anchorY, anchorZ, anchorDist, distStartIdx, distSamples, DIST_QUEUE_LENGTH, inPosition.z, forceZ);
        // Multilateration if enough
        else if (distSamples >= 5)
          uwbPosMultilatTwr(&inPosition, anchorX, anchorY, anchorZ, anchorDist, distTimestamp, distStartIdx, distSamples, DIST_QUEUE_LENGTH, inPosition.z, forceZ);

        // Update counters
        doneUpdate = true;
        STATS_CNT_RATE_EVENT(&uwbCounter);

        // Set tick of position estimate
        inPosition.timestamp = osTick;
      }
      else
      {
        // Neither TDoA nor distance measurements
        // Do nothing
        //DEBUG_PRINT("Neither TDoA nor distance measurements, doing nothing\n");
      }

      // Next update
      nextUwb = osTick + S2T(1.0f / UWB_UPDATE_RATE);
    }

    // If an update has been made, we check the position for out-of-bounds
    if (doneUpdate)
    {
      if (!uwb2posWithinBounds())
      {
        resetEstimation = true;
        DEBUG_PRINT("Position estimate out of bounds, resetting\n");
      }

      // If all is fine, copy internal position to external position
      // Mutex needed because exPosition can be read from elsewhere
      // Shallow copy is fine
      /**
       * TODO: tick of inPosition (and thus exPosition) is set during UWB
       * estimation; is this correct?
       */
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      exPosition = inPosition;
      // exPosition.timestamp = osTick;
      xSemaphoreGive(dataMutex);
    }

    // Counter for loops
    STATS_CNT_RATE_EVENT(&loopCounter);
  }
}


/**
 * Interface functions to state estimators
 */


// Externalize position estimate:
// - externalPosition is the position in the state estimator
// - exPosition is intermediate value between inPosition and externalPosition
/**
 * TODO: we don't change tick here
 */
bool uwb2posExternalize(point_t* externalPosition)
{
  // Acquire mutex
  xSemaphoreTake(dataMutex, portMAX_DELAY);

  // memcpy because we only know addresses
  // dest, src
  memcpy(externalPosition, &exPosition, sizeof(point_t));

  // Release mutex
  xSemaphoreGive(dataMutex);

  // Counter for externalizations
  STATS_CNT_RATE_EVENT(&extCounter);

  // Success
  return true;
}


// Call task from stabilizer loop
// void uwb2posCall(const point_t* estPosition)
void uwb2posCall(void)
{
  // // Get position from state estimation as previous position
  // xSemaphoreTake(dataMutex, portMAX_DELAY);
  // // dest, src
  // memcpy(&prevPosition, estPosition, sizeof(point_t));
  // xSemaphoreGive(dataMutex);

  // Give semaphore such that this task can run
  xSemaphoreGive(runTaskSemaphore);
}


/**
 * Estimator task subfunctions
 */


// Reset
void uwb2posReset(void)
{
  // Reset queues
  xQueueReset(tofDataQueue);
  xQueueReset(tdoaDataQueue);
  xQueueReset(distDataQueue);

  // Reset position estimates
  // Shallow copy is fine
  inPosition = zeroPosition;
  exPosition = zeroPosition;

  // Reset forcing Z
  forceZ = false;
}


// Supervisor: check if state within bounds
static bool uwb2posWithinBounds(void)
{
  // Check whether internal position is within bounds
  if (inPosition.x > maxPosition) return false;
  if (inPosition.y > maxPosition) return false;
  if (inPosition.z > maxPosition) return false;

  // Check whether external position is within bounds
  if (exPosition.x > maxPosition) return false;
  if (exPosition.y > maxPosition) return false;
  if (exPosition.z > maxPosition) return false;

  // All is well!
  return true;
}


/**
 * Measurement queue functions
 * - overwrite and peek for ToF
 * - append and receive for TDoA / distance
 */


// General measurement overwrite
// Overwrites item in queue
static bool overwriteMeasurement(xQueueHandle queue, void* measurement)
{
  portBASE_TYPE result;
  bool isInInterrupt = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;

  if (isInInterrupt)
  {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    result = xQueueOverwriteFromISR(queue, measurement, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE)
      portYIELD();
  }
  else
  {
    result = xQueueOverwrite(queue, measurement);
  }

  return (result == pdTRUE);
}


// General measurement append
// Appends item to queue, meaning queue can get full
/**
 * TODO: on which side of the queue are items appended?
 */
static bool appendMeasurement(xQueueHandle queue, void* measurement)
{
  portBASE_TYPE result;
  bool isInInterrupt = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;

  if (isInInterrupt)
  {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    result = xQueueSendFromISR(queue, measurement, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE)
      portYIELD();
  }
  else
  {
    result = xQueueSend(queue, measurement, 0);
  }

  if (result == pdTRUE)
  {
    STATS_CNT_RATE_EVENT(&measurementAppendedCounter);
    return true;
  }
  else
  {
    STATS_CNT_RATE_EVENT(&measurementNotAppendedCounter);
    return true;
  }
}


// ToF queue peek = only get
static bool latestTofMeasurement(tofMeasurement_t* tof)
{
  return (xQueuePeek(tofDataQueue, tof, 0) == pdTRUE);
}


// TDoA queue peek as check
static bool checkTdoa(tdoaMeasurement_t* tdoa)
{
  return (xQueuePeek(tdoaDataQueue, tdoa, 0) == pdTRUE);
}


// Distance queue peek as check
static bool checkDist(distanceMeasurement_t* dist)
{
  return (xQueuePeek(distDataQueue, dist, 0) == pdTRUE);
}


// TDoA queue receive = get + delete
/**
 * TODO: does this get the first or last item of the queue?
 */
static bool latestTdoaMeasurement(tdoaMeasurement_t* tdoa)
{
  return (xQueueReceive(tdoaDataQueue, tdoa, 0) == pdTRUE);
}


// Distance queue receive = get + delete
/**
 * TODO: does this get the first or last item of the queue?
 */
static bool latestDistanceMeasurement(distanceMeasurement_t* dist)
{
  return (xQueueReceive(distDataQueue, dist, 0) == pdTRUE);
}


// ToF overwrite measurement in queue
bool uwb2posEnqueueTOF(const tofMeasurement_t* tof)
{
  // A distance to the ground along the body axis
  return overwriteMeasurement(tofDataQueue, (void*) tof);
}


// TDoA add measurement to queue
bool uwb2posEnqueueTDOA(const tdoaMeasurement_t* tdoa)
{
  // A distance difference to two UWB anchors
  return appendMeasurement(tdoaDataQueue, (void*) tdoa);
}


// Distance add measurement to queue
bool uwb2posEnqueueDistance(const distanceMeasurement_t* dist)
{
  // A distance to a single UWB anchor
  return appendMeasurement(distDataQueue, (void*) dist);
}


/**
 * Logging and adjustable parameters
 */

// Stock group
LOG_GROUP_START(UWB2POS)
  // Estimated position: internal and external
  LOG_ADD(LOG_FLOAT, inX, &inPosition.x)
  LOG_ADD(LOG_FLOAT, inY, &inPosition.y)
  LOG_ADD(LOG_FLOAT, inZ, &inPosition.z)
  LOG_ADD(LOG_FLOAT, exX, &exPosition.x)
  LOG_ADD(LOG_FLOAT, exY, &exPosition.y)
  LOG_ADD(LOG_FLOAT, exZ, &exPosition.z)

  // Statistics
  STATS_CNT_RATE_LOG_ADD(rtLoop, &loopCounter)
  STATS_CNT_RATE_LOG_ADD(rtLaser, &laserCounter)
  STATS_CNT_RATE_LOG_ADD(rtUwb, &uwbCounter)
  STATS_CNT_RATE_LOG_ADD(rtExt, &extCounter)
  STATS_CNT_RATE_LOG_ADD(rtApnd, &measurementAppendedCounter)
  STATS_CNT_RATE_LOG_ADD(rtRej, &measurementNotAppendedCounter)
LOG_GROUP_STOP(UWB2POS)

// Parameters
PARAM_GROUP_START(UWB2POS)
  PARAM_ADD(PARAM_UINT8, resetEstimation, &resetEstimation)
  PARAM_ADD(PARAM_FLOAT, maxPosition, &maxPosition)
  PARAM_ADD(PARAM_UINT8, forceZ, &forceZ)
PARAM_GROUP_STOP(UWB2POS)
