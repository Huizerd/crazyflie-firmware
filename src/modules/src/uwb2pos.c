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
 */

// Update rates
// Height from laser ranger
#define LASER_UPDATE_RATE RATE_100_HZ
// Position from UWB measurements
#define UWB_UPDATE_RATE RATE_10_HZ

// Measurement queue lengths
#define TOF_QUEUE_LENGTH (1)
#define TDOA_QUEUE_LENGTH (20)
#define DIST_QUEUE_LENGTH (20)
#define POS_QUEUE_LENGTH (1)

// One second in ticks
#define ONE_SECOND (1000)


/**
 * File statics
 */

// Position estimated from UWB / laser measurements:
// - one for internal use / appending to queue when correct
// - one for resetting (that we don't change)
static point_t estPosition;
static const point_t zeroPosition;

// Snapshots:
// - of sensor data for use in height estimation (barometer only)
// - of latest laser height in case averaging (below) fails
static sensorData_t sensorSnapshot;
static float laserHeightSnapshot = 0.0f;

// Accumulator + counter for laser height averaging
static float laserHeightAccumulator = 0.0f;
static int laserHeightCount = 0;

// Optional exponentially weighted moving average (chosen because ridiculously simple & no buffer)
static float laserHeightEWMA = 0.0f;
static float alphaEWMA = 0.99f;  // higher = older observations discounted faster
static bool useLaserEWMA = false;

// Measurement queue handles:
// - ToF for height from laser ranger
// - TDoA for position
// - TWR/distance for position
// - estimated position (to be received by other tasks)
static xQueueHandle tofDataQueue;
static xQueueHandle tdoaDataQueue;
static xQueueHandle distDataQueue;
static xQueueHandle posDataQueue;

// Semaphores:
// - task semaphore to signal we can run the task loop
static SemaphoreHandle_t runTaskSemaphore;

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

// Enqueue for position estimate
static bool uwb2posEnqueuePos(const point_t* pos);


/**
 * Measurement queue memory allocation
 */

// ToF
STATIC_MEM_QUEUE_ALLOC(tofDataQueue, TOF_QUEUE_LENGTH, sizeof(tofMeasurement_t));
// TDoA
STATIC_MEM_QUEUE_ALLOC(tdoaDataQueue, TDOA_QUEUE_LENGTH, sizeof(tdoaMeasurement_t));
// Distance
STATIC_MEM_QUEUE_ALLOC(distDataQueue, DIST_QUEUE_LENGTH, sizeof(distanceMeasurement_t));
// Estimated positions
STATIC_MEM_QUEUE_ALLOC(posDataQueue, POS_QUEUE_LENGTH, sizeof(point_t));


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
// Position estimate updated
static STATS_CNT_RATE_DEFINE(updateCounter, ONE_SECOND);
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
  posDataQueue = STATIC_MEM_QUEUE_CREATE(posDataQueue);

  // Create binary semaphore for task handling
  vSemaphoreCreateBinary(runTaskSemaphore);

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

    // Tracks whether position has been updated
    bool doneUpdate = false;

    // Current time
    uint32_t osTick = xTaskGetTickCount();

    // Estimate height from laser ranger
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

        // Add height estimate to accumulator
        laserHeightSnapshot = laserHeight(&sensorSnapshot, &tof, dt, osTick);
        laserHeightAccumulator += laserHeightSnapshot;
        laserHeightCount++;

        // Update EWMA for laser height
        if (laserHeightEWMA == 0.0f)
          laserHeightEWMA += laserHeightSnapshot;
        else
          laserHeightEWMA = laserHeightSnapshot * alphaEWMA + (1.0f - alphaEWMA) * laserHeightEWMA;

        // Use the estimated height for UWB position estimation
        forceZ = true;

        // Update counters
        lastLaser = osTick;
        STATS_CNT_RATE_EVENT(&laserCounter);
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

      // EWMA for laser height or not
      /**
       * TODO: add this
       */

      // Get averaged laser height estimation
      // Needs check for nan
      if (laserHeightCount > 0)
      {
        estPosition.z = laserHeightAccumulator / (float) laserHeightCount;
        laserHeightAccumulator = 0.0f;
        laserHeightCount = 0;
      }
      else
      {
        estPosition.z = laserHeightSnapshot;
      }

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
        DEBUG_PRINT("Both TDoA and distance measurements, doing nothing\n");
      }
      // TDoA
      else if (checkTdoa(&tdoa))
      {
        // Arrays to store measurements
        /**
         * TODO: make these global, such that they can be reset
         */
        static float anchorAx[TDOA_QUEUE_LENGTH];
        static float anchorAy[TDOA_QUEUE_LENGTH];
        static float anchorAz[TDOA_QUEUE_LENGTH];
        static float anchorBx[TDOA_QUEUE_LENGTH];
        static float anchorBy[TDOA_QUEUE_LENGTH];
        static float anchorBz[TDOA_QUEUE_LENGTH];
        static float anchorDistDiff[TDOA_QUEUE_LENGTH];
        static uint32_t tdoaTimestamp[TDOA_QUEUE_LENGTH];

        // Start indices
        static int tdoaStartIdx = TDOA_QUEUE_LENGTH;
        // Sample counters
        static int tdoaSamples = 0;

        // Get latest TDoA measurements
        while(latestTdoaMeasurement(&tdoa))
        {
          // Decrement start index and check overflow (circular array)
          tdoaStartIdx--;
          if (tdoaStartIdx < 0)
            tdoaStartIdx = TDOA_QUEUE_LENGTH - 1;

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

          // Increment counter
          if (tdoaSamples < TDOA_QUEUE_LENGTH)
            tdoaSamples++;
        }

        // Projection if not enough samples
        /**
         * TODO: for TDoA, measurements seem to be sparse in general,
         * so projection is more important here!
         */
        if (tdoaSamples > 0 && tdoaSamples < 5)
          uwbPosProjectTdoa(&estPosition, anchorAx, anchorAy, anchorAz, anchorBx, anchorBy, anchorBz, anchorDistDiff, tdoaStartIdx, tdoaSamples, TDOA_QUEUE_LENGTH, estPosition.z, forceZ);
        // Multilateration if enough
        else if (tdoaSamples >= 5)
          uwbPosMultilatTdoa(&estPosition, anchorAx, anchorAy, anchorAz, anchorBx, anchorBy, anchorBz, anchorDistDiff, tdoaTimestamp, tdoaStartIdx, tdoaSamples, TDOA_QUEUE_LENGTH, estPosition.z, forceZ);

        // Update counters
        doneUpdate = true;
        STATS_CNT_RATE_EVENT(&uwbCounter);

        // Set tick of position estimate
        estPosition.timestamp = osTick;
      }
      // Distance
      else if (checkDist(&dist))
      {
        // Arrays to store measurements
        /**
         * TODO: make these global, such that they can be reset
         */
        static float anchorX[DIST_QUEUE_LENGTH];
        static float anchorY[DIST_QUEUE_LENGTH];
        static float anchorZ[DIST_QUEUE_LENGTH];
        static float anchorDist[DIST_QUEUE_LENGTH];
        static uint32_t distTimestamp[DIST_QUEUE_LENGTH];

        // Start indices
        static int distStartIdx = DIST_QUEUE_LENGTH;
        // Sample counters
        static int distSamples = 0;

        // Get latest distance measurements
        while(latestDistanceMeasurement(&dist))
        {
          // Decrement start index and check overflow (circular array)
          distStartIdx--;
          if (distStartIdx < 0)
            distStartIdx = DIST_QUEUE_LENGTH - 1;

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

          // Increment counter
          if (distSamples < DIST_QUEUE_LENGTH)
            distSamples++;
        }

        // Projection if not enough samples
        if (distSamples > 0 && distSamples < 5)
          uwbPosProjectTwr(&estPosition, anchorX, anchorY, anchorZ, anchorDist, distStartIdx, distSamples, DIST_QUEUE_LENGTH, estPosition.z, forceZ);
        // Multilateration if enough
        else if (distSamples >= 5)
          uwbPosMultilatTwr(&estPosition, anchorX, anchorY, anchorZ, anchorDist, distTimestamp, distStartIdx, distSamples, DIST_QUEUE_LENGTH, estPosition.z, forceZ);

        // Update counters
        doneUpdate = true;
        STATS_CNT_RATE_EVENT(&uwbCounter);

        // Set tick of position estimate
        estPosition.timestamp = osTick;
      }
      else
      {
        // Neither TDoA nor distance measurements
        // Do nothing
        DEBUG_PRINT("Neither TDoA nor distance measurements, doing nothing\n");
      }

      // Next update
      nextUwb = osTick + S2T(1.0f / UWB_UPDATE_RATE);
    }

    // If an update has been made, we check the position for out-of-bounds
    // Only UWB updates count, laser is updated much more often
    // and assumed to be always up-to-date
    if (doneUpdate)
    {
      if (uwb2posWithinBounds())
      {
        // If all is fine, add position estimate to queue
        uwb2posEnqueuePos(&estPosition);

        // Counter for updates
        STATS_CNT_RATE_EVENT(&updateCounter);
      }
      else
      {
        resetEstimation = true;
        DEBUG_PRINT("Position estimate out of bounds, resetting\n");
      }
    }

    // Counter for loops
    STATS_CNT_RATE_EVENT(&loopCounter);
  }
}


/**
 * Interface functions to state estimators
 */


// Call task from stabilizer loop
// void uwb2posCall(const point_t* estPosition)
void uwb2posCall(void)
{
  // Give semaphore such that this task can run
  xSemaphoreGive(runTaskSemaphore);
}


// Position queue receive = get + delete
bool latestPosMeasurement(point_t* pos)
{
  return (xQueueReceive(posDataQueue, pos, 0) == pdTRUE);
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
  xQueueReset(posDataQueue);

  // Reset position estimate
  // Shallow copy is fine
  estPosition = zeroPosition;

  // Reset snapshots
  laserHeightSnapshot = 0.0f;

  // Reset accumulators / counts
  laserHeightAccumulator = 0.0f;
  laserHeightCount = 0;

  // Reset EWMA
  laserHeightEWMA = 0.0f;

  // Reset forcing Z
  forceZ = false;
}


// Supervisor: check if state within bounds
static bool uwb2posWithinBounds(void)
{
  if (estPosition.x > maxPosition) return false;
  if (estPosition.y > maxPosition) return false;
  if (estPosition.z > maxPosition) return false;

  // All is well!
  return true;
}


/**
 * Measurement queue functions
 * - overwrite and peek for ToF
 * - append and receive for TDoA / distance / position estimates
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


// TDoA queue receive = get + delete
static bool latestTdoaMeasurement(tdoaMeasurement_t* tdoa)
{
  return (xQueueReceive(tdoaDataQueue, tdoa, 0) == pdTRUE);
}


// Distance queue receive = get + delete
static bool latestDistanceMeasurement(distanceMeasurement_t* dist)
{
  return (xQueueReceive(distDataQueue, dist, 0) == pdTRUE);
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


// Estimated position add measurement to queue
// static because only used here
static bool uwb2posEnqueuePos(const point_t* pos)
{
  // A position (X, Y, Z) estimate
  return appendMeasurement(posDataQueue, (void*) pos);
}


/**
 * Logging and adjustable parameters
 */

// Stock group
LOG_GROUP_START(UWB2POS)
  // Estimated position
  LOG_ADD(LOG_FLOAT, estX, &estPosition.x)
  LOG_ADD(LOG_FLOAT, estY, &estPosition.y)
  LOG_ADD(LOG_FLOAT, estZ, &estPosition.z)
  LOG_ADD(LOG_FLOAT, laserHeightEWMA, &laserHeightEWMA)

  // Statistics
  STATS_CNT_RATE_LOG_ADD(rtLoop, &loopCounter)
  STATS_CNT_RATE_LOG_ADD(rtLaser, &laserCounter)
  STATS_CNT_RATE_LOG_ADD(rtUwb, &uwbCounter)
  STATS_CNT_RATE_LOG_ADD(rtUpdate, &updateCounter)
  STATS_CNT_RATE_LOG_ADD(rtApnd, &measurementAppendedCounter)
  STATS_CNT_RATE_LOG_ADD(rtRej, &measurementNotAppendedCounter)
LOG_GROUP_STOP(UWB2POS)

// Parameters
PARAM_GROUP_START(UWB2POS)
  PARAM_ADD(PARAM_UINT8, resetEstimation, &resetEstimation)
  PARAM_ADD(PARAM_FLOAT, maxPosition, &maxPosition)
  PARAM_ADD(PARAM_UINT8, forceZ, &forceZ)
  PARAM_ADD(PARAM_UINT8, useLaserEWMA, &useLaserEWMA)
  PARAM_ADD(PARAM_FLOAT, alphaEWMA, &alphaEWMA)
PARAM_GROUP_STOP(UWB2POS)
