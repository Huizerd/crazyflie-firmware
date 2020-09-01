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
 * uwb2pos_core.c - Core functions for position estimation
 * from UWB / laser measurements
 */


// Own
#include "uwb2pos_core.h"

// Chip
#include "stm32f4xx.h"
// Real-time OS
#include "FreeRTOS.h"
// Task ticks
#include "task.h"

// 3-element vector math
#include "math3d.h"
// Arm matrices
#include "cf_math.h"
// Deadband
#include "num.h"

// Logging
#include "log.h"
// Adjustable parameters
#include "param.h"

// Debugging
#define DEBUG_MODULE "UWB2POS_CORE"
#include "debug.h"


/**
 * File constants
 */

// Maximum sample ages (in milliseconds)
// Additional check to those in uwb2pos.c
#define LASER_AGE_MS (50)
#define TDOA_AGE_MS (200)
#define TWR_AGE_MS (200)

// Gravity
#define GRAV (9.81f)

// Settings for laser height estimation
laserStruct_t laserStruct = {
  .estimatedZ = 0.0f,
  .velocityZ = 0.0f,
  .estAlphaZrange = 0.0f,
  .estAlphaAsl = 0.997f,
  .velocityFactor = 1.0f,
  .vAccDeadband = 0.04f,
  .velZAlpha = 0.995f,
};


/**
 * File statics
 */

// Logging:
// - estimated position
// - separate estimated height
static point_t logPosition;
static float logHeight;


/**
 * Static function prototypes
 */

// Check for position / height being NaN
static bool stateNotNaN(void);

// Matrix rank function (returns actual rank instead of success)
static int mat_rank(const arm_matrix_instance_f32* pA);

// Matrix pseudo-inverse function (called below)
static arm_status arm_mat_pinverse_f32(const arm_matrix_instance_f32* pSrc, arm_matrix_instance_f32* pDest);

// Fast arm sqrt
static inline float arm_sqrt(float32_t in)
{ float pOut = 0; if (ARM_MATH_SUCCESS != arm_sqrt_f32(in, &pOut)) {DEBUG_PRINT("Square root failed\n");} return pOut; }

// Matrix functions: transpose, inverse, pseudo-inverse, multiplication, addition, subtraction
static inline void mat_trans(const arm_matrix_instance_f32* pSrc, arm_matrix_instance_f32* pDst)
{ ASSERT(ARM_MATH_SUCCESS == arm_mat_trans_f32(pSrc, pDst)); }
static inline void mat_inv(const arm_matrix_instance_f32* pSrc, arm_matrix_instance_f32* pDst)
{ if (ARM_MATH_SUCCESS != arm_mat_inverse_f32(pSrc, pDst)) {DEBUG_PRINT("Matrix inverse failed\n");} }
static inline void mat_pinv(const arm_matrix_instance_f32* pSrc, arm_matrix_instance_f32* pDst)
{ ASSERT(ARM_MATH_SUCCESS == arm_mat_pinverse_f32(pSrc, pDst)); }
static inline void mat_mult(const arm_matrix_instance_f32* pSrcA, const arm_matrix_instance_f32* pSrcB, arm_matrix_instance_f32* pDst)
{ ASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst)); }
static inline void mat_add(const arm_matrix_instance_f32* pSrcA, const arm_matrix_instance_f32* pSrcB, arm_matrix_instance_f32* pDst)
{ ASSERT(ARM_MATH_SUCCESS == arm_mat_add_f32(pSrcA, pSrcB, pDst)); }
static inline void mat_sub(const arm_matrix_instance_f32* pSrcA, const arm_matrix_instance_f32* pSrcB, arm_matrix_instance_f32* pDst)
{ ASSERT(ARM_MATH_SUCCESS == arm_mat_sub_f32(pSrcA, pSrcB, pDst)); }


/**
 * Height estimation
 */

// Laser ranger
float laserHeight(const sensorData_t* sensorData, const tofMeasurement_t* tof, float dt, uint32_t tick)
{
  // Filtered Z
  float filteredZ;
  // Use laser ranger for surface following
  static bool surfaceFollowingMode = false;

  // Max sample age
  /**
   * TODO: why don't we use the given tick?
   */
  const uint32_t MAX_SAMPLE_AGE = M2T(LASER_AGE_MS);
  uint32_t now = xTaskGetTickCount();
  // Check for sample usefulness (age)
  bool isSampleUseful = ((now - tof->timestamp) <= MAX_SAMPLE_AGE);

  // Only use laser ranger if sample is useful
  if (isSampleUseful)
    surfaceFollowingMode = true;

  // Functional part:
  // - if sample has been useful and still is: first block
  // - else if sample has been useful but isn't anymore: do nothing
  // - else if sample was never useful: init with asl
  if (surfaceFollowingMode)
  {
    if (isSampleUseful)
    {
      // IIR filter Zrange
      filteredZ = laserStruct.estAlphaZrange * laserStruct.estimatedZ + (1.0f - laserStruct.estAlphaZrange) * tof->distance;
      // Use Zrange as base and add velocity changes
      laserStruct.estimatedZ = filteredZ + laserStruct.velocityFactor * laserStruct.velocityZ * dt;

    }
  }
  else
  {
    // Hack to init IIR filter
    /**
     * TODO: baro as alternative, more robust way of
     * determining height
     */
    if (laserStruct.estimatedZ == 0.0f)
      filteredZ = sensorData->baro.asl;
    else
      // IIR filter asl
      filteredZ = laserStruct.estAlphaAsl * laserStruct.estimatedZ + (1.0f - laserStruct.estAlphaAsl) * sensorData->baro.asl;

    // Use asl as base and add velocity changes
    laserStruct.estimatedZ = filteredZ + laserStruct.velocityFactor * laserStruct.velocityZ * dt;
  }

  // Write to logging variable
  logHeight = laserStruct.estimatedZ;

  // Check for NaNs
  ASSERT(stateNotNaN());

  // Return estimated height for averaging
  return laserStruct.estimatedZ;
}


// Update velocity
void laserVelocity(float accWZ, float dt)
{
  // Add deadband
  laserStruct.velocityZ += deadband(accWZ, laserStruct.vAccDeadband) * dt * GRAV;
  // Correct?
  laserStruct.velocityZ *= laserStruct.velZAlpha;
}


/**
 * Position estimation
 */

// TDoA projection
void uwbPosProjectTdoa(point_t* position, const float* anchorAx, const float* anchorAy, const float* anchorAz, const float* anchorBx, const float* anchorBy, const float* anchorBz, const float* anchorDistDiff, int startIndex, int samples, int queueSize, float forcedZ, bool forceZ)
{
  /**
   * TODO: make this
   */
  // // Forced Z
  // if (forceZ)
  // {
  //   // Doesn't work properly yet!
  // }
  // // No forced Z
  // else
  // {
  //   // Previous estimate of position
  //   float pos[4] = {position->x, position->y, position->z, 0.0f};
  // }

  // Check for NaNs
  ASSERT(stateNotNaN());
}


// TDoA multilateration
/**
 * TODO: case where we don't force Z is not working properly yet!
 */
void uwbPosMultilatTdoa(point_t* position, float* anchorAx, float* anchorAy, float* anchorAz, float* anchorBx, float* anchorBy, float* anchorBz, float* anchorDistDiff, uint32_t* anchorTimestamp, int startIndex, int samples, int queueSize, float forcedZ, bool forceZ)
{
  // Previous estimate of position
  /**
   * TODO: use arm vectors here? (see e.g. arm_add_f32)
   */
  struct vec pos = mkvec(position->x, position->y, position->z);

  // Overwrite Z if forced
  if (forceZ)
    pos.z = forcedZ;

  // Checks
  ASSERT(startIndex < queueSize);
  ASSERT(samples <= queueSize);

  // Vectors and matrices
  struct vec dA;
  struct vec dB;
  float S[samples];
  float J2[samples * 2];
  float J3[samples * 3];

  // Init arm matrices
  arm_matrix_instance_f32 Sm = {samples, 1, S};
  arm_matrix_instance_f32 J2m = {samples, 2, J2};
  arm_matrix_instance_f32 J3m = {samples, 3, J3};

  // Valid matrix based on rank
  bool validJ;

  // Perform Gauss-Newton
  while (true)
  {
    // Go over samples
    for (int i = 0; i < samples; i++)
    {
      // Compute distance difference from prior position to anchors
      dA = vsub(pos, mkvec(anchorAx[(startIndex + i) % queueSize], anchorAy[(startIndex + i) % queueSize], anchorAz[(startIndex + i) % queueSize]));
      dB = vsub(pos, mkvec(anchorBx[(startIndex + i) % queueSize], anchorBy[(startIndex + i) % queueSize], anchorBz[(startIndex + i) % queueSize]));
      // Error with given distance difference
      S[i] = vmag(dB) - vmag(dA) - anchorDistDiff[(startIndex + i) % queueSize];

      // Forced Z
      if (forceZ)
      {
        // Jacobian
        J2[i * 2] = dB.x / vmag(dB)  - dA.x / vmag(dA);
        J2[i * 2 + 1] = dB.y / vmag(dB)  - dA.y / vmag(dA);
        validJ = (mat_rank(&J2m) >= 2);
      }
      // No forced Z
      else
      {
        // Jacobian
        J3[i * 3] = dB.x / vmag(dB)  - dA.x / vmag(dA);
        J3[i * 3 + 1] = dB.y / vmag(dB)  - dA.y / vmag(dA);
        J3[i * 3 + 2] = dB.z / vmag(dB)  - dA.z / vmag(dA);
        validJ = (mat_rank(&J3m) >= 3);
      }
    }

    // Break if not valid
    if (!validJ)
      break;
    
    // Forced Z
    if (forceZ)
    {
      // Matrices for inverse
      float Pinv[2 * samples];
      float Delta[2];
      arm_matrix_instance_f32 Pinvm = {2, samples, Pinv};
      arm_matrix_instance_f32 Deltam = {2, 1, Delta};

      // Do inverse
      mat_pinv(&J2m, &Pinvm);

      // Get update
      mat_mult(&Pinvm, &Sm, &Deltam);

      // Do update
      pos.x -= Delta[0];
      pos.y -= Delta[1];

      // Break if enough accuracy
      if (vmag(mkvec(Delta[0], Delta[1], 0.0f)) < 0.001f)
        break;
    }
    // No forced Z
    else
    {
      // Matrices for inverse
      float Pinv[3 * samples];
      float Delta[3];
      arm_matrix_instance_f32 Pinvm = {3, samples, Pinv};
      arm_matrix_instance_f32 Deltam = {3, 1, Delta};

      // Do inverse
      mat_pinv(&J3m, &Pinvm);

      // Get update
      mat_mult(&Pinvm, &Sm, &Deltam);

      // Do update
      pos.x -= Delta[0];
      pos.y -= Delta[1];
      pos.z -= Delta[2];

      // Break if enough accuracy
      if (vmag(mkvec(Delta[0], Delta[1], Delta[2])) < 0.001f)
        break;
    }
  }

  // Write position
  position->x = pos.x;
  position->y = pos.y;
  position->z = pos.z;

  // Write to logging variables
  logPosition.x = pos.x;
  logPosition.y = pos.y;
  logPosition.z = pos.z;

  // Check for NaNs
  ASSERT(stateNotNaN());
}


// TWR projection
void uwbPosProjectTwr(point_t* position, const float* anchorX, const float* anchorY, const float* anchorZ, const float* anchorDistance, int startIndex, int samples, int queueSize, float forcedZ, bool forceZ)
{
  // Previous estimate of position
  /**
   * TODO: use arm vectors here? (see e.g. arm_add_f32)
   */
  struct vec prior = mkvec(position->x, position->y, position->z);
  struct vec prior2d = mkvec(position->x, position->y, 0.0f);
  // Output estimate of position
  struct vec pos = mkvec(0.0f, 0.0f, 0.0f);
  // Anchor position
  struct vec anchor;
  struct vec anchor2d;
  // Direction
  struct vec dir;
  // Distance 2d
  float dist2d;

  // Checks
  ASSERT(startIndex < queueSize);
  ASSERT(samples <= queueSize);

  // Go over measurements
  for (int i = 0; i < samples; i++)
  {
    // We are forcing an external Z
    if (forceZ)
    {
      // 2D anchor and distance
      anchor2d = mkvec(anchorX[(startIndex + i) % queueSize], anchorY[(startIndex + i) % queueSize], 0.0f);
      dist2d = arm_sqrt(anchorDistance[(startIndex + i) % queueSize] * anchorDistance[(startIndex + i) % queueSize] - (forcedZ - anchorZ[(startIndex + i) % queueSize]) * (forcedZ - anchorZ[(startIndex + i) % queueSize]));

      // Check for prior == anchor (leads to NaN)
      if (veq(prior2d, anchor2d))
        // Along X
        dir = mkvec(1.0f, 0.0f, 0.0f);
      else
        dir = vdiv(vsub(prior2d, anchor2d), vmag(vsub(prior2d, anchor2d)));

      // Compute position
      pos = vadd(pos, vscl(1.0f / (float) samples, vadd(anchor2d, vscl(dist2d, dir))));
      pos.z += 1.0f / (float) samples * forcedZ;
    }
    // We are not forcing an external Z
    else
    {
      // Anchor location
      anchor = mkvec(anchorX[(startIndex + i) % queueSize], anchorY[(startIndex + i) % queueSize], anchorZ[(startIndex + i) % queueSize]);

      // Check for prior == anchor (leads to NaN)
      if (veq(prior, anchor))
        // Along X
        dir = mkvec(1.0f, 0.0f, 0.0f);
      else
        dir = vdiv(vsub(prior, anchor), vmag(vsub(prior, anchor)));

      // Compute position
      pos = vadd(pos, vscl(1.0f / (float) samples, vadd(anchor, vscl(anchorDistance[i], dir))));
    }
  }

  // Write position
  position->x = pos.x;
  position->y = pos.y;
  position->z = pos.z;

  // Write to logging variables
  logPosition.x = pos.x;
  logPosition.y = pos.y;
  logPosition.z = pos.z;

  // Check for NaNs
  ASSERT(stateNotNaN());
}


// TWR multilateration
void uwbPosMultilatTwr(point_t* position, float* anchorX, float* anchorY, float* anchorZ, float* anchorDistance, uint32_t* anchorTimestamp, int startIndex, int samples, int queueSize, float forcedZ, bool forceZ)
{
  // Checks
  ASSERT(startIndex < queueSize);
  ASSERT(samples <= queueSize);

  // We are forcing an external Z
  if (forceZ)
  {
    // Combine matrices into A matrix
    float A[samples * 3];
    arm_matrix_instance_f32 Am = {samples, 3, A};

    for (int i = 0; i < samples; i++)
    {
      A[i * 3] = 1.0f;
      A[i * 3 + 1] = -2.0f * anchorX[(startIndex + i) % queueSize];
      A[i * 3 + 2] = -2.0f * anchorY[(startIndex + i) % queueSize];
    }

    // Combute b matrix
    float b[samples];
    arm_matrix_instance_f32 bm = {samples, 1, b};

    for (int i = 0; i < samples; i++)
      b[i] = anchorDistance[(startIndex + i) % queueSize] * anchorDistance[(startIndex + i) % queueSize] - anchorX[(startIndex + i) % queueSize] * anchorX[(startIndex + i) % queueSize] - anchorY[(startIndex + i) % queueSize] * anchorY[(startIndex + i) % queueSize] - (anchorZ[(startIndex + i) % queueSize] - forcedZ) * (anchorZ[(startIndex + i) % queueSize] -forcedZ);

    // Calculate position
    // Prepare matrices
    float Pinv[3 * samples];  // pseudo-inverse
    float Pos[3];             // pseudo-inverse * b
    arm_matrix_instance_f32 Pinvm = {3, samples, Pinv};
    arm_matrix_instance_f32 Posm = {3, 1, Pos};

    // Get pseudo-inverse
    mat_pinv(&Am, &Pinvm);

    // Get position
    mat_mult(&Pinvm, &bm, &Posm);

    // Write position
    position->x = Pos[1];
    position->y = Pos[2];
    position->z = forcedZ;

    // Write to logging variables
    logPosition.x = Pos[1];
    logPosition.y = Pos[2];
    logPosition.z = forcedZ;
  }
  // We are not forcing an external Z
  else
  {
    // Combine matrices into A matrix
    float A[samples * 4];
    arm_matrix_instance_f32 Am = {samples, 4, A};

    for (int i = 0; i < samples; i++)
    {
      A[i * 4] = 1.0f;
      A[i * 4 + 1] = -2.0f * anchorX[(startIndex + i) % queueSize];
      A[i * 4 + 2] = -2.0f * anchorY[(startIndex + i) % queueSize];
      A[i * 4 + 3] = -2.0f * anchorZ[(startIndex + i) % queueSize];
    }

    // Combute b matrix
    float b[samples];
    arm_matrix_instance_f32 bm = {samples, 1, b};

    for (int i = 0; i < samples; i++)
      b[i] = anchorDistance[(startIndex + i) % queueSize] * anchorDistance[(startIndex + i) % queueSize] - anchorX[(startIndex + i) % queueSize] * anchorX[(startIndex + i) % queueSize] - anchorY[(startIndex + i) % queueSize] * anchorY[(startIndex + i) % queueSize] - anchorZ[(startIndex + i) % queueSize] * anchorZ[(startIndex + i) % queueSize];

    // Calculate position
    // Prepare matrices
    float Pinv[4 * samples];  // pseudo-inverse
    float Pos[4];             // pseudo-inverse * b
    arm_matrix_instance_f32 Pinvm = {4, samples, Pinv};
    arm_matrix_instance_f32 Posm = {4, 1, Pos};

    // Get pseudo-inverse
    mat_pinv(&Am, &Pinvm);

    // Get position
    mat_mult(&Pinvm, &bm, &Posm);

    // Write position
    position->x = Pos[1];
    position->y = Pos[2];
    position->z = Pos[3];

    // Write to logging variables
    logPosition.x = Pos[1];
    logPosition.y = Pos[2];
    logPosition.z = Pos[3];
  }

  // Check for NaNs
  ASSERT(stateNotNaN());
}


/**
 * Utility functions
 */


// Check position and height are not NaN
static bool stateNotNaN(void)
{
  if ((isnan(logHeight)) ||
      (isnan(logPosition.x)) ||
      (isnan(logPosition.y)) ||
      (isnan(logPosition.z)))
  {
    return false;
  }
  else
  {
    return true;
  }
}


// Matrix rank
// Based on https://cp-algorithms.com/linear_algebra/rank-matrix.html
static int mat_rank(const arm_matrix_instance_f32* pA)
{
  // Precision
  const float EPS = 1e-9f;

  // Shape
  int m = pA->numRows;
  int n = pA->numCols;

  // Copy the matrix
  // Column-row traversing: i = col index, j = row index
  float A[m][n];
  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j < m; j++)
      A[j][i] = pA->pData[j * n + i];
  }

  // Counters
  int rank = 0;
  bool rowSelected[m];

  // Columns
  for (int i = 0; i < n; i++)
  {
    // Row counter
    int j;

    // Check if done
    // Rows
    for (j = 0; j < m; j++)
    {
      if (!rowSelected[j] && fabsf(A[j][i]) > EPS)
        break;
    }

    // Do stuff
    if (j != m)
    {
      // Increase rank
      rank++;
      rowSelected[j] = true;

      for (int p = i + 1; p < n; p++)
        A[j][p] /= A[j][i];

      for (int k = 0; k < m; k++)
      {
        if (k != j && fabsf(A[k][i]) > EPS)
        {
          for (int pp = i + 1; pp < n; pp++)
            A[k][pp] -= A[j][pp] * A[k][i];
        }
      }
    }
  }
  return rank;
}


// Moore-Pensore pseudo inverse
static arm_status arm_mat_pinverse_f32(const arm_matrix_instance_f32* pSrc, arm_matrix_instance_f32* pDest)
{
  // Check sizes
  if (!(pSrc->numRows == pDest->numCols && pSrc->numCols == pDest->numRows))
    return ARM_MATH_SIZE_MISMATCH;

  // And save them for short use
  int m = pSrc->numRows;
  int n = pSrc->numCols;

  // Intermediate arrays
  // Source matrix is now 'A'
  float At[n * m];       // A transpose
  float AtA[n * n];      // A transpose * A
  float iAtA[n * n];     // inverse( A transpose * A )
  // Accompanying arm matrices
  arm_matrix_instance_f32 Atm = {n, m, At};
  arm_matrix_instance_f32 AtAm = {n, n, AtA};
  arm_matrix_instance_f32 iAtAm = {n, n, iAtA};

  // Transpose
  mat_trans(pSrc, &Atm);
  // Multiply
  mat_mult(&Atm, pSrc, &AtAm);
  // Invert
  mat_inv(&AtAm, &iAtAm);
  // Get pseudo inverse
  mat_mult(&iAtAm, &Atm, pDest);

  // Success!
  return ARM_MATH_SUCCESS;
}


/**
 * Logging and adjustable parameters
 */

// Stock group
LOG_GROUP_START(UWB2POS)
  // Estimated height
  LOG_ADD(LOG_FLOAT, coreForcedZ, &logHeight)

  // Estimated position
  LOG_ADD(LOG_FLOAT, coreX, &logPosition.x)
  LOG_ADD(LOG_FLOAT, coreY, &logPosition.y)
  LOG_ADD(LOG_FLOAT, coreZ, &logPosition.z)
LOG_GROUP_STOP(UWB2POS)

// Parameters
PARAM_GROUP_START(UWB2POS)
  PARAM_ADD(PARAM_FLOAT, estAlphaAsl, &laserStruct.estAlphaAsl)
  PARAM_ADD(PARAM_FLOAT, estAlphaZr, &laserStruct.estAlphaZrange)
  PARAM_ADD(PARAM_FLOAT, velFactor, &laserStruct.velocityFactor)
  PARAM_ADD(PARAM_FLOAT, velZAlpha, &laserStruct.velZAlpha)
  PARAM_ADD(PARAM_FLOAT, vAccDeadband, &laserStruct.vAccDeadband)
PARAM_GROUP_STOP(UWB2POS)
