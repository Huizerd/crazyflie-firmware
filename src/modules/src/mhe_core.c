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
 * mhe_core.c - Core functions for the moving horizon estimator
 */


/**
 * Includes
 */

// Own
#include "mhe_core.h"

// Assert
#include "cfassert.h"
// TDoA outlier filter
// #include "outlierFilter.h"
// 3-element vector math
#include "math3d.h"
// Includes arm matrices + math
#include "cf_math.h"

// Logging
#include "log.h"
// Adjustable parameters
#include "param.h"

// Debugging
#define DEBUG_MODULE "MHE_CORE"
#include "debug.h"

// Time
#include <time.h>
// Random numbers
#include <stdlib.h>


/**
 * File constants
 */

// Constants for prediction
static const float GRAV = 9.81f;  // gravity
static const float DRAG = 0.35f;  // drag

// Constants for moving horizon filter
// Define to allow array initialization
/**
 * TODO: tune this value in Cyberzoo
 */
#define MAXWINDOWSIZE 20


/**
 * File statics
 */

// Position and yaw initialization
// Needs to be adjustable
// Yaw: 0 -> pos X; PI / 2 -> pos Y; etc.
static float initX = 0.0f;
static float initY = 0.0f;
static float initZ = 0.0f;
static float initYaw = 0.0f;

// Settings for moving horizon functions
static float timeHorizon = 0.5f;
static int ransacIterations = 10;
static int ransacSamples = 2;
static float ransacPrior[4] = {0.0f};
static float ransacThresh = 0.2f;


/**
 * Static function prototypes
 */

// Estimator subfunctions
// Correction: update windows
static void mheCoreUpdateWindows(const mheCoreData_t* this, const point_t* position, float timestamp, float* wT, float* wDp, int* start, int* stop);
// Correction: do RANSAC
static void mheCoreDoRansac(mheCoreData_t* this, const float* wDt, const float* wDp, int start, int stop, int samples);

// Check state is not NaN
static bool stateNotNaN(const mheCoreData_t* this);

// Random numbers up to a limit (inclusive)
static int randLim(int limit);

// Linear least squares
static void linearLeastSquares(float* b, float* a, const arm_matrix_instance_f32* Xm, const arm_matrix_instance_f32* Ym, const arm_matrix_instance_f32* priorm);

// Fast arm sqrt
static inline float arm_sqrt(float32_t in)
{ float pOut = 0; if (ARM_MATH_SUCCESS != arm_sqrt_f32(in, &pOut)) {DEBUG_PRINT("Square root failed\n");} return pOut; }

// Matrix functions: transpose, inverse, multiplication, addition
static inline void mat_trans(const arm_matrix_instance_f32* pSrc, arm_matrix_instance_f32* pDst)
{ ASSERT(ARM_MATH_SUCCESS == arm_mat_trans_f32(pSrc, pDst)); }
static inline void mat_inv(const arm_matrix_instance_f32* pSrc, arm_matrix_instance_f32* pDst)
{ if (ARM_MATH_SUCCESS != arm_mat_inverse_f32(pSrc, pDst)) {DEBUG_PRINT("Matrix inverse failed\n");} }
static inline void mat_mult(const arm_matrix_instance_f32* pSrcA, const arm_matrix_instance_f32* pSrcB, arm_matrix_instance_f32* pDst)
{ ASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst)); }
static inline void mat_add(const arm_matrix_instance_f32* pSrcA, const arm_matrix_instance_f32* pSrcB, arm_matrix_instance_f32* pDst)
{ ASSERT(ARM_MATH_SUCCESS == arm_mat_add_f32(pSrcA, pSrcB, pDst)); }


/**
 * Core estimator functions
 */


// Init and reset of core state
void mheCoreInit(mheCoreData_t* this)
{
  // Set random seed
  srand(time(NULL));

  // Set all data to 0 (like upon system reset)
  memset(this, 0, sizeof(mheCoreData_t));

  // Set initial position (for prediction, correction and finalization) and yaw
  this->S[MHC_STATE_X] = initX;
  this->S[MHC_STATE_Y] = initY;
  this->S[MHC_STATE_Z] = initZ;
  this->C[MHC_STATE_X] = initX;
  this->C[MHC_STATE_Y] = initY;
  this->C[MHC_STATE_Z] = initZ;
  this->F[MHC_STATE_X] = initX;
  this->F[MHC_STATE_Y] = initY;
  this->F[MHC_STATE_Z] = initZ;
  this->att[2] = initYaw;

  // Check for NaNs
  ASSERT(stateNotNaN(this));
}


// Update prediction of state
void mheCorePredict(mheCoreData_t* this, float dt)
{
  // Compute roll and pitch in the world frame
  float rollG = -this->att[1] * arm_sin_f32(this->att[2])
               + this->att[0] * arm_cos_f32(this->att[2]);
  float pitchG = this->att[1] * arm_cos_f32(this->att[2])
               + this->att[0] * arm_sin_f32(this->att[2]);

  // Position
  this->S[MHC_STATE_X] += this->S[MHC_STATE_VX] * dt;
  this->S[MHC_STATE_Y] += this->S[MHC_STATE_VY] * dt;
  this->S[MHC_STATE_Z] += 0.0f;

  // Velocity
  // tan = sin / cos
  this->S[MHC_STATE_VX] += (-GRAV * arm_sin_f32(pitchG) / arm_cos_f32(pitchG)
                            - DRAG * this->S[MHC_STATE_VX]) * dt;
  this->S[MHC_STATE_VY] += (-GRAV * arm_sin_f32(rollG) / arm_cos_f32(rollG)
                            - DRAG * this->S[MHC_STATE_VY]) * dt;
  this->S[MHC_STATE_VZ] = 0.0f;

  // Check for NaNs
  ASSERT(stateNotNaN(this));
}


// Update correction of state
// No need for dt, since position carries a float timestamp
void mheCoreCorrect(mheCoreData_t* this, const point_t* position, float timestamp)
{
  // Keep statics of windows here so we can use them again
  // Absolute time
  static float wT[MAXWINDOWSIZE] = {0.0f};
  // Time difference with start of window
  static float wDt[MAXWINDOWSIZE] = {0.0f};
  // Position difference
  static float wDp[MAXWINDOWSIZE * 3] = {0.0f};

  // Start and stop rows to emulate variable-length arrays
  static int start = MAXWINDOWSIZE - 1;
  static int stop = MAXWINDOWSIZE - 1;

  // Update windows
  // Pointers because we want to change all
  mheCoreUpdateWindows(this, position, timestamp, wT, wDp, &start, &stop);

  // Compute window size (dependent on order of start and stop)
  int samples;
  if (start < stop)
    samples = stop - start;
  else
    samples = MAXWINDOWSIZE - (stop - start);

  // Do RANSAC if enough samples
  if (samples >= ransacSamples)
  {
    // Update dt matrix first
    for (int i = 0; i < MAXWINDOWSIZE; i++)
      wDt[i] = wT[i] - wT[stop];

    // Then RANSAC
    mheCoreDoRansac(this, wDt, wDp, start, stop, samples);
  }

  // Check for NaNs
  ASSERT(stateNotNaN(this));
}


// Finalize state: combine prediction and correction
bool mheCoreFinalize(mheCoreData_t* this, float dt)
{
  // Position: prediction + correction in world frame
  this->F[MHC_STATE_X] = this->S[MHC_STATE_X] + this->C[MHC_STATE_X] + this->C[MHC_STATE_VX] * dt;
  this->F[MHC_STATE_Y] = this->S[MHC_STATE_Y] + this->C[MHC_STATE_Y] + this->C[MHC_STATE_VY] * dt;
  this->F[MHC_STATE_Z] = this->S[MHC_STATE_Z] + this->C[MHC_STATE_Z] + this->C[MHC_STATE_VZ] * dt;

  // Velocity: also in world frame
  this->F[MHC_STATE_VX] = this->S[MHC_STATE_VX] + this->C[MHC_STATE_VX];
  this->F[MHC_STATE_VY] = this->S[MHC_STATE_VY] + this->C[MHC_STATE_VY];
  this->F[MHC_STATE_VZ] = this->S[MHC_STATE_VZ] + this->C[MHC_STATE_VZ];

  // Check for NaNs
  ASSERT(stateNotNaN(this));

  // Success
  return true;
}


// Externalize state
void mheCoreExternalize(mheCoreData_t* this, state_t* state, uint32_t tick)
{
  // Position
  state->position = (point_t){
      .timestamp = tick,
      .x = this->F[MHC_STATE_X],
      .y = this->F[MHC_STATE_Y],
      .z = this->F[MHC_STATE_Z]
  };

  // Velocity
  state->velocity = (velocity_t){
      .timestamp = tick,
      .x = this->F[MHC_STATE_VX],
      .y = this->F[MHC_STATE_VY],
      .z = this->F[MHC_STATE_VZ]
  };

  // Check for NaNs
  ASSERT(stateNotNaN(this));
}


/**
 * Core subfunctions
 */


// Update windows function
static void mheCoreUpdateWindows(const mheCoreData_t* this, const point_t* position, float timestamp, float* wT, float* wDp, int* start, int* stop)
{
  // Add new measurements in a backwards, circular fashion
  wT[*start] = timestamp;
  wDp[*start * 3] = position->x - this->S[MHC_STATE_X];
  wDp[*start * 3 + 1] = position->y - this->S[MHC_STATE_Y];
  wDp[*start * 3 + 2] = position->z - this->S[MHC_STATE_Z];

  // Decrement start; reset at negative to emulate circular array
  (*start)--;
  if (*start < 0)
    *start = MAXWINDOWSIZE - 1;

  // Decrement stop while we are beyond the time horizon; reset at negative
  while ((wT[*start] - wT[*stop]) > timeHorizon)
  {
    (*stop)--;
    if (*stop < 0)
      *stop = MAXWINDOWSIZE - 1;
  }

  // stop should be earlier in time than start
  ASSERT(timestamp >= wT[*stop]);
}


// Estimate with RANSAC function
static void mheCoreDoRansac(mheCoreData_t* this, const float* wDt, const float* wDp, int start, int stop, int samples)
{
  // Upper bound for total error
  float bestError = (float) samples * ransacThresh + 1.0f;
  // Variables for best results
  int bestInliers = 0;
  bool bestIsInlier[samples];

  // Iterations
  for (int i = 0; i < ransacIterations; i++)
  {
    // Get random sample of row indices
    int idx[ransacSamples];
    for (int j = 0; j < ransacSamples; j++)
      idx[j] = (randLim(samples) + start) % MAXWINDOWSIZE;

    // RANSAC results
    float sDp[3], sDv[3];

    // Easier implementation with only 2 points
    if (ransacSamples == 2)
    {
      // Penalty
      float pv = ransacPrior[3];

      // Denominator
      float denom = (wDt[idx[1]] - wDt[idx[0]]) * (wDt[idx[1]] - wDt[idx[0]]) + 2.0f * pv;

      // No clue what sDp represents, position?
      sDp[0] = ((wDp[idx[0] * 3] * wDt[idx[1]] - wDp[idx[1] * 3] * wDt[idx[0]])
              * (wDt[idx[1]] - wDt[idx[0]]) + (wDp[idx[0] * 3] + wDp[idx[1] * 3]) * pv)
              / denom;
      sDp[1] = ((wDp[idx[0] * 3 + 1] * wDt[idx[1]] - wDp[idx[1] * 3 + 1] * wDt[idx[0]])
              * (wDt[idx[1]] - wDt[idx[0]])
              + (wDp[idx[0] * 3 + 1] + wDp[idx[1] * 3 + 1]) * pv)
              / denom;
      sDp[2] = ((wDp[idx[0] * 3 + 2] * wDt[idx[1]] - wDp[idx[1] * 3 + 2] * wDt[idx[0]])
              * (wDt[idx[1]] - wDt[idx[0]])
              + (wDp[idx[0] * 3 + 2] + wDp[idx[1] * 3 + 2]) * pv)
              / denom;

      // No clue what sDv represents, velocity?
      sDv[0] = (wDp[idx[1] * 3] - wDp[idx[0] * 3])
             * (wDt[idx[1]] - wDt[idx[0]])
             / denom;
      sDv[1] = (wDp[idx[1] * 3 + 1] - wDp[idx[0] * 3 + 1])
             * (wDt[idx[1]] - wDt[idx[0]])
             / denom;
      sDv[2] = (wDp[idx[1] * 3 + 2] - wDp[idx[0] * 3 + 2])
             * (wDt[idx[1]] - wDt[idx[0]])
             / denom;
    }
    // More samples
    else
    {
      // Prepare matrices
      float wDtSel[ransacSamples];
      float wDpSelX[ransacSamples], wDpSelY[ransacSamples], wDpSelZ[ransacSamples];
      // Accompanying arm matrices
      arm_matrix_instance_f32 wDtSelm = {ransacSamples, 1, wDtSel};
      arm_matrix_instance_f32 wDpSelXm = {ransacSamples, 1, wDpSelX};
      arm_matrix_instance_f32 wDpSelYm = {ransacSamples, 1, wDpSelY};
      arm_matrix_instance_f32 wDpSelZm = {ransacSamples, 1, wDpSelZ};
      arm_matrix_instance_f32 ransacPriorm = {2, 2, ransacPrior};

      // Fill matrices
      for (int j = 0; j < ransacSamples; j++)
      {
        wDtSel[j] = wDt[idx[j]];
        wDpSelX[j] = wDp[idx[j] * 3];
        wDpSelY[j] = wDp[idx[j] * 3 + 1];
        wDpSelZ[j] = wDp[idx[j] * 3 + 2];
      }

      // Do least squares
      linearLeastSquares(&sDp[0], &sDv[0], &wDtSelm, &wDpSelXm, &ransacPriorm);
      linearLeastSquares(&sDp[1], &sDv[1], &wDtSelm, &wDpSelYm, &ransacPriorm);
      linearLeastSquares(&sDp[2], &sDv[2], &wDtSelm, &wDpSelZm, &ransacPriorm);
    }

    // Find outliers
    bool isInlier[samples];
    int inliers = 0;
    float totalError = 0.0f;

    // Iterate over samples
    for (int j = 0; j < samples; j++)
    {
      // Errors
      float errX = (sDv[0] * wDt[(start + j) % MAXWINDOWSIZE] + sDp[0] - wDp[(start + j) % MAXWINDOWSIZE * 3])
                 * (sDv[0] * wDt[(start + j) % MAXWINDOWSIZE] + sDp[0] - wDp[(start + j) % MAXWINDOWSIZE * 3]);
      float errY = (sDv[1] * wDt[(start + j) % MAXWINDOWSIZE] + sDp[1] - wDp[(start + j) % MAXWINDOWSIZE * 3 + 1])
                 * (sDv[1] * wDt[(start + j) % MAXWINDOWSIZE] + sDp[1] - wDp[(start + j) % MAXWINDOWSIZE * 3 + 1]);
      float errZ = (sDv[2] * wDt[(start + j) % MAXWINDOWSIZE] + sDp[2] - wDp[(start + j) % MAXWINDOWSIZE * 3 + 2])
                 * (sDv[2] * wDt[(start + j) % MAXWINDOWSIZE] + sDp[2] - wDp[(start + j) % MAXWINDOWSIZE * 3 + 2]);

      // Sum
      float errSum = errX + errY + errZ;

      // Sqrt
      float stepError = arm_sqrt(errSum);

      // Is inlier?
      if (stepError <= ransacThresh)
      {
        isInlier[j] = true;
        inliers++;
      }
      totalError += stepError;
    }

    // Update best
    if (totalError < bestError)
    {
      // Copy by value
      bestError = totalError;
      bestInliers = inliers;

      // Copy inlier array
      for (int j = 0; j < samples; j++)
        bestIsInlier[j] = isInlier[j];
    }
  }

  // Recalculate best model with inliers
  // Prepare arrays
  float dtIn[bestInliers], dpInX[bestInliers], dpInY[bestInliers], dpInZ[bestInliers];
  // And accompanying arm matrices
  arm_matrix_instance_f32 dtInm = {bestInliers, 1, dtIn};
  arm_matrix_instance_f32 dpInXm = {bestInliers, 1, dpInX};
  arm_matrix_instance_f32 dpInYm = {bestInliers, 1, dpInY};
  arm_matrix_instance_f32 dpInZm = {bestInliers, 1, dpInZ};
  arm_matrix_instance_f32 ransacPriorm = {2, 2, ransacPrior};

  // Fill matrices
  // We go over all samples (i)
  // Only increment inlier counter j when we add one
  for (int i = 0, j = 0; i < samples; i++)
  {
    if (bestIsInlier[i])
    {
      dtIn[j] = wDt[(start + i) % MAXWINDOWSIZE];
      dpInX[j] = wDp[(start + i) % MAXWINDOWSIZE * 3];
      dpInY[j] = wDp[(start + i) % MAXWINDOWSIZE * 3 + 1];
      dpInZ[j] = wDp[(start + i) % MAXWINDOWSIZE * 3 + 2];
      j++;
    }
  }

  // Least squares results
  float dp[3], dv[3];

  // Do least squares
  linearLeastSquares(&dp[0], &dv[0], &dtInm, &dpInXm, &ransacPriorm);
  linearLeastSquares(&dp[1], &dv[1], &dtInm, &dpInYm, &ransacPriorm);
  linearLeastSquares(&dp[2], &dv[2], &dtInm, &dpInZm, &ransacPriorm);

  // Write to correction state
  // Position
  this->C[MHC_STATE_X] = dp[0];
  this->C[MHC_STATE_Y] = dp[1];
  this->C[MHC_STATE_Z] = dp[2];
  // Velocity
  this->C[MHC_STATE_VX] = dv[0];
  this->C[MHC_STATE_VY] = dv[1];
  this->C[MHC_STATE_VZ] = dv[2];
}


/**
 * Utility functions
 */


// Check state is not NaN
static bool stateNotNaN(const mheCoreData_t* this)
{
  if ((isnan(this->S[MHC_STATE_X])) ||
      (isnan(this->S[MHC_STATE_Y])) ||
      (isnan(this->S[MHC_STATE_Z])) ||
      (isnan(this->S[MHC_STATE_VX])) ||
      (isnan(this->S[MHC_STATE_VY])) ||
      (isnan(this->S[MHC_STATE_VZ])) ||
      (isnan(this->C[MHC_STATE_X])) ||
      (isnan(this->C[MHC_STATE_Y])) ||
      (isnan(this->C[MHC_STATE_Z])) ||
      (isnan(this->C[MHC_STATE_VX])) ||
      (isnan(this->C[MHC_STATE_VY])) ||
      (isnan(this->C[MHC_STATE_VZ])) ||
      (isnan(this->F[MHC_STATE_X])) ||
      (isnan(this->F[MHC_STATE_Y])) ||
      (isnan(this->F[MHC_STATE_Z])) ||
      (isnan(this->F[MHC_STATE_VX])) ||
      (isnan(this->F[MHC_STATE_VY])) ||
      (isnan(this->F[MHC_STATE_VZ])) ||
      (isnan(this->att[0])) ||
      (isnan(this->att[1])) ||
      (isnan(this->att[2])))
  {
    return false;
  }
  else
  {
    return true;
  }
}


// Random numbers up to a limit (inclusive)
// From https://stackoverflow.com/questions/2999075/generate-a-random-number-within-range/2999130#2999130
static int randLim(int limit)
{
  int divisor = RAND_MAX / (limit + 1);
  int retval;

  do
    retval = rand() / divisor;
  while (retval > limit);

  return retval;
}


// Linear least squares
static void linearLeastSquares(float* b, float* a, const arm_matrix_instance_f32* Xm, const arm_matrix_instance_f32* Ym, const arm_matrix_instance_f32* priorm)
{
  // Check matrix shapes
  // Xm and Ym should have 1 column and equal rows
  // priorm should be 2 x 2
  if (Xm->numCols != 1) DEBUG_PRINT("Xm has too many cols\n");
  if (Ym->numCols != 1) DEBUG_PRINT("Ym has too many cols\n");
  if (Xm->numRows != Ym->numRows) DEBUG_PRINT("Xm have a different number of rows\n");
  if (priorm->numRows != 2) DEBUG_PRINT("priorm doesn't have 2 rows\n");
  if (priorm->numCols != 2) DEBUG_PRINT("priorm doesn't have 2 cols\n");

  int m = Xm->numRows;

  // New X-matrix: append column of ones
  float X2[m * 2];
  arm_matrix_instance_f32 X2m = {m, 2, X2};

  // One column of ones, one with data
  for (int i = 0; i < m; i++)
  {
    X2[i * 2] = 1.0f;
    X2[i * 2 + 1] = Xm->pData[i];
  }

  /**
   * TODO: merge pseudo-inverse here with those in position estimation
   */
  // Prepare matrices for pseudo-inverse
  // Source matrix 'X2' is now 'A'
  float At[2 * m];       // A transpose
  float AtA[2 * 2];      // A transpose * A
  float AtA2[2 * 2];     // A transpose * A + prior
  float iAtA[2 * 2];     // inverse( A transpose * A )
  float pInv[2 * m];
  float beta[2];
  // Accompanying arm matrices
  arm_matrix_instance_f32 Atm = {2, m, At};
  arm_matrix_instance_f32 AtAm = {2, 2, AtA};
  arm_matrix_instance_f32 AtA2m = {2, 2, AtA2};
  arm_matrix_instance_f32 iAtAm = {2, 2, iAtA};
  arm_matrix_instance_f32 pInvm = {2, m, pInv};
  arm_matrix_instance_f32 betam = {2, 1, beta};

  // Transpose
  mat_trans(&X2m, &Atm);
  // Multiply
  mat_mult(&Atm, &X2m, &AtAm);
  // Add prior
  mat_add(&AtAm, priorm, &AtA2m);
  // Invert
  mat_inv(&AtA2m, &iAtAm);
  // Get pseudo inverse
  mat_mult(&iAtAm, &Atm, &pInvm);
  // Get beta
  mat_mult(&pInvm, Ym, &betam);

  // Get result
  *b = beta[0];
  *a = beta[1];
}


/**
 * Logging and adjustable parameters
 */

// Parameters
PARAM_GROUP_START(MHE)
  PARAM_ADD(PARAM_FLOAT, initX, &initX)
  PARAM_ADD(PARAM_FLOAT, initY, &initY)
  PARAM_ADD(PARAM_FLOAT, initZ, &initZ)
  PARAM_ADD(PARAM_FLOAT, initYaw, &initYaw)
  PARAM_ADD(PARAM_FLOAT, timeHorizon, &timeHorizon)
  PARAM_ADD(PARAM_UINT32, ransacIterations, &ransacIterations)
  PARAM_ADD(PARAM_UINT32, ransacSamples, &ransacSamples)
  PARAM_ADD(PARAM_FLOAT, ransacPriorB, &ransacPrior[0])
  PARAM_ADD(PARAM_FLOAT, ransacPriorA, &ransacPrior[3])
  PARAM_ADD(PARAM_FLOAT, ransacThresh, &ransacThresh)
PARAM_GROUP_STOP(MHE)
