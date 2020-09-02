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
 * mhe_core.h - Interface for core functions for the moving horizon estimator
 */

#ifndef __MHE_CORE_H__
#define __MHE_CORE_H__


/**
 * Includes
 */

// Important types
#include "stabilizer_types.h"


/**
 * Custom types
 */

// Indices to access the quad's state, stored as a row vector
// Positions & velocities all in the global/world frame
typedef enum
{
  MHC_STATE_X, MHC_STATE_Y, MHC_STATE_Z,
  MHC_STATE_VX, MHC_STATE_VY, MHC_STATE_VZ,
  MHC_STATE_DIM
} mheCoreStateIdx_t;

// Data used by the moving horizon estimator
typedef struct
{
  /**
   * Quadcopter state
   *
   * The internal state is:
   * - X, Y, Z: the quad's position in the world frame
   * - VX, VY, VZ: the quad's velocity in the world frame
   *
   * Separate copies are kept for prediction, correction and finalization
   */
  float S[MHC_STATE_DIM];
  float C[MHC_STATE_DIM];
  float F[MHC_STATE_DIM];

  // The quad's attitude as Euler angles
  float att[3];

  // Indicates that the internal state is corrupt and needs resetting
  bool resetEstimation;
} mheCoreData_t;


/**
 * Core estimator functions
 */

// Init and reset
void mheCoreInit(mheCoreData_t* this);

// Update prediction of state
#ifdef NO_IMU
bool mheCorePredictNoIMU(mheCoreData_t* this, velocity_t* vel, float dt);
#else
bool mheCorePredict(mheCoreData_t* this, float dt);
#endif

// Update correction of state
bool mheCoreCorrect(mheCoreData_t* this, const point_t* position, float timestamp);

// Finalize state: combine prediction and correction
bool mheCoreFinalize(mheCoreData_t* this, float dt);

// Externalize state
void mheCoreExternalize(mheCoreData_t* this, state_t *state, uint32_t tick);


#endif //__MHE_CORE_H__
