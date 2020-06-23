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
 * Copyright (C) 2020 Bitcraze AB, Huizerd
 * 
 * uwb2pos_core.h - Interface for core functions for position estimation
 * from UWB / laser measurements
 */

#ifndef __UWB2POS_CORE_H__
#define __UWB2POS_CORE_H__


/**
 * Includes
 */

// Important types
#include "stabilizer_types.h"


/**
 * Custom types
 */

// Settings and placeholders for laser height estimation
typedef struct
{
  float estimatedZ;  // current Z estimate, same offset as asl
  float velocityZ;  // vertical speed (world frame) integrated from vertical acc (m/s)
  float estAlphaZrange;
  float estAlphaAsl;
  float velocityFactor;
  float vAccDeadband;  // vertical acceleration deadband
  float velZAlpha;  // blending factor to avoid vertical speed to accumulate error
} laserStruct_t;


/**
 * Height estimation
 */

// Laser ranger:
// - estimate height (called by state estimator at same rate as position estimation)
// - update vertical velocity (called by state estimator at same rate as attitude update)

// Estimate height
void laserHeight(point_t* position, const sensorData_t* sensorData, const tofMeasurement_t* tof, float dt, uint32_t tick);

// Update velocity
void laserVelocity(float accWZ, float dt);



/**
 * Position estimation:
 * - TDoA or TWR (distance)
 * - projection or multilateration (when enough samples)
 */

// TDoA projection
void uwbPosProjectTdoa(point_t* position, const float* anchorAx, const float* anchorAy, const float* anchorAz, const float* anchorBx, const float* anchorBy, const float* anchorBz, const float* anchorDistDiff, int startIndex, int samples, int queueSize, float forcedZ, bool forceZ);

// TDoA multilateration
void uwbPosMultilatTdoa(point_t* position, float* anchorAx, float* anchorAy, float* anchorAz, float* anchorBx, float* anchorBy, float* anchorBz, float* anchorDistDiff, uint32_t* anchorTimestamp, int startIndex, int samples, int queueSize, float forcedZ, bool forceZ);

// TWR projection
void uwbPosProjectTwr(point_t* position, const float* anchorX, const float* anchorY, const float* anchorZ, const float* anchorDistance, int startIndex, int samples, int queueSize, float forcedZ, bool forceZ);

// TWR multilateration
void uwbPosMultilatTwr(point_t* position, float* anchorX, float* anchorY, float* anchorZ, float* anchorDistance, uint32_t* anchorTimestamp, int startIndex, int samples, int queueSize, float forcedZ, bool forceZ);


#endif //__UWB2POS_CORE_H__
