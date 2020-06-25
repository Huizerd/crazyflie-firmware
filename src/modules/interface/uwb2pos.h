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
 * uwb2pos.h - Interface for position estimation from UWB / laser measurements
 */

#ifndef __UWB2POS_H__
#define __UWB2POS_H__


/**
 * Includes
 */

// Important types
#include "stabilizer_types.h"


/**
 * Estimator task functions
 */

// Init
void uwb2posTaskInit(void);

// Test
bool uwb2posTaskTest(void);


/**
 * Interface functions to state estimators
 */

// Reset measurement queues
// This should be callable from MHE
void uwb2posReset(void);

// Externalize position estimate
bool uwb2posExternalize(point_t* externalPosition);

// Call task from stabilizer loop
// void uwb2posCall(const point_t* estPosition);
void uwb2posCall(void);


/**
 * Add to measurement queue functions
 */

// ToF / laser ranger for altitude
bool uwb2posEnqueueTOF(const tofMeasurement_t* tof);

// TDoA / position
bool uwb2posEnqueueTDOA(const tdoaMeasurement_t* tdoa);

// Distance / TWR / position
bool uwb2posEnqueueDistance(const distanceMeasurement_t* dist);


#endif //__UWB2POS_H__
