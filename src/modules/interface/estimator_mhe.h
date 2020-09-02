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
 * estimator_mhe.h - Interface for moving horizon estimator
 */

#ifndef __ESTIMATOR_MHE_H__
#define __ESTIMATOR_MHE_H__


/**
 * Includes
 */

// Important types
#include "stabilizer_types.h"


/**
 * Estimator task functions
 */

// Init
void estimatorMheTaskInit(void);

// Test
bool estimatorMheTaskTest(void);


/**
 * Main estimator functions
 */

// Init / reset
void estimatorMheInit(void);

// Test
bool estimatorMheTest(void);

// Main (called from stabilizer loop)
/**
 * TODO: some const here?
 */
void estimatorMhe(state_t* state, sensorData_t* sensors, control_t* control, uint32_t tick);

#ifdef NO_IMU
void estimatorMheEnqueueVelocity(const velocity_t *vel);
#endif

#endif //__ESTIMATOR_MHE_H__
