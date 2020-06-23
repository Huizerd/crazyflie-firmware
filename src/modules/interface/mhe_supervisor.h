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
 * mhe_supervisor.h - Interface for state supervisor for the
 * moving horizon estimator
 */

#ifndef __MHE_SUPERVISOR_H__
#define __MHE_SUPERVISOR_H__


/**
 * Includes
 */

// Use of coreData
#include "mhe_core.h"
// Use of bool
#include <stdbool.h>


/**
 * Main supervisor functions
 */

// Check if state within bounds
bool mheSupervisorIsStateWithinBounds(const mheCoreData_t* this);


#endif //__MHE_SUPERVISOR_H__
