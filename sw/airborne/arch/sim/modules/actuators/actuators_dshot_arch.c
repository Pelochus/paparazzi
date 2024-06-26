/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file "modules/actuators/actuators_dshot_arch.c"
 * @author Gautier Hattenberger
 * Simulation driver for DSHOT speed controller protocol
 * Arch dependent part
 */

#include "modules/actuators/actuators_dshot.h"

struct dshot actuators_dshot_values[ACTUATORS_DSHOT_NB];

void actuators_dshot_arch_init(void) {}

void actuators_dshot_arch_commit(void) {}

