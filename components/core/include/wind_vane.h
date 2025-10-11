/*
 * Ozone Open Source Project
 * Copyright (C) 2025 David Moreau
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __WINDVANE_H__
#define __WINDVANE_H__

#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
  const char* name;
  float degrees;
  int value;
} wind_direction_t;

esp_err_t direction_to_degrees(const char* direction, float* degrees);
esp_err_t get_wind_direction(const char** direction, i2c_master_dev_handle_t* dev_handle);

#endif /* __WINDVANE_H__ */
