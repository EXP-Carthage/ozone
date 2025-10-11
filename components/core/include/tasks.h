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

#ifndef __TASKS_H__
#define __TASKS_H__

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

void anemometer_task(void* pvParameters);
void bh1750_task(void* pvParameters);
void bme280_task(void* pvParameters);
void wind_vane_task(void* pvParameters);
esp_err_t create_task(TaskFunction_t task_func, const char* name, uint32_t stack_size, UBaseType_t priority,
                      void* pvParameters);

#endif /* __TASKS_H__ */
