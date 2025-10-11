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

#ifndef __BH1750_H__
#define __BH1750_H__

#include "driver/i2c_master.h"
#include "esp_err.h"

#define BH1750_ADDRESS_ADDR_TO_GND 0x23
#define BH1750_ADDRESS_ADDR_TO_VCC 0x5C

#define BH1750_CMD_POWER_ON              0x01
#define BH1750_CMD_RESET                 0x07
#define BH1750_CMD_CONTINUOUS_HIGH_RES   0x10
#define BH1750_CMD_CONTINUOUS_HIGH_RES2  0x11
#define BH1750_CMD_CONTINUOUS_LOW_RES    0x13
#define BH1750_CMD_SINGLE_SHOT_HIGH_RES  0x20
#define BH1750_CMD_SINGLE_SHOT_HIGH_RES2 0x21
#define BH1750_CMD_SINGLE_SHOT_LOW_RES   0x23

#define BH1750_DELAY_HIGH_RES 120
#define BH1750_DELAY_LOW_RES  16

#define BH1750_ILLUMINANCE_MEAS_MIN_VALUE 1
#define BH1750_ILLUMINANCE_MEAS_MAX_VALUE 65535

esp_err_t bh1750_init(i2c_master_dev_handle_t* dev_handle);
esp_err_t bh1750_read_illuminance(float* lux, i2c_master_dev_handle_t* dev_handle);

#endif /* __BH1750_H__ */
