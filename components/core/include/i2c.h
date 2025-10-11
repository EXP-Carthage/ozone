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

#ifndef __I2C_H__
#define __I2C_H__

#include "driver/i2c_master.h"
#include "esp_err.h"

#define I2C_FREQ_100KHZ_STANDARD 100000
#define I2C_FREQ_400KHZ_FAST     400000

esp_err_t i2c_init(void);
esp_err_t i2c_add_device(uint8_t dev_addr, uint32_t scl_speed_hz, i2c_master_dev_handle_t* dev_handle);
esp_err_t i2c_read_data(uint8_t* data, size_t length, i2c_master_dev_handle_t* dev_handle);
esp_err_t i2c_read_reg8(uint8_t reg_addr, uint8_t* value, i2c_master_dev_handle_t* dev_handle);
esp_err_t i2c_read_sequential_reg8(uint8_t start_reg_addr, uint8_t* data, size_t length,
                                   i2c_master_dev_handle_t* dev_handle);
esp_err_t i2c_read_reg16(uint8_t reg_addr, uint16_t* value, i2c_master_dev_handle_t* dev_handle);
esp_err_t i2c_read_reg16_LE(uint8_t reg_addr, uint16_t* value, i2c_master_dev_handle_t* dev_handle);
esp_err_t i2c_read_regS16_LE(uint8_t reg_addr, int16_t* value, i2c_master_dev_handle_t* dev_handle);
esp_err_t i2c_write_command(uint8_t command, i2c_master_dev_handle_t* dev_handle);
esp_err_t i2c_write_reg8(uint8_t reg_addr, uint8_t value, i2c_master_dev_handle_t* dev_handle);
esp_err_t i2c_write_reg16(uint8_t reg_addr, uint16_t value, i2c_master_dev_handle_t* dev_handle);

#endif /* __I2C_H__ */
