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

#ifndef __ADS1115_H__
#define __ADS1115_H__

#include "driver/i2c_master.h"
#include "esp_err.h"

#define ADS1115_ADDRESS_ADDR_TO_GND 0x48
#define ADS1115_ADDRESS_ADDR_TO_VDD 0x49
#define ADS1115_ADDRESS_ADDR_TO_SDA 0x4A
#define ADS1115_ADDRESS_ADDR_TO_SCL 0x4B

#define ADS1115_REG_CONVERSION 0x00
#define ADS1115_REG_CONFIG     0x01

#define ADS1115_OS_START 0x8000

#define ADS1115_MUX_0_1   0x0000
#define ADS1115_MUX_0_3   0x1000
#define ADS1115_MUX_1_3   0x2000
#define ADS1115_MUX_2_3   0x3000
#define ADS1115_MUX_0_GND 0x4000
#define ADS1115_MUX_1_GND 0x5000
#define ADS1115_MUX_2_GND 0x6000
#define ADS1115_MUX_3_GND 0x7000

#define ADS1115_PGA_6_144 0x0000
#define ADS1115_PGA_4_096 0x0200
#define ADS1115_PGA_2_048 0x0400
#define ADS1115_PGA_1_024 0x0600
#define ADS1115_PGA_0_512 0x0800
#define ADS1115_PGA_0_256 0x0A00

#define ADS1115_MODE_CONTINUOUS  0x0000
#define ADS1115_MODE_SINGLE_SHOT 0x0100

#define ADS1115_DR_8_SPS   0x0000
#define ADS1115_DR_16_SPS  0x0020
#define ADS1115_DR_32_SPS  0x0040
#define ADS1115_DR_64_SPS  0x0060
#define ADS1115_DR_128_SPS 0x0080
#define ADS1115_DR_250_SPS 0x00A0
#define ADS1115_DR_475_SPS 0x00C0
#define ADS1115_DR_860_SPS 0x00E0

#define ADS1115_COMP_MODE_TRADITIONAL 0x0000
#define ADS1115_COMP_MODE_WINDOW      0x0010

#define ADS1115_COMP_POL_LOW  0x0000
#define ADS1115_COMP_POL_HIGH 0x0008

#define ADS1115_COMP_LAT_NONLATCHING 0x0000
#define ADS1115_COMP_LAT_LATCHING    0x0004

#define ADS1115_COMP_QUE_ASSERT1 0x0000
#define ADS1115_COMP_QUE_ASSERT2 0x0001
#define ADS1115_COMP_QUE_ASSERT4 0x0002
#define ADS1115_COMP_QUE_DISABLE 0x0003

#define ADS1115_CONVERSION_TIMEOUT_MS 200

esp_err_t ads1115_init(i2c_master_dev_handle_t* dev_handle);
esp_err_t ads1115_read_ADC(uint8_t channel, int16_t* adc_value, i2c_master_dev_handle_t* dev_handle);

#endif /* __ADS1115_H__ */
