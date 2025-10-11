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

#ifndef __BME280_H__
#define __BME280_H__

#include "driver/i2c_master.h"
#include "esp_err.h"

#define BME280_ADDRESS_SDO_TO_GND 0x76
#define BME280_ADDRESS_SDO_TO_VDD 0x77

#define BME280_REG_DIG_T1     0x88
#define BME280_REG_DIG_T2     0x8A
#define BME280_REG_DIG_T3     0x8C
#define BME280_REG_DIG_P1     0x8E
#define BME280_REG_DIG_P2     0x90
#define BME280_REG_DIG_P3     0x92
#define BME280_REG_DIG_P4     0x94
#define BME280_REG_DIG_P5     0x96
#define BME280_REG_DIG_P6     0x98
#define BME280_REG_DIG_P7     0x9A
#define BME280_REG_DIG_P8     0x9C
#define BME280_REG_DIG_P9     0x9E
#define BME280_REG_DIG_H1     0xA1
#define BME280_REG_DIG_H2     0xE1
#define BME280_REG_DIG_H3     0xE3
#define BME280_REG_DIG_H4     0xE4
#define BME280_REG_DIG_H5_MSB 0xE5
#define BME280_REG_DIG_H5_LSB 0xE6
#define BME280_REG_DIG_H6     0xE7
#define BME280_REG_CTRL_HUM   0xF2
#define BME280_REG_STATUS     0xF3
#define BME280_REG_CTRL_MEAS  0xF4
#define BME280_REG_CONFIG     0xF5
#define BME280_REG_PRESS_MSB  0xF7

#define BME280_IIR_FILTER_OFF 0x00
#define BME280_IIR_FILTER_X2  0x01
#define BME280_IIR_FILTER_X4  0x02
#define BME280_IIR_FILTER_X8  0x03
#define BME280_IIR_FILTER_X16 0x04

#define BME280_MODE_SLEEP  0x00
#define BME280_MODE_FORCED 0x01
#define BME280_MODE_NORMAL 0x03

#define BME280_NO_OVERSAMPLING  0x00
#define BME280_OVERSAMPLING_X1  0x01
#define BME280_OVERSAMPLING_X2  0x02
#define BME280_OVERSAMPLING_X4  0x03
#define BME280_OVERSAMPLING_X8  0x04
#define BME280_OVERSAMPLING_X16 0x05

#define BME280_STANDBY_0_5_MS  0x00
#define BME280_STANDBY_62_5_MS 0x01
#define BME280_STANDBY_125_MS  0x02
#define BME280_STANDBY_250_MS  0x03
#define BME280_STANDBY_500_MS  0x04
#define BME280_STANDBY_1000_MS 0x05
#define BME280_STANDBY_10_MS   0x06
#define BME280_STANDBY_20_MS   0x07

#define BME280_HUMIDITY_MEAS_MIN_VALUE 0
#define BME280_HUMIDITY_MEAS_MAX_VALUE 100

#define BME280_PRESSURE_MEAS_MIN_VALUE 300
#define BME280_PRESSURE_MEAS_MAX_VALUE 1100

#define BME280_TEMPERATURE_MEAS_MIN_VALUE -40
#define BME280_TEMPERATURE_MEAS_MAX_VALUE 85

typedef struct {
  uint32_t humidity;
  uint32_t pressure;
  int32_t temperature;
} bme280_data_t;

esp_err_t bme280_init(i2c_master_dev_handle_t* dev_handle);
esp_err_t bme280_read_values(bme280_data_t* data, i2c_master_dev_handle_t* dev_handle);

#endif /* __BME280_H__ */
