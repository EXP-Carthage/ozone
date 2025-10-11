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

#include "anemometer.h"

#include <math.h>

#include "ads1115.h"
#include "esp_log.h"
#include "sdkconfig.h"

#if CONFIG_ANEMOMETER_ENABLED

/**
 * @brief Get the current wind speed from the anemometer sensor.
 *
 * This function reads the wind speed measurement from an I2C-connected anemometer
 * sensor and stores the result in the provided buffer.
 *
 * @param[out] wind_speed_ms Pointer to store the wind speed value in meters per second
 * @param[in] dev_handle Pointer to the I2C master device handle for the anemometer
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer)
 *     - ESP_FAIL: Communication with sensor failed
 *     - Other ESP error codes from I2C operations
 *
 * @note The caller must ensure that wind_speed_ms points to valid memory
 * @note The I2C device handle must be initialized before calling this function
 */

esp_err_t get_wind_speed(uint8_t* wind_speed_ms, i2c_master_dev_handle_t* dev_handle) {
  if (wind_speed_ms == NULL || dev_handle == NULL) return ESP_ERR_INVALID_ARG;

  int16_t adc_value;

  ESP_ERROR_CHECK(ads1115_read_ADC(CONFIG_ANEMOMETER_CHANNEL_TO_USE, &adc_value, dev_handle));
  if (adc_value < 0) adc_value = 0;

  *wind_speed_ms =
      (uint8_t)(fminf(fmaxf(((float)(adc_value - CONFIG_ANEMOMETER_CALIBRATION_MIN) /
                             (float)(CONFIG_ANEMOMETER_CALIBRATION_MAX - CONFIG_ANEMOMETER_CALIBRATION_MIN)) *
                                CONFIG_ANEMOMETER_MAX_RANGE,
                            0.0f),
                      255.0f) +
                0.5f);

  return ESP_OK;
}

#endif
