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

#include "wind_vane.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

#include "ads1115.h"
#include "sdkconfig.h"

#if CONFIG_WIND_VANE_ENABLED

  #if CONFIG_WIND_VANE_RESOLUTION_8_POINTS
static const wind_direction_t wind_directions[] = {
  { "N", 0.0f, CONFIG_WIND_VANE_CALIBRATION_N },   { "NE", 45.0f, CONFIG_WIND_VANE_CALIBRATION_NE },
  { "E", 90.0f, CONFIG_WIND_VANE_CALIBRATION_E },  { "SE", 135.0f, CONFIG_WIND_VANE_CALIBRATION_SE },
  { "S", 180.0f, CONFIG_WIND_VANE_CALIBRATION_S }, { "SW", 225.0f, CONFIG_WIND_VANE_CALIBRATION_SW },
  { "W", 270.0f, CONFIG_WIND_VANE_CALIBRATION_W }, { "NW", 315.0f, CONFIG_WIND_VANE_CALIBRATION_NW }
};
static const size_t wind_directions_count = sizeof(wind_directions) / sizeof(wind_directions[0]);
  #endif

/**
 * @brief Converts a wind direction string to degrees.
 *
 * This function takes a cardinal or intercardinal direction string (e.g., "N", "NE", "SW")
 * and converts it to its corresponding angle in degrees, where North is 0°/360°,
 * East is 90°, South is 180°, and West is 270°.
 *
 * @param[in] direction Pointer to a null-terminated string representing the wind direction.
 *                      Expected values include cardinal directions (N, E, S, W) and
 *                      intercardinal directions (NE, SE, SW, NW, etc.).
 * @param[out] degrees Pointer to a float where the converted degree value will be stored.
 *                     The value will be in the range [0, 360).
 *
 * @return
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if direction is NULL, degrees is NULL, or direction string is invalid
 *     - ESP_FAIL if the conversion fails for any other reason
 */

esp_err_t direction_to_degrees(const char* direction, float* degrees) {
  for (size_t i = 0; i < wind_directions_count; i++) {
    if (!strcmp(direction, wind_directions[i].name)) {
      *degrees = wind_directions[i].degrees;
      return ESP_OK;
    }
  }

  return ESP_ERR_NOT_FOUND;
}

/**
 * @brief Get the current wind direction from the wind vane sensor.
 *
 * This function reads the wind direction data from the wind vane sensor via I2C
 * and returns the direction as a string.
 *
 * @param[out] direction Pointer to a string pointer that will be set to the wind direction.
 *                       The direction string is typically one of the cardinal or intercardinal
 *                       directions (e.g., "N", "NE", "E", "SE", "S", "SW", "W", "NW").
 * @param[in] dev_handle Pointer to the I2C master device handle for communicating with the sensor.
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer)
 *     - ESP_FAIL: Communication failure or sensor error
 *     - Other ESP_ERR codes depending on I2C communication errors
 */

esp_err_t get_wind_direction(const char** direction, i2c_master_dev_handle_t* dev_handle) {
  if (direction == NULL || dev_handle == NULL) return ESP_ERR_INVALID_ARG;

  int16_t min_diff = INT16_MAX;
  int16_t adc_value;

  if (ads1115_read_ADC(CONFIG_WIND_VANE_CHANNEL_TO_USE, &adc_value, dev_handle) == ESP_OK) {
    for (size_t i = 0; i < wind_directions_count; i++) {
      int16_t diff = abs(adc_value - wind_directions[i].value);

      if (diff < min_diff) {
        min_diff = diff;
        *direction = wind_directions[i].name;
      }
    }

    return ESP_OK;
  }

  *direction = "UNKNOWN";
  return ESP_FAIL;
}

#endif
