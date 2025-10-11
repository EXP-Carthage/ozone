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

#include "bh1750.h"

#include "freertos/FreeRTOS.h"
#include "i2c.h"
#include "sdkconfig.h"

/**
 * @brief Initialize the BH1750 light sensor
 *
 * @param dev_handle Pointer to the I2C master device handle
 *
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_FAIL: Initialization failed
 *         - Other error codes from I2C operations
 */

esp_err_t bh1750_init(i2c_master_dev_handle_t* dev_handle) {
  if (dev_handle == NULL) return ESP_ERR_INVALID_ARG;

#if CONFIG_BH1750_I2C_ADDRESS_ADDR_TO_GND
  uint8_t dev_addr = BH1750_ADDRESS_ADDR_TO_GND;
#else
  uint8_t dev_addr = BH1750_ADDRESS_ADDR_TO_VCC;
#endif

#if CONFIG_I2C_FREQ_STANDARD
  uint32_t scl_freq = I2C_FREQ_100KHZ_STANDARD;
#else
  uint32_t scl_freq = I2C_FREQ_400KHZ_FAST;
#endif

  ESP_ERROR_CHECK(i2c_add_device(dev_addr, scl_freq, dev_handle));
  ESP_ERROR_CHECK(i2c_write_command(BH1750_CMD_POWER_ON, dev_handle));
  ESP_ERROR_CHECK(i2c_write_command(BH1750_CMD_RESET, dev_handle));

  return ESP_OK;
}

/**
 * @brief Read illuminance value from BH1750 sensor
 *
 * This function reads the current light intensity from the BH1750 ambient light sensor
 * and converts it to lux units.
 *
 * @param[out] lux Pointer to float variable where the illuminance value (in lux) will be stored
 * @param[in] dev_handle Pointer to I2C master device handle for BH1750 sensor
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer)
 *     - ESP_FAIL: I2C communication failure
 *     - Other ESP_ERR codes: Other errors from I2C operations
 */

esp_err_t bh1750_read_illuminance(float* lux, i2c_master_dev_handle_t* dev_handle) {
  if (dev_handle == NULL || lux == NULL) return ESP_ERR_INVALID_ARG;

#if CONFIG_BH1750_MODE_SINGLE_SHOT
  ESP_ERROR_CHECK(i2c_write_command(BH1750_CMD_POWER_ON, dev_handle));

  #if CONFIG_BH1750_L_RESOLUTION
  ESP_ERROR_CHECK(i2c_write_command(BH1750_CMD_SINGLE_SHOT_LOW_RES, dev_handle));
  #elif CONFIG_BH1750_H_RESOLUTION
  ESP_ERROR_CHECK(i2c_write_command(BH1750_CMD_SINGLE_SHOT_HIGH_RES, dev_handle));
  #else
  ESP_ERROR_CHECK(i2c_write_command(BH1750_CMD_SINGLE_SHOT_HIGH_RES2, dev_handle));
  #endif

  #if CONFIG_BH1750_L_RESOLUTION
  vTaskDelay(pdMS_TO_TICKS(BH1750_DELAY_LOW_RES));
  #else
  vTaskDelay(pdMS_TO_TICKS(BH1750_DELAY_HIGH_RES));
  #endif
#else
  #if CONFIG_BH1750_L_RESOLUTION
  ESP_ERROR_CHECK(i2c_write_command(BH1750_CMD_CONTINUOUS_LOW_RES, dev_handle));
  #elif CONFIG_BH1750_H_RESOLUTION
  ESP_ERROR_CHECK(i2c_write_command(BH1750_CMD_CONTINUOUS_HIGH_RES, dev_handle));
  #else
  ESP_ERROR_CHECK(i2c_write_command(BH1750_CMD_CONTINUOUS_HIGH_RES2, dev_handle));
  #endif

  #if CONFIG_BH1750_L_RESOLUTION
  vTaskDelay(pdMS_TO_TICKS(BH1750_DELAY_LOW_RES));
  #else
  vTaskDelay(pdMS_TO_TICKS(BH1750_DELAY_HIGH_RES));
  #endif
#endif

  uint8_t data[2];
  ESP_ERROR_CHECK(i2c_read_data(data, 2, dev_handle));
  *lux = ((data[0] << 8) | data[1]) / 1.2;

#if CONFIG_BH1750_VH_RESOLUTION
  *lux /= 2;
#endif

  return ESP_OK;
}
