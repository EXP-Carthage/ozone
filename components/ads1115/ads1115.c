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

#include "ads1115.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "i2c.h"
#include "sdkconfig.h"

static SemaphoreHandle_t ads1115_mutex = NULL;

/**
 * @brief Waits for the ADS1115 ADC conversion to complete.
 *
 * This function polls or waits for the conversion ready status of the ADS1115
 * analog-to-digital converter. It monitors the conversion completion before
 * reading the conversion result.
 *
 * @param[in] dev_handle Pointer to the I2C master device handle for the ADS1115
 *
 * @return
 *     - ESP_OK: Conversion completed successfully
 *     - ESP_ERR_TIMEOUT: Conversion did not complete within expected time
 *     - ESP_ERR_INVALID_ARG: Invalid device handle
 *     - Other ESP_ERR codes: I2C communication errors
 */

static esp_err_t wait_conversion(i2c_master_dev_handle_t* dev_handle) {
  uint16_t config;
  TickType_t start = xTaskGetTickCount();

  while (1) {
    ESP_ERROR_CHECK(i2c_read_reg16(ADS1115_REG_CONFIG, &config, dev_handle));

    if (config & 0x8000) return ESP_OK;
    vTaskDelay(pdMS_TO_TICKS(1));
    if ((xTaskGetTickCount() - start) * portTICK_PERIOD_MS > ADS1115_CONVERSION_TIMEOUT_MS) return ESP_ERR_TIMEOUT;
  }
}

/**
 * @brief Initialize the ADS1115 ADC device
 *
 * This function initializes the ADS1115 analog-to-digital converter by setting up
 * the I2C device handle for communication.
 *
 * @param[out] dev_handle Pointer to I2C master device handle that will be initialized
 *                        for communication with the ADS1115
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer)
 *     - ESP_FAIL: Initialization failed
 *     - Other ESP_ERR codes from I2C driver operations
 */

esp_err_t ads1115_init(i2c_master_dev_handle_t* dev_handle) {
  if (dev_handle == NULL) return ESP_ERR_INVALID_ARG;

  if (ads1115_mutex == NULL) {
    ads1115_mutex = xSemaphoreCreateMutex();
    if (ads1115_mutex == NULL) return ESP_ERR_NO_MEM;
  }

#if CONFIG_ADS1115_I2C_ADDRESS_ADDR_TO_GND
  uint8_t dev_addr = ADS1115_ADDRESS_ADDR_TO_GND;
#elif CONFIG_ADS1115_I2C_ADDRESS_ADDR_TO_VDD
  uint8_t dev_addr = ADS1115_ADDRESS_ADDR_TO_VDD;
#elif CONFIG_ADS1115_I2C_ADDRESS_ADDR_TO_SDA
  uint8_t dev_addr = ADS1115_ADDRESS_ADDR_TO_SDA;
#else
  uint8_t dev_addr = ADS1115_ADDRESS_ADDR_TO_SCL;
#endif

#if CONFIG_I2C_FREQ_STANDARD
  uint32_t scl_freq = I2C_FREQ_100KHZ_STANDARD;
#else
  uint32_t scl_freq = I2C_FREQ_400KHZ_FAST;
#endif

  return i2c_add_device(dev_addr, scl_freq, dev_handle);
}

/**
 * @brief Read ADC value from a specific channel of the ADS1115
 *
 * This function reads the analog-to-digital conversion result from the specified
 * channel of the ADS1115 ADC chip via I2C communication.
 *
 * @param[in] channel The ADC channel to read from (typically 0-3 for ADS1115)
 * @param[out] adc_value Pointer to store the read 16-bit ADC value
 * @param[in] dev_handle Pointer to the I2C master device handle for communication
 *
 * @return
 *     - ESP_OK: Successfully read the ADC value
 *     - ESP_ERR_INVALID_ARG: Invalid parameter (NULL pointer or invalid channel)
 *     - ESP_FAIL: I2C communication failure or other errors
 */

esp_err_t ads1115_read_ADC(uint8_t channel, int16_t* adc_value, i2c_master_dev_handle_t* dev_handle) {
  if (dev_handle == NULL || adc_value == NULL) return ESP_ERR_INVALID_ARG;
  if (channel > 3) return ESP_ERR_INVALID_ARG;

  TickType_t start_time = xTaskGetTickCount();
  const TickType_t max_wait_ticks = 250;

  while (xSemaphoreTake(ads1115_mutex, 0) != pdTRUE) {
    if ((xTaskGetTickCount() - start_time) >= max_wait_ticks) return ESP_ERR_TIMEOUT;
    vTaskDelay(1);
  }

  uint16_t reg_config = ADS1115_OS_START;

  if (channel == 0) {
#if CONFIG_ADS1115_AIN0_MUX_0_1
    reg_config |= ADS1115_MUX_0_1;
#elif CONFIG_ADS1115_AIN0_MUX_0_3
    reg_config |= ADS1115_MUX_0_3;
#else
    reg_config |= ADS1115_MUX_0_GND;
#endif

#if CONFIG_ADS1115_AIN0_PGA_0_256
    reg_config |= ADS1115_PGA_0_256;
#elif CONFIG_ADS1115_AIN0_PGA_0_512
    reg_config |= ADS1115_PGA_0_512;
#elif CONFIG_ADS1115_AIN0_PGA_1_024
    reg_config |= ADS1115_PGA_1_024;
#elif CONFIG_ADS1115_AIN0_PGA_2_048
    reg_config |= ADS1115_PGA_2_048;
#elif CONFIG_ADS1115_AIN0_PGA_4_096
    reg_config |= ADS1115_PGA_4_096;
#else
    reg_config |= ADS1115_PGA_6_144;
#endif

#if CONFIG_ADS1115_AIN0_MODE_CONTINUOUS
    reg_config |= ADS1115_MODE_CONTINUOUS;
#else
    reg_config |= ADS1115_MODE_SINGLE_SHOT;
#endif

#if CONFIG_ADS1115_AIN0_DR_8_SPS
    reg_config |= ADS1115_DR_8_SPS;
#elif CONFIG_ADS1115_AIN0_DR_16_SPS
    reg_config |= ADS1115_DR_16_SPS;
#elif CONFIG_ADS1115_AIN0_DR_32_SPS
    reg_config |= ADS1115_DR_32_SPS;
#elif CONFIG_ADS1115_AIN0_DR_64_SPS
    reg_config |= ADS1115_DR_64_SPS;
#elif CONFIG_ADS1115_AIN0_DR_128_SPS
    reg_config |= ADS1115_DR_128_SPS;
#elif CONFIG_ADS1115_AIN0_DR_250_SPS
    reg_config |= ADS1115_DR_250_SPS;
#elif CONFIG_ADS1115_AIN0_DR_475_SPS
    reg_config |= ADS1115_DR_475_SPS;
#else
    reg_config |= ADS1115_DR_860_SPS;
#endif

#if CONFIG_ADS1115_AIN0_COMP_MODE_TRADITIONAL
    reg_config |= ADS1115_COMP_MODE_TRADITIONAL;
#else
    reg_config |= ADS1115_COMP_MODE_WINDOW;
#endif

#if CONFIG_ADS1115_AIN0_COMP_POL_LOW
    reg_config |= ADS1115_COMP_POL_LOW;
#else
    reg_config |= ADS1115_COMP_POL_HIGH;
#endif

#if CONFIG_ADS1115_AIN0_COMP_LAT_NONLATCHING
    reg_config |= ADS1115_COMP_LAT_NONLATCHING;
#else
    reg_config |= ADS1115_COMP_LAT_LATCHING;
#endif

#if CONFIG_ADS1115_AIN0_COMP_QUE_ASSERT1
    reg_config |= ADS1115_COMP_QUE_ASSERT1;
#elif CONFIG_ADS1115_AIN0_COMP_QUE_ASSERT2
    reg_config |= ADS1115_COMP_QUE_ASSERT2;
#elif CONFIG_ADS1115_AIN0_COMP_QUE_ASSERT4
    reg_config |= ADS1115_COMP_QUE_ASSERT4;
#else
    reg_config |= ADS1115_COMP_QUE_DISABLE;
#endif
  } else if (channel == 1) {
#if CONFIG_ADS1115_AIN1_MUX_0_1
    reg_config |= ADS1115_MUX_0_1;
#elif CONFIG_ADS1115_AIN1_MUX_1_3
    reg_config |= ADS1115_MUX_1_3;
#else
    reg_config |= ADS1115_MUX_1_GND;
#endif

#if CONFIG_ADS1115_AIN1_PGA_0_256
    reg_config |= ADS1115_PGA_0_256;
#elif CONFIG_ADS1115_AIN1_PGA_0_512
    reg_config |= ADS1115_PGA_0_512;
#elif CONFIG_ADS1115_AIN1_PGA_1_024
    reg_config |= ADS1115_PGA_1_024;
#elif CONFIG_ADS1115_AIN1_PGA_2_048
    reg_config |= ADS1115_PGA_2_048;
#elif CONFIG_ADS1115_AIN1_PGA_4_096
    reg_config |= ADS1115_PGA_4_096;
#else
    reg_config |= ADS1115_PGA_6_144;
#endif

#if CONFIG_ADS1115_AIN1_MODE_CONTINUOUS
    reg_config |= ADS1115_MODE_CONTINUOUS;
#else
    reg_config |= ADS1115_MODE_SINGLE_SHOT;
#endif

#if CONFIG_ADS1115_AIN1_DR_8_SPS
    reg_config |= ADS1115_DR_8_SPS;
#elif CONFIG_ADS1115_AIN1_DR_16_SPS
    reg_config |= ADS1115_DR_16_SPS;
#elif CONFIG_ADS1115_AIN1_DR_32_SPS
    reg_config |= ADS1115_DR_32_SPS;
#elif CONFIG_ADS1115_AIN1_DR_64_SPS
    reg_config |= ADS1115_DR_64_SPS;
#elif CONFIG_ADS1115_AIN1_DR_128_SPS
    reg_config |= ADS1115_DR_128_SPS;
#elif CONFIG_ADS1115_AIN1_DR_250_SPS
    reg_config |= ADS1115_DR_250_SPS;
#elif CONFIG_ADS1115_AIN1_DR_475_SPS
    reg_config |= ADS1115_DR_475_SPS;
#else
    reg_config |= ADS1115_DR_860_SPS;
#endif

#if CONFIG_ADS1115_AIN1_COMP_MODE_TRADITIONAL
    reg_config |= ADS1115_COMP_MODE_TRADITIONAL;
#else
    reg_config |= ADS1115_COMP_MODE_WINDOW;
#endif

#if CONFIG_ADS1115_AIN1_COMP_POL_LOW
    reg_config |= ADS1115_COMP_POL_LOW;
#else
    reg_config |= ADS1115_COMP_POL_HIGH;
#endif

#if CONFIG_ADS1115_AIN1_COMP_LAT_NONLATCHING
    reg_config |= ADS1115_COMP_LAT_NONLATCHING;
#else
    reg_config |= ADS1115_COMP_LAT_LATCHING;
#endif

#if CONFIG_ADS1115_AIN1_COMP_QUE_ASSERT1
    reg_config |= ADS1115_COMP_QUE_ASSERT1;
#elif CONFIG_ADS1115_AIN1_COMP_QUE_ASSERT2
    reg_config |= ADS1115_COMP_QUE_ASSERT2;
#elif CONFIG_ADS1115_AIN1_COMP_QUE_ASSERT4
    reg_config |= ADS1115_COMP_QUE_ASSERT4;
#else
    reg_config |= ADS1115_COMP_QUE_DISABLE;
#endif
  } else if (channel == 2) {
#if CONFIG_ADS1115_AIN2_MUX_2_3
    reg_config |= ADS1115_MUX_2_3;
#else
    reg_config |= ADS1115_MUX_2_GND;
#endif

#if CONFIG_ADS1115_AIN2_PGA_0_256
    reg_config |= ADS1115_PGA_0_256;
#elif CONFIG_ADS1115_AIN2_PGA_0_512
    reg_config |= ADS1115_PGA_0_512;
#elif CONFIG_ADS1115_AIN2_PGA_1_024
    reg_config |= ADS1115_PGA_1_024;
#elif CONFIG_ADS1115_AIN2_PGA_2_048
    reg_config |= ADS1115_PGA_2_048;
#elif CONFIG_ADS1115_AIN2_PGA_4_096
    reg_config |= ADS1115_PGA_4_096;
#else
    reg_config |= ADS1115_PGA_6_144;
#endif

#if CONFIG_ADS1115_AIN2_MODE_CONTINUOUS
    reg_config |= ADS1115_MODE_CONTINUOUS;
#else
    reg_config |= ADS1115_MODE_SINGLE_SHOT;
#endif

#if CONFIG_ADS1115_AIN2_DR_8_SPS
    reg_config |= ADS1115_DR_8_SPS;
#elif CONFIG_ADS1115_AIN2_DR_16_SPS
    reg_config |= ADS1115_DR_16_SPS;
#elif CONFIG_ADS1115_AIN2_DR_32_SPS
    reg_config |= ADS1115_DR_32_SPS;
#elif CONFIG_ADS1115_AIN2_DR_64_SPS
    reg_config |= ADS1115_DR_64_SPS;
#elif CONFIG_ADS1115_AIN2_DR_128_SPS
    reg_config |= ADS1115_DR_128_SPS;
#elif CONFIG_ADS1115_AIN2_DR_250_SPS
    reg_config |= ADS1115_DR_250_SPS;
#elif CONFIG_ADS1115_AIN2_DR_475_SPS
    reg_config |= ADS1115_DR_475_SPS;
#else
    reg_config |= ADS1115_DR_860_SPS;
#endif

#if CONFIG_ADS1115_AIN2_COMP_MODE_TRADITIONAL
    reg_config |= ADS1115_COMP_MODE_TRADITIONAL;
#else
    reg_config |= ADS1115_COMP_MODE_WINDOW;
#endif

#if CONFIG_ADS1115_AIN2_COMP_POL_LOW
    reg_config |= ADS1115_COMP_POL_LOW;
#else
    reg_config |= ADS1115_COMP_POL_HIGH;
#endif

#if CONFIG_ADS1115_AIN2_COMP_LAT_NONLATCHING
    reg_config |= ADS1115_COMP_LAT_NONLATCHING;
#else
    reg_config |= ADS1115_COMP_LAT_LATCHING;
#endif

#if CONFIG_ADS1115_AIN2_COMP_QUE_ASSERT1
    reg_config |= ADS1115_COMP_QUE_ASSERT1;
#elif CONFIG_ADS1115_AIN2_COMP_QUE_ASSERT2
    reg_config |= ADS1115_COMP_QUE_ASSERT2;
#elif CONFIG_ADS1115_AIN2_COMP_QUE_ASSERT4
    reg_config |= ADS1115_COMP_QUE_ASSERT4;
#else
    reg_config |= ADS1115_COMP_QUE_DISABLE;
#endif
  } else {
#if CONFIG_ADS1115_AIN3_MUX_0_3
    reg_config |= ADS1115_MUX_0_3;
#elif CONFIG_ADS1115_AIN3_MUX_1_3
    reg_config |= ADS1115_MUX_1_3;
#elif CONFIG_ADS1115_AIN3_MUX_2_3
    reg_config |= ADS1115_MUX_2_3;
#else
    reg_config |= ADS1115_MUX_3_GND;
#endif

#if CONFIG_ADS1115_AIN3_PGA_0_256
    reg_config |= ADS1115_PGA_0_256;
#elif CONFIG_ADS1115_AIN3_PGA_0_512
    reg_config |= ADS1115_PGA_0_512;
#elif CONFIG_ADS1115_AIN3_PGA_1_024
    reg_config |= ADS1115_PGA_1_024;
#elif CONFIG_ADS1115_AIN3_PGA_2_048
    reg_config |= ADS1115_PGA_2_048;
#elif CONFIG_ADS1115_AIN3_PGA_4_096
    reg_config |= ADS1115_PGA_4_096;
#else
    reg_config |= ADS1115_PGA_6_144;
#endif

#if CONFIG_ADS1115_AIN3_MODE_CONTINUOUS
    reg_config |= ADS1115_MODE_CONTINUOUS;
#else
    reg_config |= ADS1115_MODE_SINGLE_SHOT;
#endif

#if CONFIG_ADS1115_AIN3_DR_8_SPS
    reg_config |= ADS1115_DR_8_SPS;
#elif CONFIG_ADS1115_AIN3_DR_16_SPS
    reg_config |= ADS1115_DR_16_SPS;
#elif CONFIG_ADS1115_AIN3_DR_32_SPS
    reg_config |= ADS1115_DR_32_SPS;
#elif CONFIG_ADS1115_AIN3_DR_64_SPS
    reg_config |= ADS1115_DR_64_SPS;
#elif CONFIG_ADS1115_AIN3_DR_128_SPS
    reg_config |= ADS1115_DR_128_SPS;
#elif CONFIG_ADS1115_AIN3_DR_250_SPS
    reg_config |= ADS1115_DR_250_SPS;
#elif CONFIG_ADS1115_AIN3_DR_475_SPS
    reg_config |= ADS1115_DR_475_SPS;
#else
    reg_config |= ADS1115_DR_860_SPS;
#endif

#if CONFIG_ADS1115_AIN3_COMP_MODE_TRADITIONAL
    reg_config |= ADS1115_COMP_MODE_TRADITIONAL;
#else
    reg_config |= ADS1115_COMP_MODE_WINDOW;
#endif

#if CONFIG_ADS1115_AIN3_COMP_POL_LOW
    reg_config |= ADS1115_COMP_POL_LOW;
#else
    reg_config |= ADS1115_COMP_POL_HIGH;
#endif

#if CONFIG_ADS1115_AIN3_COMP_LAT_NONLATCHING
    reg_config |= ADS1115_COMP_LAT_NONLATCHING;
#else
    reg_config |= ADS1115_COMP_LAT_LATCHING;
#endif

#if CONFIG_ADS1115_AIN3_COMP_QUE_ASSERT1
    reg_config |= ADS1115_COMP_QUE_ASSERT1;
#elif CONFIG_ADS1115_AIN3_COMP_QUE_ASSERT2
    reg_config |= ADS1115_COMP_QUE_ASSERT2;
#elif CONFIG_ADS1115_AIN3_COMP_QUE_ASSERT4
    reg_config |= ADS1115_COMP_QUE_ASSERT4;
#else
    reg_config |= ADS1115_COMP_QUE_DISABLE;
#endif
  }

  uint16_t raw;
  ESP_ERROR_CHECK(i2c_write_reg16(ADS1115_REG_CONFIG, reg_config, dev_handle));
  ESP_ERROR_CHECK(wait_conversion(dev_handle));
  ESP_ERROR_CHECK(i2c_read_reg16(ADS1115_REG_CONVERSION, &raw, dev_handle));
  *adc_value = (int16_t)raw;

  xSemaphoreGive(ads1115_mutex);

  return ESP_OK;
}
