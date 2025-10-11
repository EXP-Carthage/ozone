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

#include "esp_log.h"
#include "i2c.h"
#include "sdkconfig.h"
#include "tasks.h"
#include "zigbee.h"

static const char* TAG = "MAIN";

#if (CONFIG_ANEMOMETER_TYPE_ADS1115 || CONFIG_WIND_VANE_TYPE_ADS1115)
  #include "ads1115.h"
static i2c_master_dev_handle_t ads1115_handle;
#endif

#if CONFIG_ILLUMINANCE_SENSOR_TYPE_BH1750
  #include "bh1750.h"
static i2c_master_dev_handle_t bh1750_handle;
#endif

#if (CONFIG_HUMIDITY_SENSOR_TYPE_BME280 || CONFIG_PRESSURE_SENSOR_TYPE_BME280 || CONFIG_TEMPERATURE_SENSOR_TYPE_BME280)
  #include "bme280.h"
static i2c_master_dev_handle_t bme280_handle;
#endif

#if (!CONFIG_ANEMOMETER_ENABLED && !CONFIG_HUMIDITY_SENSOR_ENABLED && !CONFIG_ILLUMINANCE_SENSOR_ENABLED && \
     !CONFIG_PRESSURE_SENSOR_ENABLED && !CONFIG_TEMPERATURE_SENSOR_ENABLED && !CONFIG_WIND_VANE_ENABLED)
  #error "No sensors activated. Please check your configuration."
#endif

#if ((CONFIG_ANEMOMETER_ENABLED && CONFIG_ANEMOMETER_TYPE_ADS1115) && \
     (CONFIG_WIND_VANE_ENABLED && CONFIG_WIND_VANE_TYPE_ADS1115))
  #if (CONFIG_ANEMOMETER_CHANNEL_TO_USE == CONFIG_WIND_VANE_CHANNEL_TO_USE)
    #error "Anemometer and Wind Vane cannot use the same ADS1115 channel!"
  #endif
#endif

void init_tasks(void);

/**
 * @brief Initializes all sensors connected to the weather station.
 *
 * This function performs the initialization sequence for all sensor modules
 * used in the weather station system. It should be called during system startup
 * before attempting to read any sensor data.
 *
 * @return void
 */

void init_sensors(void) {
  if (i2c_init() == ESP_OK) {
    ESP_LOGI(TAG, "I2C initialized");

#if (CONFIG_ANEMOMETER_TYPE_ADS1115 || CONFIG_WIND_VANE_TYPE_ADS1115)
    if (ads1115_init(&ads1115_handle) == ESP_OK) {
      ESP_LOGI(TAG, "ADS1115 initialized");
    } else {
      ESP_LOGE(TAG, "ADS1115 initialization failed");
      return;
    }
#endif

#if CONFIG_ILLUMINANCE_SENSOR_TYPE_BH1750
    if (bh1750_init(&bh1750_handle) == ESP_OK) {
      ESP_LOGI(TAG, "BH1750 initialized");
    } else {
      ESP_LOGE(TAG, "BH1750 initialization failed");
      return;
    }
#endif

#if (CONFIG_HUMIDITY_SENSOR_TYPE_BME280 || CONFIG_PRESSURE_SENSOR_TYPE_BME280 || CONFIG_TEMPERATURE_SENSOR_TYPE_BME280)
    if (bme280_init(&bme280_handle) == ESP_OK) {
      ESP_LOGI(TAG, "BME280 initialized");
    } else {
      ESP_LOGE(TAG, "BME280 initialization failed");
      return;
    }
#endif

    init_tasks();
  } else {
    ESP_LOGE(TAG, "I2C initialization failed");
    return;
  }
}

/**
 * @brief Initializes and creates all system tasks for the weather station.
 *
 * This function sets up and starts all necessary FreeRTOS tasks required for
 * the weather station operation. Tasks may include sensor reading, data processing,
 * communication handling, and other system-level operations.
 *
 * @note This function should be called once during system initialization,
 *       typically after hardware peripherals have been configured.
 * @note Ensure sufficient heap memory is available for task creation.
 *
 * @return void
 */

void init_tasks(void) {
#if CONFIG_ANEMOMETER_TYPE_ADS1115
  #if CONFIG_DEBUG_MODE
  create_task(anemometer_task, "anemometer", 2048, 1, &ads1115_handle);
  #else
  create_task(anemometer_task, "anemometer", 1024, 1, &ads1115_handle);
  #endif
#endif

#if CONFIG_ILLUMINANCE_SENSOR_TYPE_BH1750
  #if CONFIG_DEBUG_MODE
  create_task(bh1750_task, "bh1750", 2048, 1, &bh1750_handle);
  #else
  create_task(bh1750_task, "bh1750", 1024, 1, &bh1750_handle);
  #endif
#endif

#if (CONFIG_HUMIDITY_SENSOR_TYPE_BME280 || CONFIG_PRESSURE_SENSOR_TYPE_BME280 || CONFIG_TEMPERATURE_SENSOR_TYPE_BME280)
  #if CONFIG_DEBUG_MODE
  create_task(bme280_task, "bme280", 2048, 1, &bme280_handle);
  #else
  create_task(bme280_task, "bme280", 1024, 1, &bme280_handle);
  #endif
#endif

#if CONFIG_WIND_VANE_TYPE_ADS1115
  #if CONFIG_DEBUG_MODE
  create_task(wind_vane_task, "wind_vane", 2048, 1, &ads1115_handle);
  #else
  create_task(wind_vane_task, "wind_vane", 1024, 1, &ads1115_handle);
  #endif
#endif
}

/**
 * @brief Main application entry point for the weather station.
 *
 * This function serves as the entry point for the ESP-IDF application.
 * It initializes all necessary components and starts the main application logic
 * for the weather station system.
 *
 * @note This function is called automatically by the ESP-IDF framework after
 *       system initialization is complete.
 *
 * @return void This function does not return as it contains the main application loop.
 */

void app_main(void) {
#if !CONFIG_DEBUG_MODE
  esp_log_level_set("*", ESP_LOG_NONE);
#endif

#if CONFIG_ZIGBEE_COMMUNICATION_ENABLED
  if (zigbee_init() == ESP_OK) {
    ESP_LOGI(TAG, "Zigbee initialized");
  } else {
    ESP_LOGE(TAG, "ZigBee initialization failed");
    return;
  }
#else
  init_sensors();
#endif
}
