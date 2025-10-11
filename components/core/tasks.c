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

#include "tasks.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char* TAG = "TASK";

#if CONFIG_ANEMOMETER_ENABLED
  #include "anemometer.h"
#endif

#if CONFIG_ZIGBEE_COMMUNICATION_ENABLED
  #include "esp_zigbee_core.h"
#endif

#if (CONFIG_HUMIDITY_SENSOR_ENABLED || CONFIG_PRESSURE_SENSOR_ENABLED || CONFIG_TEMPERATURE_SENSOR_ENABLED)
  #include "bme280.h"
#endif

#if CONFIG_ILLUMINANCE_SENSOR_TYPE_BH1750
  #include "bh1750.h"
#endif

#if CONFIG_WIND_VANE_ENABLED
  #include "wind_vane.h"
#endif

/**
 * @brief Task function for monitoring and processing anemometer data.
 *
 * This task handles the anemometer sensor readings, which measure wind speed.
 * It runs continuously as a FreeRTOS task and performs periodic measurements
 * and calculations based on anemometer input.
 *
 * @param pvParameters Pointer to task parameters (typically unused or cast to specific type).
 *                     This is a standard FreeRTOS task parameter that can be used
 *                     to pass configuration data to the task at creation time.
 *
 * @note This function should never return and must contain an infinite loop.
 * @note The task should be created using xTaskCreate() or similar FreeRTOS API.
 */

#if CONFIG_ANEMOMETER_ENABLED
void anemometer_task(void* pvParameters) {
  i2c_master_dev_handle_t* ads1115_handle = (i2c_master_dev_handle_t*)pvParameters;
  uint8_t wind_speed_ms;

  while (1) {
    if (get_wind_speed(&wind_speed_ms, ads1115_handle) == ESP_OK) {
  #if CONFIG_ANEMOMETER_CONVERSION_UNIT_KMH
      ESP_LOGI(TAG, "Wind speed: %d km/h", (uint16_t)(wind_speed_ms * 3.6f + 0.5f));
  #else
      ESP_LOGI(TAG, "Wind speed: %d mp/h", (uint16_t)(wind_speed_ms * 2.237f + 0.5f));
  #endif

  #if CONFIG_ZIGBEE_COMMUNICATION_ENABLED
      uint16_t wind_speed_ms_zb = (uint16_t)wind_speed_ms * 100;
      esp_zb_zcl_set_attribute_val(1, ESP_ZB_ZCL_CLUSTER_ID_WIND_SPEED_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                   ESP_ZB_ZCL_ATTR_WIND_SPEED_MEASUREMENT_MEASURED_VALUE_ID, &wind_speed_ms_zb, false);
  #endif
    } else {
      ESP_LOGE(TAG, "ADS1115 read error");
    }

    vTaskDelay(pdMS_TO_TICKS(CONFIG_ANEMOMETER_READ_INTERVAL_MS));
  }
}
#endif

/**
 * @brief Task function for reading data from BH1750 light sensor
 *
 * This task continuously reads ambient light intensity from the BH1750 sensor
 * and processes the data according to the configured measurement mode.
 *
 * @param pvParameters Pointer to task parameters (unused, can be NULL)
 *
 * @note This is a FreeRTOS task function that should be created using xTaskCreate()
 * @note The task runs indefinitely until explicitly deleted
 */

#if CONFIG_ILLUMINANCE_SENSOR_TYPE_BH1750
void bh1750_task(void* pvParameters) {
  i2c_master_dev_handle_t* bh1750_handle = (i2c_master_dev_handle_t*)pvParameters;
  float lux;

  while (1) {
    if (bh1750_read_illuminance(&lux, bh1750_handle) == ESP_OK) {
      ESP_LOGI(TAG, "Illuminance: %.2f lux", lux);

  #if CONFIG_ZIGBEE_COMMUNICATION_ENABLED
      uint16_t lux_zb = (uint16_t)(10000 * log10f(lux) + 1);
      esp_zb_zcl_set_attribute_val(1, ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                   ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID, &lux_zb, false);
  #endif
    }

    vTaskDelay(pdMS_TO_TICKS(CONFIG_BH1750_READ_INTERVAL_MS));
  }
}
#endif

/**
 * @brief FreeRTOS task for handling BME280 sensor operations.
 *
 * This task continuously reads data from the BME280 temperature, humidity,
 * and pressure sensor. It typically runs in a loop, reading sensor values
 * at regular intervals and processing or transmitting the data as needed.
 *
 * @param pvParameters Pointer to task parameters (not used in this implementation,
 *                     can be NULL or point to task-specific configuration data)
 *
 * @return This function does not return as it runs continuously in the FreeRTOS
 *         task scheduler. If the task needs to be terminated, it should call
 *         vTaskDelete(NULL) internally.
 */

#if (CONFIG_HUMIDITY_SENSOR_ENABLED || CONFIG_PRESSURE_SENSOR_ENABLED || CONFIG_TEMPERATURE_SENSOR_ENABLED)
void bme280_task(void* pvParameters) {
  i2c_master_dev_handle_t* bme280_handle = (i2c_master_dev_handle_t*)pvParameters;
  bme280_data_t data = { 0 };

  while (1) {
    if (bme280_read_values(&data, bme280_handle) == ESP_OK) {
  #if CONFIG_TEMPERATURE_SENSOR_TYPE_BME280
      int16_t temperature = (int16_t)data.temperature;
      ESP_LOGI(TAG, "Temperature: %.2f °C", (temperature / 100.0f));

    #if CONFIG_ZIGBEE_COMMUNICATION_ENABLED
      esp_zb_zcl_set_attribute_val(1, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                   ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &temperature, false);
    #endif
  #endif

  #if CONFIG_HUMIDITY_SENSOR_TYPE_BME280
      int16_t humidity = (int16_t)((data.humidity * 100) / 1024);
      ESP_LOGI(TAG, "Humidity: %.2f %%", (humidity / 100.0f));

    #if CONFIG_ZIGBEE_COMMUNICATION_ENABLED
      esp_zb_zcl_set_attribute_val(1, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                   ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &humidity, false);
    #endif
  #endif

  #if CONFIG_PRESSURE_SENSOR_TYPE_BME280
      int16_t pressure = (int16_t)(data.pressure / 10);
      ESP_LOGI(TAG, "Pressure: %.1f kPa", (pressure / 10.0f));

    #if CONFIG_ZIGBEE_COMMUNICATION_ENABLED
      esp_zb_zcl_set_attribute_val(1, ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                   ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID, &pressure, false);
    #endif
  #endif
    }

    vTaskDelay(pdMS_TO_TICKS(CONFIG_BME280_READ_INTERVAL_MS));
  }
}
#endif

/**
 * @brief FreeRTOS task for reading and processing wind vane sensor data.
 *
 * This task continuously monitors the wind vane to determine wind direction.
 * It typically reads analog values from the wind vane sensor and converts them
 * to directional readings.
 *
 * @param pvParameters Pointer to task parameters (typically NULL or task-specific configuration)
 *
 * @note This is a FreeRTOS task function and should be created using xTaskCreate()
 * @note The task should contain an infinite loop and use appropriate delays
 */

#if CONFIG_WIND_VANE_TYPE_ADS1115
void wind_vane_task(void* pvParameters) {
  i2c_master_dev_handle_t* ads1115_handle = (i2c_master_dev_handle_t*)pvParameters;
  const char* direction;

  while (1) {
    if (get_wind_direction(&direction, ads1115_handle) == ESP_OK) {
      float degrees;

      if (direction_to_degrees(direction, &degrees) == ESP_OK) {
        ESP_LOGI(TAG, "Wind vane - Direction: %s, degrees: %.1f", direction, degrees);

  #if CONFIG_ZIGBEE_COMMUNICATION_ENABLED
        esp_zb_zcl_set_attribute_val(1, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                     ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &degrees, false);
  #endif
      } else {
        ESP_LOGE(TAG, "Impossible to determine the angle of orientation");
      }
    } else {
      ESP_LOGE(TAG, "ADS1115 read error");
    }

    vTaskDelay(pdMS_TO_TICKS(CONFIG_WIND_VANE_READ_INTERVAL_MS));
  }
}
#endif

/**
 * @brief Creates and starts a FreeRTOS task
 *
 * @param task_func Pointer to the task function to be executed
 * @param name Descriptive name for the task (for debugging purposes)
 * @param stack_size Stack depth allocated for the task in bytes
 * @param priority Priority at which the task will run (0 being lowest)
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */

esp_err_t create_task(TaskFunction_t task_func, const char* name, uint32_t stack_size, UBaseType_t priority,
                      void* pvParameters) {
  if (xTaskCreate(task_func, name, stack_size, pvParameters, priority, NULL) != pdPASS) {
    ESP_LOGE(TAG, "FATAL: Unable to create task '%s'", name);
    ESP_LOGE(TAG, "Insufficient memory or system error");

    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Task '%s' created (stack: %lu)", name, stack_size);

  return ESP_OK;
}
