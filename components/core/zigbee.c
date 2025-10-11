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

#include "zigbee.h"

#include "freertos/FreeRTOS.h"
#include "macros.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#if CONFIG_ILLUMINANCE_SENSOR_TYPE_BH1750
  #include "bh1750.h"
#endif

#if (CONFIG_HUMIDITY_SENSOR_TYPE_BME280 || CONFIG_PRESSURE_SENSOR_TYPE_BME280 || CONFIG_TEMPERATURE_SENSOR_TYPE_BME280)
  #include "bme280.h"
#endif

static const char* TAG = "ZIGBEE";

/**
 * @brief Callback function invoked when top-level commissioning starts
 *
 * This callback is triggered by the Zigbee Base Device Behavior (BDB) specification
 * when the device begins the commissioning process. It indicates which commissioning
 * modes are being attempted.
 *
 * @param mode_mask Bitmask indicating the commissioning mode(s) being started.
 *                  Can be a combination of:
 *                  - BDB_COMMISSIONING_MODE_INITIALIZATION
 *                  - BDB_COMMISSIONING_MODE_NWK_STEERING
 *                  - BDB_COMMISSIONING_MODE_NWK_FORMATION
 *                  - BDB_COMMISSIONING_MODE_FINDING_BINDING
 *                  - BDB_COMMISSIONING_MODE_TOUCHLINK
 *
 * @return None
 */

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
  ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

/**
 * @brief Zigbee task function that runs in a separate FreeRTOS task
 *
 * This function serves as the main task handler for Zigbee operations.
 * It is typically used to initialize and manage Zigbee stack operations
 * in a dedicated FreeRTOS task context.
 *
 * @param[in] pvParameters Pointer to task parameters (as per FreeRTOS task convention)
 *
 * @return None (task function does not return)
 */

static void esp_zb_task(void* pvParameters) {
  esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
  esp_zb_init(&zb_nwk_cfg);

  // BASIC cluster
  esp_zb_basic_cluster_cfg_t basic_cluster_cfg = { .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
#if CONFIG_ZIGBEE_POWER_SOURCE_DC
                                                   .power_source = 0x04
#endif
  };

  uint32_t ApplicationVersion = 0x000100;
  uint32_t StackVersion = 0x010607;
  uint32_t HWVersion = 0x000001;
  P_STRING(ManufacturerName, "Ozone");
  P_STRING(ModelIdentifier, "Weather Station");
  P_STRING(DateCode, "20251005");
  esp_zb_attribute_list_t* esp_zb_basic_cluster = esp_zb_basic_cluster_create(&basic_cluster_cfg);
  esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID,
                                &ApplicationVersion);
  esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID, &StackVersion);
  esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_HW_VERSION_ID, &HWVersion);
  esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
                                (void*)&ManufacturerName);
  esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
                                (void*)&ModelIdentifier);
  esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, (void*)&DateCode);

  // IDENTIFY cluster
  esp_zb_identify_cluster_cfg_t identify_cluster_cfg = { .identify_time =
                                                             ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE };
  esp_zb_attribute_list_t* esp_zb_identify_cluster = esp_zb_identify_cluster_create(&identify_cluster_cfg);

  // ANEMOMETER cluster
#if CONFIG_ANEMOMETER_ENABLED
  esp_zb_wind_speed_measurement_cluster_cfg_t wind_speed_meas_cluster_cfg = {
    .measured_value = 0xFFFF, .min_measured_value = 0, .max_measured_value = CONFIG_ANEMOMETER_MAX_RANGE * 100
  };
  esp_zb_attribute_list_t* esp_zb_wind_speed_meas_cluster =
      esp_zb_wind_speed_measurement_cluster_create(&wind_speed_meas_cluster_cfg);
#endif

// ILLUMINANCE cluster
#if CONFIG_ILLUMINANCE_SENSOR_ENABLED
  esp_zb_illuminance_meas_cluster_cfg_t illuminance_meas_cluster_cfg = {
    .measured_value = 0xFFFF,
  #if CONFIG_ILLUMINANCE_SENSOR_TYPE_BH1750
    .min_value = (10000 * log10f(BH1750_ILLUMINANCE_MEAS_MIN_VALUE) + 1),
  #endif

  #if CONFIG_ILLUMINANCE_SENSOR_TYPE_BH1750
    .max_value = (10000 * log10f(BH1750_ILLUMINANCE_MEAS_MAX_VALUE) + 1)
  #endif
  };
  esp_zb_attribute_list_t* esp_zb_illuminance_meas_cluster =
      esp_zb_illuminance_meas_cluster_create(&illuminance_meas_cluster_cfg);
#endif

// HUMIDITY cluster
#if CONFIG_HUMIDITY_SENSOR_ENABLED
  esp_zb_humidity_meas_cluster_cfg_t humidity_meas_cluster_cfg = { .measured_value = 0xFFFF,
  #if CONFIG_HUMIDITY_SENSOR_TYPE_BME280
                                                                   .min_value = BME280_HUMIDITY_MEAS_MIN_VALUE * 100,
  #endif

  #if CONFIG_HUMIDITY_SENSOR_TYPE_BME280
                                                                   .max_value = BME280_HUMIDITY_MEAS_MAX_VALUE * 100
  #endif
  };
  esp_zb_attribute_list_t* esp_zb_humidity_meas_cluster =
      esp_zb_humidity_meas_cluster_create(&humidity_meas_cluster_cfg);
#endif

// PRESSURE cluster
#if CONFIG_PRESSURE_SENSOR_ENABLED
  esp_zb_pressure_meas_cluster_cfg_t pressure_meas_cluster_cfg = { .measured_value = 0xFFFF,
  #if CONFIG_PRESSURE_SENSOR_TYPE_BME280
                                                                   .min_value = BME280_PRESSURE_MEAS_MIN_VALUE * 10,
  #endif

  #if CONFIG_PRESSURE_SENSOR_TYPE_BME280
                                                                   .max_value = BME280_PRESSURE_MEAS_MAX_VALUE * 10
  #endif
  };
  esp_zb_attribute_list_t* esp_zb_pressure_meas_cluster =
      esp_zb_pressure_meas_cluster_create(&pressure_meas_cluster_cfg);
#endif

  // TEMPERATURE cluster
#if CONFIG_TEMPERATURE_SENSOR_ENABLED
  esp_zb_temperature_meas_cluster_cfg_t temperature_meas_cluster_cfg = {
    .measured_value = 0xFFFF,
  #if CONFIG_TEMPERATURE_SENSOR_TYPE_BME280
    .min_value = BME280_TEMPERATURE_MEAS_MIN_VALUE * 100,
  #endif

  #if CONFIG_TEMPERATURE_SENSOR_TYPE_BME280
    .max_value = BME280_TEMPERATURE_MEAS_MAX_VALUE * 100
  #endif
  };
  esp_zb_attribute_list_t* esp_zb_temperature_meas_cluster =
      esp_zb_temperature_meas_cluster_create(&temperature_meas_cluster_cfg);
#endif

  // WIND VANE cluster
#if CONFIG_WIND_VANE_ENABLED
  esp_zb_analog_input_cluster_cfg_t wind_vane_cluster_cfg = { .present_value = 0.0f,
                                                              .out_of_service = false,
                                                              .status_flags = 0 };
  esp_zb_attribute_list_t* esp_zb_wind_vane_cluster = esp_zb_analog_input_cluster_create(&wind_vane_cluster_cfg);

  float wind_vane_zb_min_value = 0.0f;
  float wind_vane_zb_max_value = 360.0f;
  uint8_t wind_vane_zb_resolution = 1;
  esp_zb_analog_input_cluster_add_attr(esp_zb_wind_vane_cluster, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_MIN_PRESENT_VALUE_ID,
                                       &wind_vane_zb_min_value);
  esp_zb_analog_input_cluster_add_attr(esp_zb_wind_vane_cluster, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_MAX_PRESENT_VALUE_ID,
                                       &wind_vane_zb_max_value);
  esp_zb_analog_input_cluster_add_attr(esp_zb_wind_vane_cluster, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_RESOLUTION_ID,
                                       &wind_vane_zb_resolution);
#endif

  // Creating the cluster list for the endpoint
  esp_zb_cluster_list_t* esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
  esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster,
                                           ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

#if CONFIG_ANEMOMETER_ENABLED
  esp_zb_cluster_list_add_wind_speed_measurement_cluster(esp_zb_cluster_list, esp_zb_wind_speed_meas_cluster,
                                                         ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
#endif

#if CONFIG_HUMIDITY_SENSOR_ENABLED
  esp_zb_cluster_list_add_humidity_meas_cluster(esp_zb_cluster_list, esp_zb_humidity_meas_cluster,
                                                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
#endif

#if CONFIG_ILLUMINANCE_SENSOR_ENABLED
  esp_zb_cluster_list_add_illuminance_meas_cluster(esp_zb_cluster_list, esp_zb_illuminance_meas_cluster,
                                                   ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
#endif

#if CONFIG_PRESSURE_SENSOR_ENABLED
  esp_zb_cluster_list_add_pressure_meas_cluster(esp_zb_cluster_list, esp_zb_pressure_meas_cluster,
                                                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
#endif

#if CONFIG_TEMPERATURE_SENSOR_ENABLED
  esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list, esp_zb_temperature_meas_cluster,
                                                   ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
#endif

#if CONFIG_WIND_VANE_ENABLED
  esp_zb_cluster_list_add_analog_input_cluster(esp_zb_cluster_list, esp_zb_wind_vane_cluster,
                                               ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
#endif

  // Endpoint configuration
  esp_zb_ep_list_t* esp_zb_ep_list = esp_zb_ep_list_create();
  esp_zb_endpoint_config_t endpoint_config = { .endpoint = 1,
                                               .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
                                               .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID };

  esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_config);

  // Register device
  esp_zb_device_register(esp_zb_ep_list);
  esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);
  ESP_ERROR_CHECK(esp_zb_start(false));
  esp_zb_stack_main_loop();
}

/**
 * @brief Zigbee application signal handler callback function.
 *
 * This function handles various Zigbee stack signals and events during the
 * application lifecycle. It processes signals such as device startup,
 * network formation, joining, network steering, and other stack events.
 *
 * @param[in] signal_struct Pointer to the Zigbee application signal structure
 *                          containing signal type and associated data.
 *
 * @return void
 *
 * @note This function should be registered with the Zigbee stack to receive
 *       application signals and events.
 * @note Typical signals include: skip startup, device reboot, network formation,
 *       network steering, connection status, etc.
 */

void esp_zb_app_signal_handler(esp_zb_app_signal_t* signal_struct) {
  uint32_t* p_sg_p = signal_struct->p_app_signal;
  esp_err_t err_status = signal_struct->esp_err_status;
  esp_zb_app_signal_type_t sig_type = *p_sg_p;

  switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
      if (err_status == ESP_OK)
        ESP_LOGI(TAG, "Production config loaded");
      else
        ESP_LOGI(TAG, "No production config (using runtime network data)");
      break;
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
      esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
      break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
      if (err_status == ESP_OK) {
        ESP_LOGI(TAG, "Device started successfully");
        if (esp_zb_bdb_is_factory_new()) {
          ESP_LOGI(TAG, "Starting network commissioning");
          esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
          ESP_LOGI(TAG, "Device not factory new, network data preserved");
          init_sensors();
        }
      } else {
        ESP_LOGE(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
      }
      break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
      if (err_status == ESP_OK) {
        esp_zb_ieee_addr_t extended_pan_id;
        esp_zb_get_extended_pan_id(extended_pan_id);
        ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x)",
                 extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4], extended_pan_id[3],
                 extended_pan_id[2], extended_pan_id[1], extended_pan_id[0]);
        init_sensors();
      } else {
        ESP_LOGW(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                               ESP_ZB_BDB_MODE_NETWORK_STEERING, 2750);
      }
      break;

    case ESP_ZB_NLME_STATUS_INDICATION:
      ESP_LOGI(TAG, "Network status indication");
      break;
    default:
      ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
               esp_err_to_name(err_status));
      break;
  }
}

/**
 * @brief Initialize the Zigbee component
 *
 * This function initializes the Zigbee stack and prepares it for operation.
 * It should be called before any other Zigbee operations are performed.
 *
 * @return esp_err_t ESP_OK on success, error code otherwise
 */

esp_err_t zigbee_init(void) {
  esp_zb_platform_config_t config = { .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
                                      .host_config = ESP_ZB_DEFAULT_HOST_CONFIG() };

  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_zb_platform_config(&config));

#if CONFIG_DEBUG_MODE
  xTaskCreate(esp_zb_task, "zigbee", 8192, NULL, 5, NULL);
#else
  xTaskCreate(esp_zb_task, "zigbee", 4096, NULL, 5, NULL);
#endif

  return ESP_OK;
}
