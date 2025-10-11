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

#ifndef __ZIGBEE_H__
#define __ZIGBEE_H__

#include "esp_err.h"
#include "esp_zigbee_core.h"

#define INSTALLCODE_POLICY_ENABLE false
#define ED_AGING_TIMEOUT          ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE             3000

#define ESP_ZB_DEFAULT_RADIO_CONFIG() { .radio_mode = ZB_RADIO_MODE_NATIVE }
#define ESP_ZB_DEFAULT_HOST_CONFIG()  { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE }

#define ESP_ZB_ZED_CONFIG()                                                                                      \
  {                                                                                                              \
    .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED, .install_code_policy = INSTALLCODE_POLICY_ENABLE, .nwk_cfg.zed_cfg = { \
      .ed_timeout = ED_AGING_TIMEOUT,                                                                            \
      .keep_alive = ED_KEEP_ALIVE                                                                                \
    }                                                                                                            \
  }

extern void init_sensors(void);

void esp_zb_app_signal_handler(esp_zb_app_signal_t* signal_struct);
esp_err_t zigbee_init(void);

#endif /* __ZIGBEE_H__ */
