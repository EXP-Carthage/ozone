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

#include "bme280.h"

#include "i2c.h"
#include "sdkconfig.h"

static uint8_t h1, h3;
static int8_t h6;
static int16_t h2, h4, h5, p2, p3, p4, p5, p6, p7, p8, p9, t2, t3;
static uint16_t p1, t1;

#if CONFIG_BME280_MODE_SLEEP
uint8_t mode = BME280_MODE_SLEEP;
#elif CONFIG_BME280_MODE_NORMAL
uint8_t mode = BME280_MODE_NORMAL;
#else
uint8_t mode = BME280_MODE_FORCED;
#endif

#if CONFIG_BME280_PRESSURE_NO_OVERSAMPLING
uint8_t press_oss = BME280_NO_OVERSAMPLING;
#elif CONFIG_BME280_PRESSURE_OVERSAMPLING_X1
uint8_t press_oss = BME280_OVERSAMPLING_X1;
#elif CONFIG_BME280_PRESSURE_OVERSAMPLING_X2
uint8_t press_oss = BME280_OVERSAMPLING_X2;
#elif CONFIG_BME280_PRESSURE_OVERSAMPLING_X4
uint8_t press_oss = BME280_OVERSAMPLING_X4;
#elif CONFIG_BME280_PRESSURE_OVERSAMPLING_X8
uint8_t press_oss = BME280_OVERSAMPLING_X8;
#else
uint8_t press_oss = BME280_OVERSAMPLING_X16;
#endif

#if CONFIG_BME280_TEMPERATURE_NO_OVERSAMPLING
uint8_t temp_oss = BME280_NO_OVERSAMPLING;
#elif CONFIG_BME280_TEMPERATURE_OVERSAMPLING_X1
uint8_t temp_oss = BME280_OVERSAMPLING_X1;
#elif CONFIG_BME280_TEMPERATURE_OVERSAMPLING_X2
uint8_t temp_oss = BME280_OVERSAMPLING_X2;
#elif CONFIG_BME280_TEMPERATURE_OVERSAMPLING_X4
uint8_t temp_oss = BME280_OVERSAMPLING_X4;
#elif CONFIG_BME280_TEMPERATURE_OVERSAMPLING_X8
uint8_t temp_oss = BME280_OVERSAMPLING_X8;
#else
uint8_t temp_oss = BME280_OVERSAMPLING_X16;
#endif

/**
 * @brief Compensate the raw humidity value from the BME280 sensor
 *
 * This function converts the raw ADC humidity reading into a compensated
 * humidity value using the sensor's calibration parameters and the fine
 * temperature value.
 *
 * @param adc_H Raw ADC humidity value read from the sensor
 * @param t_fine Fine temperature value obtained from temperature compensation,
 *               used to improve humidity calculation accuracy
 *
 * @return Compensated humidity value in Q22.10 format (humidity in %RH * 1024)
 *         The value should be divided by 1024 to get the actual humidity percentage
 */

static uint32_t compensate_humidity(int32_t adc_H, int32_t t_fine) {
  int32_t v_x1_u32r;

  v_x1_u32r = (t_fine - ((int32_t)76800));
  v_x1_u32r =
      (((((adc_H << 14) - (((int32_t)h4) << 20) - (((int32_t)h5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
       (((((((v_x1_u32r * ((int32_t)h6)) >> 10) * (((v_x1_u32r * ((int32_t)h3)) >> 11) + ((int32_t)32768))) >> 10) +
          ((int32_t)2097152)) *
             ((int32_t)h2) +
         8192) >>
        14));
  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)h1)) >> 4));
  v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
  v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

  return (uint32_t)(v_x1_u32r >> 12);
}

/**
 * @brief Compensates the raw pressure ADC value from the BME280 sensor.
 *
 * This function applies the temperature compensation and calibration parameters
 * to convert the raw ADC pressure reading into a compensated pressure value.
 * The compensation formula uses the fine temperature value (t_fine) which must
 * be calculated first using the temperature compensation function.
 *
 * @param adc_P Raw pressure ADC value read from the BME280 sensor registers.
 * @param t_fine Fine temperature value obtained from temperature compensation.
 *              This value is required for accurate pressure compensation.
 *
 * @return Compensated pressure value in Pa (Pascals) as an unsigned 32-bit integer.
 *         The value is in Q24.8 format (24 integer bits, 8 fractional bits).
 *         Divide by 256 to get the actual pressure in Pa.
 */

static uint32_t compensate_pressure(int32_t adc_P, int32_t t_fine) {
  int32_t var1, var2;
  uint32_t p;

  var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)p6);
  var2 = var2 + ((var1 * ((int32_t)p5)) << 1);
  var2 = (var2 >> 2) + (((int32_t)p4) << 16);
  var1 = (((p3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)p2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((int32_t)p1)) >> 15);

  if (var1 == 0) return 0;

  p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
  if (p < 0x80000000)
    p = (p << 1) / ((uint32_t)var1);
  else
    p = (p / (uint32_t)var1) * 2;

  var1 = (((int32_t)p9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
  var2 = (((int32_t)(p >> 2)) * ((int32_t)p8)) >> 13;
  p = (uint32_t)((int32_t)p + ((var1 + var2 + p7) >> 4));

  return p;
}

/**
 * @brief Compensates the raw ADC temperature value from BME280 sensor
 *
 * This function converts the raw ADC temperature reading from the BME280 sensor
 * into a compensated temperature value using the calibration parameters stored
 * in the device. The compensation algorithm follows the BME280 datasheet specifications.
 *
 * @param adc_T Raw ADC temperature value read from the BME280 sensor
 *
 * @return int32_t Compensated temperature value in hundredths of degrees Celsius
 *                 (e.g., a return value of 5123 represents 51.23°C)
 */

static int32_t compensate_temperature(int32_t adc_T) {
  int32_t var1, var2, t;
  var1 = ((((adc_T >> 3) - ((int32_t)t1 << 1))) * ((int32_t)t2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)t1)) * ((adc_T >> 4) - ((int32_t)t1))) >> 12) * ((int32_t)t3)) >> 14;
  t = var1 + var2;

  return t;
}

/**
 * @brief Reads calibration data from the BME280 sensor.
 *
 * This function reads the calibration coefficients stored in the BME280 sensor's
 * non-volatile memory. These coefficients are used to compensate the raw sensor
 * readings for temperature, pressure, and humidity measurements.
 *
 * @param[in] dev_handle Pointer to the I2C master device handle for the BME280 sensor.
 *
 * @return
 *     - ESP_OK: Success, calibration data read successfully
 *     - ESP_ERR_INVALID_ARG: Invalid argument (null pointer)
 *     - ESP_FAIL: I2C communication failure
 *     - Other ESP_ERR_* codes from I2C driver operations
 */

static esp_err_t read_calibration(i2c_master_dev_handle_t* dev_handle) {
  uint8_t h4_msb, h4_lsb_h5_msb, h5_lsb;

  ESP_ERROR_CHECK(i2c_read_reg16_LE(BME280_REG_DIG_T1, &t1, dev_handle));
  ESP_ERROR_CHECK(i2c_read_regS16_LE(BME280_REG_DIG_T2, &t2, dev_handle));
  ESP_ERROR_CHECK(i2c_read_regS16_LE(BME280_REG_DIG_T3, &t3, dev_handle));
  ESP_ERROR_CHECK(i2c_read_reg16_LE(BME280_REG_DIG_P1, &p1, dev_handle));
  ESP_ERROR_CHECK(i2c_read_regS16_LE(BME280_REG_DIG_P2, &p2, dev_handle));
  ESP_ERROR_CHECK(i2c_read_regS16_LE(BME280_REG_DIG_P3, &p3, dev_handle));
  ESP_ERROR_CHECK(i2c_read_regS16_LE(BME280_REG_DIG_P4, &p4, dev_handle));
  ESP_ERROR_CHECK(i2c_read_regS16_LE(BME280_REG_DIG_P5, &p5, dev_handle));
  ESP_ERROR_CHECK(i2c_read_regS16_LE(BME280_REG_DIG_P6, &p6, dev_handle));
  ESP_ERROR_CHECK(i2c_read_regS16_LE(BME280_REG_DIG_P7, &p7, dev_handle));
  ESP_ERROR_CHECK(i2c_read_regS16_LE(BME280_REG_DIG_P8, &p8, dev_handle));
  ESP_ERROR_CHECK(i2c_read_regS16_LE(BME280_REG_DIG_P9, &p9, dev_handle));
  ESP_ERROR_CHECK(i2c_read_reg8(BME280_REG_DIG_H1, &h1, dev_handle));
  ESP_ERROR_CHECK(i2c_read_regS16_LE(BME280_REG_DIG_H2, &h2, dev_handle));
  ESP_ERROR_CHECK(i2c_read_reg8(BME280_REG_DIG_H3, &h3, dev_handle));
  ESP_ERROR_CHECK(i2c_read_reg8(BME280_REG_DIG_H4, &h4_msb, dev_handle));
  ESP_ERROR_CHECK(i2c_read_reg8(BME280_REG_DIG_H5_MSB, &h4_lsb_h5_msb, dev_handle));
  ESP_ERROR_CHECK(i2c_read_reg8(BME280_REG_DIG_H5_LSB, &h5_lsb, dev_handle));
  ESP_ERROR_CHECK(i2c_read_reg8(BME280_REG_DIG_H6, (uint8_t*)&h6, dev_handle));

  h4 = (int16_t)((h4_msb << 4) | (h4_lsb_h5_msb & 0x0F));
  h5 = (int16_t)((h5_lsb << 4) | (h4_lsb_h5_msb >> 4));

  return ESP_OK;
}

/**
 * @brief Initialize the BME280 sensor
 *
 * This function initializes the BME280 temperature, pressure, and humidity sensor
 * by configuring the I2C communication interface and setting up the device handle.
 *
 * @param[in,out] dev_handle Pointer to the I2C master device handle for the BME280 sensor
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer)
 *     - ESP_FAIL: Initialization failed
 *     - Other ESP error codes depending on I2C communication status
 *
 * @note The I2C bus must be initialized before calling this function
 * @note The dev_handle will be populated with the initialized device handle on success
 */

esp_err_t bme280_init(i2c_master_dev_handle_t* dev_handle) {
  if (dev_handle == NULL) return ESP_ERR_INVALID_ARG;

#if CONFIG_BME280_I2C_ADDRESS_SDO_TO_GND
  uint8_t dev_addr = BME280_ADDRESS_SDO_TO_GND;
#else
  uint8_t dev_addr = BME280_ADDRESS_SDO_TO_VDD;
#endif

#if CONFIG_I2C_FREQ_STANDARD
  uint32_t scl_freq = I2C_FREQ_100KHZ_STANDARD;
#else
  uint32_t scl_freq = I2C_FREQ_400KHZ_FAST;
#endif

#if CONFIG_BME280_STANDBY_0_5_MS
  uint8_t standby_time = BME280_STANDBY_0_5_MS;
#elif CONFIG_BME280_STANDBY_10_MS
  uint8_t standby_time = BME280_STANDBY_10_MS;
#elif CONFIG_BME280_STANDBY_20_MS
  uint8_t standby_time = BME280_STANDBY_20_MS;
#elif CONFIG_BME280_STANDBY_62_5_MS
  uint8_t standby_time = BME280_STANDBY_62_5_MS;
#elif CONFIG_BME280_STANDBY_125_MS
  uint8_t standby_time = BME280_STANDBY_125_MS;
#elif CONFIG_BME280_STANDBY_250_MS
  uint8_t standby_time = BME280_STANDBY_250_MS;
#elif CONFIG_BME280_STANDBY_500_MS
  uint8_t standby_time = BME280_STANDBY_500_MS;
#else
  uint8_t standby_time = BME280_STANDBY_1000_MS;
#endif

#if CONFIG_BME280_IIR_FILTER_OFF
  uint8_t iir_filter = BME280_IIR_FILTER_OFF;
#elif CONFIG_BME280_IIR_FILTER_X2
  uint8_t iir_filter = BME280_IIR_FILTER_X2;
#elif CONFIG_BME280_IIR_FILTER_X4
  uint8_t iir_filter = BME280_IIR_FILTER_X4;
#elif CONFIG_BME280_IIR_FILTER_X8
  uint8_t iir_filter = BME280_IIR_FILTER_X8;
#else
  uint8_t iir_filter = BME280_IIR_FILTER_X16;
#endif

#if CONFIG_BME280_HUMIDITY_NO_OVERSAMPLING
  uint8_t hum_oss = BME280_NO_OVERSAMPLING;
#elif CONFIG_BME280_HUMIDITY_OVERSAMPLING_X1
  uint8_t hum_oss = BME280_OVERSAMPLING_X1;
#elif CONFIG_BME280_HUMIDITY_OVERSAMPLING_X2
  uint8_t hum_oss = BME280_OVERSAMPLING_X2;
#elif CONFIG_BME280_HUMIDITY_OVERSAMPLING_X4
  uint8_t hum_oss = BME280_OVERSAMPLING_X4;
#elif CONFIG_BME280_HUMIDITY_OVERSAMPLING_X8
  uint8_t hum_oss = BME280_OVERSAMPLING_X8;
#else
  uint8_t hum_oss = BME280_OVERSAMPLING_X16;
#endif

  ESP_ERROR_CHECK(i2c_add_device(dev_addr, scl_freq, dev_handle));
  ESP_ERROR_CHECK(read_calibration(dev_handle));

  uint8_t ctrl_meas = (temp_oss << 5) | (press_oss << 2) | mode;
  uint8_t config = (standby_time << 5) | (iir_filter << 2);
  ESP_ERROR_CHECK(i2c_write_reg8(BME280_REG_CTRL_HUM, hum_oss, dev_handle));
  ESP_ERROR_CHECK(i2c_write_reg8(BME280_REG_CTRL_MEAS, ctrl_meas, dev_handle));
  ESP_ERROR_CHECK(i2c_write_reg8(BME280_REG_CONFIG, config, dev_handle));

  uint8_t status;
  do {
    ESP_ERROR_CHECK(i2c_read_reg8(BME280_REG_STATUS, &status, dev_handle));
  } while (status & 0x01);

  return ESP_OK;
}

/**
 * @brief Read sensor values from the BME280 device
 *
 * This function reads temperature, pressure, and humidity values from the BME280
 * sensor through the I2C interface.
 *
 * @param[out] data Pointer to bme280_data_t structure where the sensor readings
 *                  will be stored
 * @param[in] dev_handle Pointer to the I2C master device handle for communication
 *                       with the BME280 sensor
 *
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid argument(s)
 *         - ESP_FAIL: Communication failure or sensor error
 */

esp_err_t bme280_read_values(bme280_data_t* data, i2c_master_dev_handle_t* dev_handle) {
  if (dev_handle == NULL || data == NULL) return ESP_ERR_INVALID_ARG;

#if CONFIG_BME280_MODE_FORCED
  uint8_t ctrl_meas = (temp_oss << 5) | (press_oss << 2) | mode;
  ESP_ERROR_CHECK(i2c_write_reg8(BME280_REG_CTRL_MEAS, ctrl_meas, dev_handle));

  uint8_t status;
  do {
    ESP_ERROR_CHECK(i2c_read_reg8(BME280_REG_STATUS, &status, dev_handle));
  } while (status & 0x08);
#endif

  uint8_t buffer[8];
  ESP_ERROR_CHECK(i2c_read_sequential_reg8(BME280_REG_PRESS_MSB, buffer, sizeof(buffer), dev_handle));

  int32_t adc_P = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);
  int32_t adc_T = ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4) | ((buffer[5] >> 4) & 0x0F);
  int32_t adc_H = ((uint32_t)buffer[6] << 8) | buffer[7];
  int32_t t_fine = compensate_temperature(adc_T);
  int32_t t = (t_fine * 5 + 128) >> 8;
  uint32_t p = compensate_pressure(adc_P, t_fine);
  uint32_t h = compensate_humidity(adc_H, t_fine);

  data->humidity = h;
  data->pressure = p;
  data->temperature = t;

  return ESP_OK;
}
