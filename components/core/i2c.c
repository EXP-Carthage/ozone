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

#include "i2c.h"

#include "sdkconfig.h"

i2c_master_bus_handle_t i2c_bus_handle;

/**
 * @brief Reads data from a register of an I2C device
 *
 * This function reads one or more bytes from a specified register address
 * of an I2C device using the ESP-IDF I2C master driver.
 *
 * @param[in] reg_addr The register address to read from
 * @param[out] data Pointer to buffer where read data will be stored
 * @param[in] length Number of bytes to read from the register
 * @param[in] dev_handle Pointer to the I2C master device handle
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument
 *     - ESP_FAIL: I2C communication failure
 *     - Other ESP-IDF error codes depending on I2C driver implementation
 */

static esp_err_t read_reg(uint8_t reg_addr, uint8_t* data, size_t length, i2c_master_dev_handle_t* dev_handle) {
  if (dev_handle == NULL || data == NULL) return ESP_ERR_INVALID_ARG;

  return i2c_master_transmit_receive(*dev_handle, &reg_addr, 1, data, length, -1);
}

/**
 * @brief Writes data to a register via I2C.
 *
 * @param data Pointer to the data buffer to be written
 * @param length Number of bytes to write from the data buffer
 * @param dev_handle Pointer to the I2C master device handle
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */

static esp_err_t write_reg(uint8_t* data, size_t length, i2c_master_dev_handle_t* dev_handle) {
  if (dev_handle == NULL || data == NULL) return ESP_ERR_INVALID_ARG;

  return i2c_master_transmit(*dev_handle, data, length, -1);
}

/**
 * @brief Initialize the I2C bus
 *
 * This function initializes the I2C peripheral with default configuration
 * for the weather station component.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */

esp_err_t i2c_init(void) {
  i2c_master_bus_config_t i2c_mst_config = { .clk_source = I2C_CLK_SRC_DEFAULT,
                                             .i2c_port = I2C_NUM_0,
                                             .scl_io_num = CONFIG_I2C_MASTER_SCL,
                                             .sda_io_num = CONFIG_I2C_MASTER_SDA,
                                             .glitch_ignore_cnt = 7,
                                             .flags.enable_internal_pullup = true };

  return i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle);
}

/**
 * @brief Add an I2C device to the I2C master bus
 *
 * This function registers a new I2C device with the specified address and SCL speed
 * to the I2C master bus, allowing subsequent communication with the device.
 *
 * @param[in] dev_addr The 7-bit I2C device address (without read/write bit)
 * @param[in] scl_speed_hz The SCL clock frequency in Hz for this device
 * @param[out] dev_handle Pointer to store the device handle for future operations
 *
 * @return
 *     - ESP_OK: Device added successfully
 *     - ESP_ERR_INVALID_ARG: Invalid arguments provided
 *     - ESP_ERR_NO_MEM: Insufficient memory to add device
 *     - ESP_FAIL: Failed to add device for other reasons
 */

esp_err_t i2c_add_device(uint8_t dev_addr, uint32_t scl_speed_hz, i2c_master_dev_handle_t* dev_handle) {
  if (dev_handle == NULL) return ESP_ERR_INVALID_ARG;

  i2c_device_config_t dev_config = { .dev_addr_length = I2C_ADDR_BIT_7,
                                     .device_address = dev_addr,
                                     .scl_speed_hz = scl_speed_hz };

  return i2c_master_bus_add_device(i2c_bus_handle, &dev_config, dev_handle);
}

/**
 * @brief Reads data from an I2C device.
 *
 * This function reads a specified number of bytes from an I2C device using the
 * provided device handle.
 *
 * @param[out] data Pointer to the buffer where the read data will be stored.
 * @param[in] length Number of bytes to read from the I2C device.
 * @param[in] dev_handle Pointer to the I2C master device handle.
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer or invalid length)
 *     - ESP_FAIL: I2C communication failure
 *     - Other ESP error codes depending on the I2C driver implementation
 */

esp_err_t i2c_read_data(uint8_t* data, size_t length, i2c_master_dev_handle_t* dev_handle) {
  if (dev_handle == NULL || data == NULL) return ESP_ERR_INVALID_ARG;

  return i2c_master_receive(*dev_handle, data, length, -1);
}

/**
 * @brief Read a single 8-bit value from an I2C device register
 *
 * This function reads one byte from the specified register address of an I2C device.
 *
 * @param[in]  reg_addr    The register address to read from
 * @param[out] value       Pointer to store the read 8-bit value
 * @param[in]  dev_handle  Pointer to the I2C master device handle
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer)
 *     - ESP_FAIL: I2C communication failure
 *     - Other ESP_ERR codes depending on I2C driver implementation
 */

esp_err_t i2c_read_reg8(uint8_t reg_addr, uint8_t* value, i2c_master_dev_handle_t* dev_handle) {
  if (dev_handle == NULL || value == NULL) return ESP_ERR_INVALID_ARG;

  return read_reg(reg_addr, value, sizeof(*value), dev_handle);
}

/**
 * @brief Reads sequential bytes from consecutive 8-bit registers starting at a specified address.
 *
 * This function performs a sequential read operation from multiple consecutive registers
 * starting at the specified register address. It's useful for reading data that spans
 * across multiple registers, such as multi-byte sensor values.
 *
 * @param[in]  start_reg_addr  The starting register address to read from
 * @param[out] data            Pointer to buffer where the read data will be stored
 * @param[in]  length          Number of bytes to read from consecutive registers
 *
 * @return
 *     - ESP_OK                Success
 *     - ESP_ERR_INVALID_ARG   Invalid argument (NULL pointer or invalid length)
 *     - ESP_FAIL              I2C communication failure
 *     - ESP_ERR_TIMEOUT       I2C timeout occurred
 */

esp_err_t i2c_read_sequential_reg8(uint8_t start_reg_addr, uint8_t* data, size_t length,
                                   i2c_master_dev_handle_t* dev_handle) {
  if (dev_handle == NULL || data == NULL) return ESP_ERR_INVALID_ARG;

  return read_reg(start_reg_addr, data, length, dev_handle);
}

/**
 * @brief Reads a 16-bit value from a specified register via I2C
 *
 * This function performs an I2C read operation to retrieve a 16-bit value from
 * a register at the specified address using the provided device handle.
 *
 * @param[in]  reg_addr    The register address to read from
 * @param[out] value       Pointer to store the 16-bit value read from the register
 * @param[in]  dev_handle  Pointer to the I2C master device handle
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer)
 *     - ESP_FAIL: I2C communication failure
 *     - Other ESP_ERR codes from underlying I2C driver
 */

esp_err_t i2c_read_reg16(uint8_t reg_addr, uint16_t* value, i2c_master_dev_handle_t* dev_handle) {
  if (dev_handle == NULL || value == NULL) return ESP_ERR_INVALID_ARG;

  uint8_t data[2];
  ESP_ERROR_CHECK(read_reg(reg_addr, data, sizeof(data), dev_handle));
  *value = (uint16_t)((data[0] << 8) | data[1]);

  return ESP_OK;
}

/**
 * @brief Read a 16-bit register value in Little Endian format from an I2C device
 *
 * This function reads a 16-bit value from a specified register address on an I2C device.
 * The data is expected to be in Little Endian format (LSB first, MSB second).
 *
 * @param[in]  reg_addr    The register address to read from
 * @param[out] value       Pointer to store the 16-bit value read from the register
 * @param[in]  dev_handle  Pointer to the I2C master device handle
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument (null pointer)
 *     - ESP_FAIL: I2C communication failure
 *     - Other ESP error codes from underlying I2C driver functions
 */

esp_err_t i2c_read_reg16_LE(uint8_t reg_addr, uint16_t* value, i2c_master_dev_handle_t* dev_handle) {
  if (dev_handle == NULL || value == NULL) return ESP_ERR_INVALID_ARG;

  uint8_t data[2];
  ESP_ERROR_CHECK(read_reg(reg_addr, data, sizeof(data), dev_handle));
  *value = (uint16_t)((data[1] << 8) | data[0]);

  return ESP_OK;
}

/**
 * @brief Read a 16-bit signed integer from an I2C register in Little Endian format
 *
 * @param[in] reg_addr Register address to read from
 * @param[out] value Pointer to store the read 16-bit signed integer value
 * @param[in] dev_handle Pointer to I2C master device handle
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Parameter error
 *     - ESP_FAIL: I2C read operation failed
 */

esp_err_t i2c_read_regS16_LE(uint8_t reg_addr, int16_t* value, i2c_master_dev_handle_t* dev_handle) {
  if (dev_handle == NULL || value == NULL) return ESP_ERR_INVALID_ARG;

  uint8_t data[2];
  ESP_ERROR_CHECK(read_reg(reg_addr, data, sizeof(data), dev_handle));
  *value = (int16_t)((data[1] << 8) | data[0]);

  return ESP_OK;
}

/**
 * @brief Writes a single command byte to an I2C device.
 *
 * This function transmits a command byte to the specified I2C device handle.
 * It is typically used for sending configuration or control commands to I2C peripherals.
 *
 * @param[in] command The command byte to be written to the I2C device
 * @param[in] dev_handle Pointer to the I2C master device handle
 *
 * @return
 *     - ESP_OK: Command successfully written
 *     - ESP_ERR_INVALID_ARG: Invalid argument (null pointer or invalid handle)
 *     - ESP_FAIL: I2C transmission failed
 *     - Other ESP_ERR codes depending on I2C driver implementation
 */

esp_err_t i2c_write_command(uint8_t command, i2c_master_dev_handle_t* dev_handle) {
  if (dev_handle == NULL) return ESP_ERR_INVALID_ARG;

  return write_reg(&command, 1, dev_handle);
}

/**
 * @brief Write a single byte to an 8-bit register of an I2C device.
 *
 * @param reg_addr The 8-bit register address to write to
 * @param value The byte value to write to the register
 * @param dev_handle Pointer to the I2C master device handle
 *
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid argument
 *         - ESP_FAIL: I2C communication failed
 */

esp_err_t i2c_write_reg8(uint8_t reg_addr, uint8_t value, i2c_master_dev_handle_t* dev_handle) {
  if (dev_handle == NULL) return ESP_ERR_INVALID_ARG;

  uint8_t data[2] = { reg_addr, value };

  return write_reg(data, sizeof(data), dev_handle);
}

/**
 * @brief Write a 16-bit value to a specific register address via I2C
 *
 * @param reg_addr The register address to write to
 * @param value The 16-bit value to write to the register
 * @param dev_handle Pointer to the I2C master device handle
 *
 * @return esp_err_t
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid argument
 *         - ESP_FAIL: I2C communication failure
 */

esp_err_t i2c_write_reg16(uint8_t reg_addr, uint16_t value, i2c_master_dev_handle_t* dev_handle) {
  if (dev_handle == NULL) return ESP_ERR_INVALID_ARG;

  uint8_t data[3] = { reg_addr, (value >> 8) & 0xFF, value & 0xFF };

  return write_reg(data, sizeof(data), dev_handle);
}
