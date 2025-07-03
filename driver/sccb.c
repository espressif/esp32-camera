/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com> updated to support new I2C library from Espressif!
 * by Arturo Vicente Jaén 2025 Universidad de Murcia (Laboratorio de Optica)
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * SCCB (I2C like) driver.
 *
 */
#include <stdbool.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sccb.h"
#include "sensor.h"
#include <stdio.h>
#include "sdkconfig.h"
#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
static const char* TAG = "sccb";
#endif

#include "driver/i2c_master.h"

#define LITTLETOBIG(x)          ((x<<8)|(x>>8))

// support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

// Default I2C port from Kconfig
#if CONFIG_SCCB_HARDWARE_I2C_PORT1
const int SCCB_I2C_PORT_DEFAULT = 1;
#else
const int SCCB_I2C_PORT_DEFAULT = 0;
#endif

// Default timeout for I2C operations in milliseconds
#define I2C_TIMEOUT_MS 1000

// Handles for the bus and the device. Static to be managed by this driver.
static i2c_master_bus_handle_t s_i2c_bus_handle = NULL;
static i2c_master_dev_handle_t s_i2c_device_handle = NULL;
static bool s_sccb_owns_i2c_bus = false;

int SCCB_Init(int pin_sda, int pin_scl)
{
    ESP_LOGI(TAG, "Initializing SCCB on sda=%d, scl=%d", pin_sda, pin_scl);

    // If we already own a bus, delete it first
    if (s_sccb_owns_i2c_bus && s_i2c_bus_handle != NULL) {
        i2c_del_master_bus(s_i2c_bus_handle);
        s_i2c_bus_handle = NULL;
    }
    
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = SCCB_I2C_PORT_DEFAULT,
        .scl_io_num = pin_scl,
        .sda_io_num = pin_sda,
        .glitch_ignore_cnt = 7, // Recommended value from Espressif documentation
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, &s_i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return ret;
    }
    s_sccb_owns_i2c_bus = true;
    return ESP_OK;
}

int SCCB_Deinit(void)
{
    if (!s_sccb_owns_i2c_bus || s_i2c_bus_handle == NULL) {
        return ESP_OK; // Nothing to do if we don't own the bus
    }

    esp_err_t ret = i2c_del_master_bus(s_i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete I2C master bus: %s", esp_err_to_name(ret));
    }
    s_i2c_bus_handle = NULL;
    s_sccb_owns_i2c_bus = false;
    return ret;
}

// This function is no longer necessary with the new API, but kept for compatibility.
// The new API manages the port through the bus handle.
int SCCB_Use_Port(int i2c_num) {
    if (i2c_num < 0 || i2c_num > I2C_NUM_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGW(TAG, "SCCB_Use_Port is deprecated. The port is defined in SCCB_Init.");
    return ESP_OK;
}

uint8_t SCCB_Probe(void)
{
    uint8_t slave_addr = 0x0;

    if (s_i2c_bus_handle == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized. Call SCCB_Init first.");
        return 0;
    }

    for (size_t i = 0; i < CAMERA_MODEL_MAX; i++) {
        // Avoid probing the same address twice if it appears consecutively in the list
        if (slave_addr == camera_sensor[i].sccb_addr) {
            continue;
        }
        slave_addr = camera_sensor[i].sccb_addr;

        // The i2c_master_probe function is the modern way to do this
        esp_err_t ret = i2c_master_probe(s_i2c_bus_handle, slave_addr, 50);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Sensor found at address 0x%02X", slave_addr);
            return slave_addr;
        }
    }
    ESP_LOGW(TAG, "No camera sensor found.");
    return 0;
}

// Internal helper function to create/update the I2C device handle
static esp_err_t sccb_get_device_handle(uint8_t slv_addr, uint32_t clk_speed)
{
    if (s_i2c_bus_handle == NULL) return ESP_ERR_INVALID_STATE;

    // If we already have a handle, delete it to create a new one (needed if address or speed changes)
    if (s_i2c_device_handle) {
        i2c_master_bus_rm_device(s_i2c_device_handle);
        s_i2c_device_handle = NULL;
    }
    
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = slv_addr,
        .scl_speed_hz = clk_speed,
    };

    return i2c_master_bus_add_device(s_i2c_bus_handle, &dev_cfg, &s_i2c_device_handle);
}
uint8_t SCCB_Read(uint8_t slv_addr, uint8_t reg)
{
    uint8_t data = 0;
    
    // Get the device handle (it's created if it doesn't exist)
    if (sccb_get_device_handle(slv_addr, CONFIG_SCCB_CLK_FREQ) != ESP_OK) {
        return -1;
    }

    // i2c_master_transmit_receive greatly simplifies the register read operation
    esp_err_t ret = i2c_master_transmit_receive(s_i2c_device_handle, &reg, 1, &data, 1, I2C_TIMEOUT_MS);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SCCB_Read Failed. addr:0x%02x, reg:0x%02x, error: %s", slv_addr, reg, esp_err_to_name(ret));
        return -1; // -1 is a better error indicator than the data value
    }
    return data;
}

int SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data)
{
    uint8_t buffer[2] = {reg, data};

    if (sccb_get_device_handle(slv_addr, CONFIG_SCCB_CLK_FREQ) != ESP_OK) {
        return -1;
    }

    // i2c_master_transmit to send the register and data together
    esp_err_t ret = i2c_master_transmit(s_i2c_device_handle, buffer, sizeof(buffer), I2C_TIMEOUT_MS);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SCCB_Write Failed. addr:0x%02x, reg:0x%02x, data:0x%02x, error: %s", slv_addr, reg, data, esp_err_to_name(ret));
    }
    return (ret == ESP_OK) ? 0 : -1;
}

uint8_t SCCB_Read16(uint8_t slv_addr, uint16_t reg)
{
    uint8_t data = 0;
    uint8_t reg_buffer[2] = {(reg >> 8) & 0xFF, reg & 0xFF};

    if (sccb_get_device_handle(slv_addr, CONFIG_SCCB_CLK_FREQ) != ESP_OK) {
        return -1;
    }

    esp_err_t ret = i2c_master_transmit_receive(s_i2c_device_handle, reg_buffer, 2, &data, 1, I2C_TIMEOUT_MS);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SCCB_Read16 Failed. addr:0x%02x, reg:0x%04x, error: %s", slv_addr, reg, esp_err_to_name(ret));
        return -1;
    }
    return data;
}

int SCCB_Write16(uint8_t slv_addr, uint16_t reg, uint8_t data)
{
    uint8_t buffer[3] = {(reg >> 8) & 0xFF, reg & 0xFF, data};

    if (sccb_get_device_handle(slv_addr, CONFIG_SCCB_CLK_FREQ) != ESP_OK) {
        return -1;
    }
    
    esp_err_t ret = i2c_master_transmit(s_i2c_device_handle, buffer, sizeof(buffer), I2C_TIMEOUT_MS);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SCCB_Write16 Failed. addr:0x%02x, reg:0x%04x, data:0x%02x, error: %s", slv_addr, reg, data, esp_err_to_name(ret));
    }
    return (ret == ESP_OK) ? 0 : -1;
}

// --- Functions for 16-bit registers and 16-bit data ---
uint16_t SCCB_Read_Addr16_Val16(uint8_t slv_addr, uint16_t reg)
{
    uint8_t reg_buffer[2] = {(reg >> 8) & 0xFF, reg & 0xFF};
    uint8_t data_buffer[2] = {0, 0};

    if (sccb_get_device_handle(slv_addr, CONFIG_SCCB_CLK_FREQ) != ESP_OK) {
        return 0xFFFF; // Return -1 as a 16-bit value
    }

    esp_err_t ret = i2c_master_transmit_receive(s_i2c_device_handle, reg_buffer, 2, data_buffer, 2, I2C_TIMEOUT_MS);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SCCB_Read_Addr16_Val16 Failed. addr:0x%02x, reg:0x%04x, error: %s", slv_addr, reg, esp_err_to_name(ret));
        return 0xFFFF;
    }
    // Combine the read bytes into a 16-bit value (Big Endian order)
    return (uint16_t)(data_buffer[0] << 8) | data_buffer[1];
}

int SCCB_Write_Addr16_Val16(uint8_t slv_addr, uint16_t reg, uint16_t data)
{
    uint8_t buffer[4] = {
        (reg >> 8) & 0xFF,  // Register (MSB)
        reg & 0xFF,         // Register (LSB)
        (data >> 8) & 0xFF, // Data (MSB)
        data & 0xFF         // Data (LSB)
    };

    if (sccb_get_device_handle(slv_addr, CONFIG_SCCB_CLK_FREQ) != ESP_OK) {
        return -1;
    }

    esp_err_t ret = i2c_master_transmit(s_i2c_device_handle, buffer, sizeof(buffer), I2C_TIMEOUT_MS);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SCCB_Write_Addr16_Val16 Failed. addr:0x%02x, reg:0x%04x, data:0x%04x, error: %s", slv_addr, reg, data, esp_err_to_name(ret));
    }
    return (ret == ESP_OK) ? 0 : -1;
}