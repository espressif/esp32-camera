/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * SCCB (I2C like) driver with the new esp-idf I2C API.
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
static const char *TAG = "sccb-ng";
#endif

#define LITTLETOBIG(x) ((x << 8) | (x >> 8))

#include "esp_private/i2c_platform.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"

// support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

#define TIMEOUT_MS 1000                /*!< I2C timeout duration */
#define SCCB_FREQ CONFIG_SCCB_CLK_FREQ /*!< I2C master frequency */
#if CONFIG_SCCB_HARDWARE_I2C_PORT1
const int SCCB_I2C_PORT_DEFAULT = 1;
#else
const int SCCB_I2C_PORT_DEFAULT = 0;
#endif

#define MAX_DEVICES UINT8_MAX-1

/*
 The legacy I2C driver used addresses to differentiate between devices, whereas the new driver uses
 i2c_master_dev_handle_t structs which are registed to the bus.
 To avoid re-writing all camera dependant code, we simply translate the devices address to the corresponding
 device_handle. This keeps all interfaces to the drivers identical.
 To perform this conversion the following local struct is used.
*/
typedef struct
{
    i2c_master_dev_handle_t dev_handle;
    uint16_t address;
} device_t;

static device_t devices[MAX_DEVICES];
static uint8_t device_count = 0;
static int sccb_i2c_port;
static bool sccb_owns_i2c_port;

i2c_master_dev_handle_t *get_handle_from_address(uint8_t slv_addr)
{
    for (uint8_t i = 0; i < device_count; i++)
    {

        if (slv_addr == devices[i].address)
        {
            return &(devices[i].dev_handle);
        }
    }

    ESP_LOGE(TAG, "Device with address %02x not found", slv_addr);
    return NULL;
}

int SCCB_Install_Device(uint8_t slv_addr)
{
    esp_err_t ret;
    i2c_master_bus_handle_t bus_handle;

    if (device_count > MAX_DEVICES)
    {
        ESP_LOGE(TAG, "cannot add more than %d devices", MAX_DEVICES);
        return ESP_FAIL;
    }

    ret = i2c_master_get_bus_handle(sccb_i2c_port, &bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "failed to get SCCB I2C Bus handle for port %d", sccb_i2c_port);
        return ret;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = slv_addr, // not yet set
        .scl_speed_hz = SCCB_FREQ,
    };

    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &(devices[device_count].dev_handle));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "failed to install SCCB I2C device: %s", esp_err_to_name(ret));
        return -1;
    }

    devices[device_count].address = slv_addr;
    device_count++;
    return 0;
}

int SCCB_Init(int pin_sda, int pin_scl)
{
    ESP_LOGI(TAG, "pin_sda %d pin_scl %d", pin_sda, pin_scl);
    // i2c_config_t conf;
    esp_err_t ret;

    sccb_i2c_port = SCCB_I2C_PORT_DEFAULT;
    sccb_owns_i2c_port = true;
    ESP_LOGI(TAG, "sccb_i2c_port=%d", sccb_i2c_port);

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = SCCB_I2C_PORT_DEFAULT,
        .scl_io_num = pin_scl,
        .sda_io_num = pin_sda,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = 1};

    i2c_master_bus_handle_t bus_handle;
    ret = i2c_new_master_bus(&i2c_mst_config, &bus_handle);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "failed to install SCCB I2C master bus on port %d: %s", sccb_i2c_port, esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

int SCCB_Use_Port(int i2c_num)
{ // sccb use an already initialized I2C port
    if (sccb_owns_i2c_port)
    {
        SCCB_Deinit();
    }
    if (i2c_num < 0 || i2c_num > I2C_NUM_MAX)
    {
        return ESP_ERR_INVALID_ARG;
    }
    sccb_i2c_port = i2c_num;

    return ESP_OK;
}

int SCCB_Deinit(void)
{
    esp_err_t ret;

    for (uint8_t i = 0; i < device_count; i++)
    {
        ret = i2c_master_bus_rm_device(devices[i].dev_handle);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "failed to remove SCCB I2C Device");
            return ret;
        }

        devices[i].dev_handle = NULL;
        devices[i].address = 0;
    }
    device_count = 0;

    if (!sccb_owns_i2c_port)
    {
        return ESP_OK;
    }
    sccb_owns_i2c_port = false;

    i2c_master_bus_handle_t bus_handle;
    ret = i2c_master_get_bus_handle(sccb_i2c_port, &bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "failed to get SCCB I2C Bus handle for port %d", sccb_i2c_port);
        return ret;
    }

    ret = i2c_del_master_bus(bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "failed to get delete SCCB I2C Master Bus at port %d", sccb_i2c_port);
        return ret;
    }

    return ESP_OK;
}

uint8_t SCCB_Probe(void)
{
    uint8_t slave_addr = 0x0;
    esp_err_t ret;
    i2c_master_bus_handle_t bus_handle;

    ret = i2c_master_get_bus_handle(sccb_i2c_port, &bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "failed to get SCCB I2C Bus handle for port %d", sccb_i2c_port);
        return ret;
    }

    for (size_t i = 0; i < CAMERA_MODEL_MAX; i++)
    {
        if (slave_addr == camera_sensor[i].sccb_addr)
        {
            continue;
        }
        slave_addr = camera_sensor[i].sccb_addr;

        ret = i2c_master_probe(bus_handle, slave_addr, TIMEOUT_MS);

        if (ret == ESP_OK)
        {
            if (SCCB_Install_Device(slave_addr) != 0)
            {
                return 0;
            }
            return slave_addr;
        }
    }
    return 0;
}

uint8_t SCCB_Read(uint8_t slv_addr, uint8_t reg)
{
    i2c_master_dev_handle_t dev_handle = *(get_handle_from_address(slv_addr));

    uint8_t tx_buffer[1];
    uint8_t rx_buffer[1];

    tx_buffer[0] = reg;

    esp_err_t ret = i2c_master_transmit_receive(dev_handle, tx_buffer, 1, rx_buffer, 1, TIMEOUT_MS);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SCCB_Read Failed addr:0x%02x, reg:0x%02x, data:0x%02x, ret:%d", slv_addr, reg, rx_buffer[0], ret);
    }

    return rx_buffer[0];
}

int SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data)
{
    i2c_master_dev_handle_t dev_handle = *(get_handle_from_address(slv_addr));

    uint8_t tx_buffer[2];
    tx_buffer[0] = reg;
    tx_buffer[1] = data;

    esp_err_t ret = i2c_master_transmit(dev_handle, tx_buffer, 2, TIMEOUT_MS);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SCCB_Write Failed addr:0x%02x, reg:0x%02x, data:0x%02x, ret:%d", slv_addr, reg, data, ret);
    }

    return ret == ESP_OK ? 0 : -1;
}

uint8_t SCCB_Read16(uint8_t slv_addr, uint16_t reg)
{
    i2c_master_dev_handle_t dev_handle = *(get_handle_from_address(slv_addr));

    uint8_t rx_buffer[1];

    uint16_t reg_htons = LITTLETOBIG(reg);
    uint8_t *reg_u8 = (uint8_t *)&reg_htons;

    esp_err_t ret = i2c_master_transmit_receive(dev_handle, reg_u8, 2, rx_buffer, 1, TIMEOUT_MS);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "W [%04x]=%02x fail\n", reg, rx_buffer[0]);
    }

    return rx_buffer[0];
}

int SCCB_Write16(uint8_t slv_addr, uint16_t reg, uint8_t data)
{
    i2c_master_dev_handle_t dev_handle = *(get_handle_from_address(slv_addr));

    uint8_t tx_buffer[3];
    tx_buffer[0] = reg >> 8;
    tx_buffer[1] = reg & 0x00ff;
    tx_buffer[2] = data;

    esp_err_t ret = i2c_master_transmit(dev_handle, tx_buffer, 3, TIMEOUT_MS);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "W [%04x]=%02x fail\n", reg, data);
    }
    return ret == ESP_OK ? 0 : -1;
}

uint16_t SCCB_Read_Addr16_Val16(uint8_t slv_addr, uint16_t reg)
{
    i2c_master_dev_handle_t dev_handle = *(get_handle_from_address(slv_addr));

    uint8_t rx_buffer[2];

    uint16_t reg_htons = LITTLETOBIG(reg);
    uint8_t *reg_u8 = (uint8_t *)&reg_htons;

    esp_err_t ret = i2c_master_transmit_receive(dev_handle, reg_u8, 2, rx_buffer, 2, TIMEOUT_MS);
    uint16_t data = ((uint16_t)rx_buffer[0] << 8) | (uint16_t)rx_buffer[1];

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "W [%04x]=%02x fail\n", reg, data);
    }

    return data;
}

int SCCB_Write_Addr16_Val16(uint8_t slv_addr, uint16_t reg, uint16_t data)
{
    i2c_master_dev_handle_t dev_handle = *(get_handle_from_address(slv_addr));

    uint8_t tx_buffer[4];
    tx_buffer[0] = reg >> 8;
    tx_buffer[1] = reg & 0x00ff;
    tx_buffer[2] = data >> 8;
    tx_buffer[3] = data & 0x00ff;

    esp_err_t ret = i2c_master_transmit(dev_handle, tx_buffer, 4, TIMEOUT_MS);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "W [%04x]=%02x fail\n", reg, data);
    }
    return ret == ESP_OK ? 0 : -1;
}
