/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * SCCB (I2C like) driver.
 *
 */
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sccb.h"
#include "twi.h"
#include <stdio.h>
#include "sdkconfig.h"
#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
static const char* TAG = "sccb";
#endif


#define SCCB_FREQ   (100000) // We don't need fast I2C. 100KHz is fine here.
#define TIMEOUT     (1000) /* Can't be sure when I2C routines return. Interrupts
while polling hardware may result in unknown delays. */


int SCCB_Init(int pin_sda, int pin_scl)
{
    twi_init(pin_sda, pin_scl);
    return 0;
}

uint8_t SCCB_Probe()
{
    uint8_t reg = 0x00;
    uint8_t slv_addr = 0x00;

    for (uint8_t i=0; i<127; i++) {
        if (twi_writeTo(i, &reg, 1, true) == 0) {
            slv_addr = i;
            break;
        }

        if (i!=126) {
            vTaskDelay(1 / portTICK_PERIOD_MS); // Necessary for OV7725 camera (not for OV2640).
        }
    }
    return slv_addr;
}

uint8_t SCCB_Read(uint8_t slv_addr, uint8_t reg)
{
    uint8_t data=0;

    int rc = twi_writeTo(slv_addr, &reg, 1, true);
    if (rc != 0) {
        data = 0xff;
    } else {
        rc = twi_readFrom(slv_addr, &data, 1, true);
        if (rc != 0) {
            data=0xFF;
        }
    }
    if (rc != 0) {
        ESP_LOGE(TAG, "SCCB_Read [%02x] failed rc=%d\n", reg, rc);
    }
    return data;
}

uint8_t SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data)
{
    uint8_t ret=0;
    uint8_t buf[] = {reg, data};

    if(twi_writeTo(slv_addr, buf, 2, true) != 0) {
        ret=0xFF;
    }
    if (ret != 0) {
        printf("SCCB_Write [%02x]=%02x failed\n", reg, data);
    }
    return ret;
}
