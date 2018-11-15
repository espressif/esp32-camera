#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "rom/lldesc.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_camera.h"
#include "sensor.h"

typedef union {
    struct {
        uint8_t sample2;
        uint8_t unused2;
        uint8_t sample1;
        uint8_t unused1;
    };
    uint32_t val;
} dma_elem_t;

typedef enum {
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 s2, 00 s2 00 s3, 00 s3 00 s4, ...
     */
    SM_0A0B_0B0C = 0,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 s2, 00 s3 00 s4, ...
     */
    SM_0A0B_0C0D = 1,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 00, 00 s2 00 00, 00 s3 00 00, ...
     */
    SM_0A00_0B00 = 3,
} i2s_sampling_mode_t;

