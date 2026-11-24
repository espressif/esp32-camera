// Copyright 2010-2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <stdint.h>
#include "sdkconfig.h"
#include "esp_idf_version.h"
#if CONFIG_IDF_TARGET_ESP32
#if ESP_IDF_VERSION_MAJOR >= 4
#include "esp32/rom/lldesc.h"
#else
#include "rom/lldesc.h"
#endif
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/rom/lldesc.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/rom/lldesc.h"
#elif CONFIG_IDF_TARGET_ESP32S31
/* ESP32-S31 has no rom/lldesc.h and its AXI GDMA uses the 8-byte aligned
 * descriptor type (as in IDF's esp_cam_ctlr_dvp_dma.h). */
#include "hal/dma_types.h"
#endif
#include "esp_log.h"
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#if __has_include("esp_private/periph_ctrl.h")
# include "esp_private/periph_ctrl.h"
#endif
#if __has_include("esp_private/gdma.h")
# include "esp_private/gdma.h"
#endif

#if CONFIG_LCD_CAM_ISR_IRAM_SAFE
#define CAMERA_ISR_IRAM_FLAG ESP_INTR_FLAG_IRAM
#define CAMERA_ISR_IRAM_ATTR IRAM_ATTR
#else
#define CAMERA_ISR_IRAM_FLAG 0
#define CAMERA_ISR_IRAM_ATTR 
#endif

#define CAMERA_DBG_PIN_ENABLE 0
#if CAMERA_DBG_PIN_ENABLE
    #if CONFIG_IDF_TARGET_ESP32
        #define DBG_PIN_NUM 26
    #else
        #define DBG_PIN_NUM 7
    #endif
    #include "hal/gpio_ll.h"
    #define DBG_PIN_SET(v) gpio_ll_set_level(&GPIO, DBG_PIN_NUM, v)
#else
    #define DBG_PIN_SET(v)
#endif

#define CAM_CHECK(a, str, ret) if (!(a)) {                                          \
        ESP_LOGE(TAG,"%s(%d): %s", __FUNCTION__, __LINE__, str);                    \
        return (ret);                                                               \
        }

#define CAM_CHECK_GOTO(a, str, lab) if (!(a)) {                                     \
        ESP_LOGE(TAG,"%s(%d): %s", __FUNCTION__, __LINE__, str);                    \
        goto lab;                                                                   \
        }

#if CONFIG_IDF_TARGET_ESP32S31
/* AXI GDMA descriptor buffer: max 4095 bytes, kept 64-byte aligned so that
 * consecutive node buffers stay aligned for PSRAM/DMA access. */
#define LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE  (4032)
#else
#define LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE  (4092)
#endif

/* DMA descriptor type: AHB/I2S targets use the ROM lldesc_t, the AXI GDMA
 * on ESP32-S31 uses dma_descriptor_align8_t. Access it only through the
 * ll_cam_dma_desc_* helpers below so cam_hal.c stays target agnostic. */
#if CONFIG_IDF_TARGET_ESP32S31
typedef dma_descriptor_align8_t cam_dma_desc_t;
#else
typedef lldesc_t cam_dma_desc_t;
#endif

static inline void ll_cam_dma_desc_init(cam_dma_desc_t *desc, uint16_t size, uint8_t *buf, cam_dma_desc_t *next)
{
#if CONFIG_IDF_TARGET_ESP32S31
    desc->dw0.size = size;
    desc->dw0.length = 0;
    desc->dw0.err_eof = 0;
    desc->dw0.suc_eof = 0;
    desc->dw0.owner = 1;
    desc->buffer = buf;
    desc->next = next;
#else
    desc->size = size;
    desc->length = 0;
    desc->sosf = 0;
    desc->eof = 0;
    desc->owner = 1;
    desc->buf = buf;
    desc->empty = (uint32_t)next;
#endif
}

static inline uint32_t ll_cam_dma_desc_get_length(const cam_dma_desc_t *desc)
{
#if CONFIG_IDF_TARGET_ESP32S31
    return desc->dw0.length;
#else
    return desc->length;
#endif
}

static inline uint32_t ll_cam_dma_desc_get_size(const cam_dma_desc_t *desc)
{
#if CONFIG_IDF_TARGET_ESP32S31
    return desc->dw0.size;
#else
    return desc->size;
#endif
}

typedef enum {
    CAM_IN_SUC_EOF_EVENT = 0,
    CAM_VSYNC_EVENT
} cam_event_t;

typedef enum {
    CAM_STATE_IDLE = 0,
    CAM_STATE_READ_BUF = 1,
} cam_state_t;

typedef struct {
    camera_fb_t fb;
    uint8_t en;
    //for RGB/YUV modes
    cam_dma_desc_t *dma;
    size_t fb_offset;
} cam_frame_t;

typedef struct {
    uint32_t dma_bytes_per_item;
    uint32_t dma_buffer_size;
    uint32_t dma_half_buffer_size;
    uint32_t dma_half_buffer_cnt;
    uint32_t dma_node_buffer_size;
    uint32_t dma_node_cnt;
    uint32_t frame_copy_cnt;

    //for JPEG mode
    cam_dma_desc_t *dma;
    uint8_t  *dma_buffer;

    cam_frame_t *frames;

    QueueHandle_t event_queue;
    QueueHandle_t frame_buffer_queue;
    TaskHandle_t task_handle;
    intr_handle_t cam_intr_handle;

    uint8_t dma_num;//ESP32-S3
    intr_handle_t dma_intr_handle;//ESP32-S3
#if SOC_GDMA_SUPPORTED
    gdma_channel_handle_t dma_channel_handle;//ESP32-S3
#endif

    uint8_t jpeg_mode;
    uint8_t vsync_pin;
    uint8_t vsync_invert;
    uint32_t frame_cnt;
    uint32_t recv_size;
    bool swap_data;
    bool psram_mode;

    //for RGB/YUV modes
    uint16_t width;
    uint16_t height;
#if CONFIG_CAMERA_CONVERTER_ENABLED
    float in_bytes_per_pixel;
    float fb_bytes_per_pixel;
    camera_conv_mode_t conv_mode;
#else
    uint8_t in_bytes_per_pixel;
    uint8_t fb_bytes_per_pixel;
#endif
    uint32_t fb_size;

    cam_state_t state;
} cam_obj_t;


bool ll_cam_stop(cam_obj_t *cam);
bool ll_cam_start(cam_obj_t *cam, int frame_pos);
esp_err_t ll_cam_config(cam_obj_t *cam, const camera_config_t *config);
esp_err_t ll_cam_deinit(cam_obj_t *cam);
void ll_cam_vsync_intr_enable(cam_obj_t *cam, bool en);
esp_err_t ll_cam_set_pin(cam_obj_t *cam, const camera_config_t *config);
esp_err_t ll_cam_init_isr(cam_obj_t *cam);
void ll_cam_do_vsync(cam_obj_t *cam);
uint8_t ll_cam_get_dma_align(cam_obj_t *cam);
bool ll_cam_dma_sizes(cam_obj_t *cam);
size_t ll_cam_memcpy(cam_obj_t *cam, uint8_t *out, const uint8_t *in, size_t len);
esp_err_t ll_cam_set_sample_mode(cam_obj_t *cam, pixformat_t pix_format, uint32_t xclk_freq_hz, uint16_t sensor_pid);
#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32S31
void ll_cam_dma_print_state(cam_obj_t *cam);
void ll_cam_dma_reset(cam_obj_t *cam);
#endif

// implemented in cam_hal
void ll_cam_send_event(cam_obj_t *cam, cam_event_t cam_event, BaseType_t * HPTaskAwoken);
