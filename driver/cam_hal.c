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

#include <stdio.h>
#include <string.h>
#include <stdalign.h>
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ll_cam.h"
#include "cam_hal.h"

#if (ESP_IDF_VERSION_MAJOR == 3) && (ESP_IDF_VERSION_MINOR == 3)
#include "rom/ets_sys.h"
#else
#include "esp_timer.h"
#include "esp_cache.h"
#include "hal/cache_hal.h"
#include "hal/cache_ll.h"
#include "esp_idf_version.h"
#ifndef ESP_CACHE_MSYNC_FLAG_DIR_M2C
#define ESP_CACHE_MSYNC_FLAG_DIR_M2C 0
#endif
#if CONFIG_IDF_TARGET_ESP32
#include "esp32/rom/ets_sys.h"  // will be removed in idf v5.0
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/rom/ets_sys.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/rom/ets_sys.h"
#endif
#endif // ESP_IDF_VERSION_MAJOR
#define ESP_CAMERA_ETS_PRINTF ets_printf

#if CONFIG_CAMERA_TASK_STACK_SIZE
#define CAM_TASK_STACK             CONFIG_CAMERA_TASK_STACK_SIZE
#else
#define CAM_TASK_STACK             (4*1024)
#endif

static const char *TAG = "cam_hal";
static cam_obj_t *cam_obj = NULL;
#if defined(CONFIG_CAMERA_PSRAM_DMA)
#define CAMERA_PSRAM_DMA_ENABLED CONFIG_CAMERA_PSRAM_DMA
#else
#define CAMERA_PSRAM_DMA_ENABLED 0
#endif

static volatile bool g_psram_dma_mode = CAMERA_PSRAM_DMA_ENABLED;
static portMUX_TYPE g_psram_dma_lock = portMUX_INITIALIZER_UNLOCKED;

/* At top of cam_hal.c – one switch for noisy ISR prints */
#ifndef CAM_LOG_SPAM_EVERY_FRAME
#define CAM_LOG_SPAM_EVERY_FRAME 0   /* set to 1 to restore old behaviour */
#endif

/* Number of bytes copied to SRAM for SOI validation when capturing
 * directly to PSRAM. Tunable to probe more of the frame start if needed. */
#ifndef CAM_SOI_PROBE_BYTES
#define CAM_SOI_PROBE_BYTES 32
#endif
/*
 * PSRAM DMA may bypass the CPU cache. Always call esp_cache_msync() on
 * PSRAM regions that the CPU will read so cached reads see the data written
 * by DMA.
 */

static inline size_t dcache_line_size(void)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
    /* cache_hal_get_cache_line_size() added extra argument from IDF 5.2 */
    return cache_hal_get_cache_line_size(CACHE_LL_LEVEL_EXT_MEM, CACHE_TYPE_DATA);
#else
    /* Older releases only expose the ROM helper, all current targets
     * have a 32‑byte DCache line */
    return 32;
#endif
}

/*
 * Invalidate CPU data cache lines that cover a region in PSRAM which
 * has just been written by DMA. This guarantees subsequent CPU reads
 * fetch the fresh data from PSRAM rather than stale cache contents.
 * Both address and length are aligned to the data cache line size.
 */
static inline void cam_drop_psram_cache(void *addr, size_t len)
{
    size_t line = dcache_line_size();
    if (line == 0) {
        line = 32; /* sane fallback */
    }
    uintptr_t start = (uintptr_t)addr & ~(line - 1);
    size_t sync_len = (len + ((uintptr_t)addr - start) + line - 1) & ~(line - 1);
    esp_cache_msync((void *)start, sync_len,
                    ESP_CACHE_MSYNC_FLAG_DIR_M2C | ESP_CACHE_MSYNC_FLAG_INVALIDATE);
}

/* Throttle repeated warnings printed from tight loops / ISRs.
 *
 * counter – static DRAM/IRAM uint16_t you pass in
 * first   – literal C string shown on first hit and as prefix of summaries
 */
#if CONFIG_LOG_DEFAULT_LEVEL >= 2
#define CAM_WARN_THROTTLE(counter, first)                                  \
    do {                                                                  \
        if (++(counter) == 1) {                                           \
            ESP_CAMERA_ETS_PRINTF(DRAM_STR("cam_hal: %s\r\n"), first);        \
        } else if ((counter) % 100 == 0) {                                \
            ESP_CAMERA_ETS_PRINTF(DRAM_STR("cam_hal: %s - 100 additional misses\r\n"), first); \
        }                                                                 \
        if ((counter) == 10000) (counter) = 1;                            \
    } while (0)
#else
#define CAM_WARN_THROTTLE(counter, first) do { (void)(counter); } while (0)
#endif

/* JPEG markers (byte-order independent). */
static const uint8_t JPEG_SOI_MARKER[] = {0xFF, 0xD8, 0xFF}; /* SOI = FF D8 FF */
#define JPEG_SOI_MARKER_LEN (3)
static const uint8_t JPEG_EOI_BYTES[] = {0xFF, 0xD9};        /* EOI = FF D9 */
#define JPEG_EOI_MARKER_LEN (2)

/* Compute the scan window for JPEG EOI detection in PSRAM. */
static inline size_t eoi_probe_window(size_t half, size_t frame_len)
{
    size_t w = half + (JPEG_EOI_MARKER_LEN - 1);
    return w > frame_len ? frame_len : w;
}

static int cam_verify_jpeg_soi(const uint8_t *inbuf, uint32_t length)
{
    static uint16_t warn_soi_miss_cnt = 0;
    if (length < JPEG_SOI_MARKER_LEN) {
        CAM_WARN_THROTTLE(warn_soi_miss_cnt,
                          "NO-SOI - JPEG start marker missing (len < 3b)");
        return -1;
    }

    for (uint32_t i = 0; i <= length - JPEG_SOI_MARKER_LEN; i++) {
        if (memcmp(&inbuf[i], JPEG_SOI_MARKER, JPEG_SOI_MARKER_LEN) == 0) {
            //ESP_LOGW(TAG, "SOI: %d", (int) i);
            return i;
        }
    }

    CAM_WARN_THROTTLE(warn_soi_miss_cnt,
                      "NO-SOI - JPEG start marker missing");
    return -1;
}

static int cam_verify_jpeg_eoi(const uint8_t *inbuf, uint32_t length, bool search_forward)
{
    if (length < JPEG_EOI_MARKER_LEN) {
        return -1;
    }

    if (search_forward) {
        /* Scan forward to honor the earliest marker in the buffer. This avoids
         * returning an EOI that belongs to a larger previous frame when the tail
         * of that frame still resides in PSRAM. JPEG data is pseudo random, so
         * the first marker byte appears rarely; test four positions per load to
         * reduce memory traffic. */
        const uint8_t *pat = JPEG_EOI_BYTES;
        const uint32_t A = pat[0] * 0x01010101u;
        const uint32_t ONE = 0x01010101u;
        const uint32_t HIGH = 0x80808080u;
        uint32_t i = 0;
        while (i + 4 <= length) {
            uint32_t w;
            memcpy(&w, inbuf + i, 4); /* unaligned load is allowed */
            uint32_t x = w ^ A; /* identify bytes equal to first marker byte */
            uint32_t m = (~x & (x - ONE)) & HIGH; /* mask has high bit set for candidate bytes */
            while (m) { /* handle only candidates to avoid unnecessary memcmp calls */
                unsigned off = __builtin_ctz(m) >> 3;
                uint32_t pos = i + off;
                if (pos + JPEG_EOI_MARKER_LEN <= length &&
                    memcmp(inbuf + pos, pat, JPEG_EOI_MARKER_LEN) == 0) {
                    return pos;
                }
                m &= m - 1; /* clear processed candidate */
            }
            i += 4;
        }
        for (; i + JPEG_EOI_MARKER_LEN <= length; i++) {
            if (memcmp(inbuf + i, pat, JPEG_EOI_MARKER_LEN) == 0) {
                return i;
            }
        }
        return -1;
    }

    const uint8_t *dptr = inbuf + length - JPEG_EOI_MARKER_LEN;
    while (dptr >= inbuf) {
        if (memcmp(dptr, JPEG_EOI_BYTES, JPEG_EOI_MARKER_LEN) == 0) {
            return dptr - inbuf;
        }
        if (dptr == inbuf) {
            break;
        }
        dptr--;
    }
    return -1;
}

static bool cam_get_next_frame(int * frame_pos)
{
    if(!cam_obj->frames[*frame_pos].en){
        for (int x = 0; x < cam_obj->frame_cnt; x++) {
            if (cam_obj->frames[x].en) {
                *frame_pos = x;
                return true;
            }
        }
    } else {
        return true;
    }
    return false;
}

static bool cam_start_frame(int * frame_pos)
{
    if (cam_get_next_frame(frame_pos)) {
        if(ll_cam_start(cam_obj, *frame_pos)){
            // Vsync the frame manually
            ll_cam_do_vsync(cam_obj);
            uint64_t us = (uint64_t)esp_timer_get_time();
            cam_obj->frames[*frame_pos].fb.timestamp.tv_sec = us / 1000000UL;
            cam_obj->frames[*frame_pos].fb.timestamp.tv_usec = us % 1000000UL;
            return true;
        }
    }
    return false;
}

void IRAM_ATTR ll_cam_send_event(cam_obj_t *cam, cam_event_t cam_event, BaseType_t * HPTaskAwoken)
{
    if (xQueueSendFromISR(cam->event_queue, (void *)&cam_event, HPTaskAwoken) != pdTRUE) {
        ll_cam_stop(cam);
        cam->state = CAM_STATE_IDLE;
#if CAM_LOG_SPAM_EVERY_FRAME
        ESP_DRAM_LOGD(TAG, "EV-%s-OVF", cam_event==CAM_IN_SUC_EOF_EVENT ? "EOF" : "VSYNC");
#else
        static uint16_t ovf_cnt = 0;
        CAM_WARN_THROTTLE(ovf_cnt,
                          cam_event==CAM_IN_SUC_EOF_EVENT ? "EV-EOF-OVF" : "EV-VSYNC-OVF");
#endif
    }
}

//Copy fram from DMA dma_buffer to fram dma_buffer
static void cam_task(void *arg)
{
    int cnt = 0;
    int frame_pos = 0;
    cam_obj->state = CAM_STATE_IDLE;
    cam_event_t cam_event = 0;

    xQueueReset(cam_obj->event_queue);

    while (1) {
        xQueueReceive(cam_obj->event_queue, (void *)&cam_event, portMAX_DELAY);
        DBG_PIN_SET(1);
        switch (cam_obj->state) {

            case CAM_STATE_IDLE: {
                if (cam_event == CAM_VSYNC_EVENT) {
                    //DBG_PIN_SET(1);
                    if(cam_start_frame(&frame_pos)){
                        cam_obj->frames[frame_pos].fb.len = 0;
                        cam_obj->state = CAM_STATE_READ_BUF;
                    }
                    cnt = 0;
                }
            }
            break;

            case CAM_STATE_READ_BUF: {
                camera_fb_t * frame_buffer_event = &cam_obj->frames[frame_pos].fb;
                size_t pixels_per_dma = (cam_obj->dma_half_buffer_size * cam_obj->fb_bytes_per_pixel) / (cam_obj->dma_bytes_per_item * cam_obj->in_bytes_per_pixel);

                if (cam_event == CAM_IN_SUC_EOF_EVENT) {
                    if(!cam_obj->psram_mode){
                        if (cam_obj->fb_size < (frame_buffer_event->len + pixels_per_dma)) {
                            ESP_CAMERA_ETS_PRINTF(DRAM_STR("cam_hal: FB-OVF\r\n"));
                            ll_cam_stop(cam_obj);
                            continue;
                        }
                        frame_buffer_event->len += ll_cam_memcpy(cam_obj,
                            &frame_buffer_event->buf[frame_buffer_event->len],
                            &cam_obj->dma_buffer[(cnt % cam_obj->dma_half_buffer_cnt) * cam_obj->dma_half_buffer_size],
                            cam_obj->dma_half_buffer_size);
                    } else {
                        // stop if the next DMA copy would exceed the framebuffer slot
                        // size, since we're called only after the copy occurs
                        // This effectively reduces maximum usable frame buffer size
                        // by one DMA operation, as we can't predict here, if the next
                        // cam event will be a VSYNC
                        if (cnt + 1 >= cam_obj->frame_copy_cnt) {
                            ESP_CAMERA_ETS_PRINTF(DRAM_STR("cam_hal: DMA overflow\r\n"));
                            ll_cam_stop(cam_obj);
                            cam_obj->state = CAM_STATE_IDLE;
                            continue;
                        }
                    }

                    //Check for JPEG SOI in the first buffer. stop if not found
                    if (cam_obj->jpeg_mode && cnt == 0) {
                        if (cam_obj->psram_mode) {
                            /* dma_half_buffer_size already in BYTES (see ll_cam_memcpy()) */
                            size_t probe_len = cam_obj->dma_half_buffer_size;
                            /* clamp to avoid copying past the end of soi_probe */
                            if (probe_len > CAM_SOI_PROBE_BYTES) {
                                probe_len = CAM_SOI_PROBE_BYTES;
                            }
                            /* Invalidate cache lines for the DMA buffer before probing */
                            cam_drop_psram_cache(frame_buffer_event->buf, probe_len);

                            uint8_t soi_probe[CAM_SOI_PROBE_BYTES];
                            memcpy(soi_probe, frame_buffer_event->buf, probe_len);
                            int soi_off = cam_verify_jpeg_soi(soi_probe, probe_len);
                            if (soi_off != 0) {
                                static uint16_t warn_psram_soi_cnt = 0;
                                if (soi_off > 0) {
                                    CAM_WARN_THROTTLE(warn_psram_soi_cnt,
                                                      "NO-SOI - JPEG start marker not at pos 0 (PSRAM)");
                                } else {
                                    CAM_WARN_THROTTLE(warn_psram_soi_cnt,
                                                      "NO-SOI - JPEG start marker missing (PSRAM)");
                                }
                                ll_cam_stop(cam_obj);
                                cam_obj->state = CAM_STATE_IDLE;
                                continue;
                            }
                        } else {
                            int soi_off = cam_verify_jpeg_soi(frame_buffer_event->buf, frame_buffer_event->len);
                            if (soi_off != 0) {
                                static uint16_t warn_soi_bad_cnt = 0;
                                if (soi_off > 0) {
                                    CAM_WARN_THROTTLE(warn_soi_bad_cnt,
                                                      "NO-SOI - JPEG start marker not at pos 0");
                                } else {
                                    CAM_WARN_THROTTLE(warn_soi_bad_cnt,
                                                      "NO-SOI - JPEG start marker missing");
                                }
                                ll_cam_stop(cam_obj);
                                cam_obj->state = CAM_STATE_IDLE;
                                continue;
                            }
                        }
                    }

                    cnt++;

                } else if (cam_event == CAM_VSYNC_EVENT) {
                    //DBG_PIN_SET(1);
                    ll_cam_stop(cam_obj);

                    if (cnt || !cam_obj->jpeg_mode || cam_obj->psram_mode) {
                        if (cam_obj->jpeg_mode) {
                            if (!cam_obj->psram_mode) {
                                if (cam_obj->fb_size < (frame_buffer_event->len + pixels_per_dma)) {
                                    ESP_CAMERA_ETS_PRINTF(DRAM_STR("cam_hal: FB-OVF\r\n"));
                                    cnt--;
                                } else {
                                    frame_buffer_event->len += ll_cam_memcpy(cam_obj,
                                        &frame_buffer_event->buf[frame_buffer_event->len],
                                        &cam_obj->dma_buffer[(cnt % cam_obj->dma_half_buffer_cnt) * cam_obj->dma_half_buffer_size],
                                        cam_obj->dma_half_buffer_size);
                                }
                            }
                            cnt++;
                        }

                        cam_obj->frames[frame_pos].en = 0;

                        if (cam_obj->psram_mode) {
                            if (cam_obj->jpeg_mode) {
                                frame_buffer_event->len = cnt * cam_obj->dma_half_buffer_size;
                            } else {
                                frame_buffer_event->len = cam_obj->recv_size;
                            }
                        } else if (!cam_obj->jpeg_mode) {
                            if (frame_buffer_event->len != cam_obj->fb_size) {
                                cam_obj->frames[frame_pos].en = 1;
                                ESP_CAMERA_ETS_PRINTF(DRAM_STR("cam_hal: FB-SIZE: %u != %u\r\n"), frame_buffer_event->len, (unsigned) cam_obj->fb_size);
                            }
                        }
                        //send frame
                        if(!cam_obj->frames[frame_pos].en && xQueueSend(cam_obj->frame_buffer_queue, (void *)&frame_buffer_event, 0) != pdTRUE) {
                            //pop frame buffer from the queue
                            camera_fb_t * fb2 = NULL;
                            if(xQueueReceive(cam_obj->frame_buffer_queue, &fb2, 0) == pdTRUE) {
                                //push the new frame to the end of the queue
                                if (xQueueSend(cam_obj->frame_buffer_queue, (void *)&frame_buffer_event, 0) != pdTRUE) {
                                    cam_obj->frames[frame_pos].en = 1;
                                    ESP_CAMERA_ETS_PRINTF(DRAM_STR("cam_hal: FBQ-SND\r\n"));
                                }
                                //free the popped buffer
                                cam_give(fb2);
                            } else {
                                //queue is full and we could not pop a frame from it
                                cam_obj->frames[frame_pos].en = 1;
                                ESP_CAMERA_ETS_PRINTF(DRAM_STR("cam_hal: FBQ-RCV\r\n"));
                            }
                        }
                    }

                    if(!cam_start_frame(&frame_pos)){
                        cam_obj->state = CAM_STATE_IDLE;
                    } else {
                        cam_obj->frames[frame_pos].fb.len = 0;
                    }
                    cnt = 0;
                }
            }
            break;
        }
        DBG_PIN_SET(0);
    }
}

static lldesc_t * allocate_dma_descriptors(uint32_t count, uint16_t size, uint8_t * buffer)
{
    lldesc_t *dma = (lldesc_t *)heap_caps_malloc(count * sizeof(lldesc_t), MALLOC_CAP_DMA);
    if (dma == NULL) {
        return dma;
    }

    for (int x = 0; x < count; x++) {
        dma[x].size = size;
        dma[x].length = 0;
        dma[x].sosf = 0;
        dma[x].eof = 0;
        dma[x].owner = 1;
        dma[x].buf = (buffer + size * x);
        dma[x].empty = (uint32_t)&dma[(x + 1) % count];
    }
    return dma;
}

static esp_err_t cam_dma_config(const camera_config_t *config)
{
    bool ret = ll_cam_dma_sizes(cam_obj);
    if (0 == ret) {
        return ESP_FAIL;
    }

    cam_obj->dma_node_cnt = (cam_obj->dma_buffer_size) / cam_obj->dma_node_buffer_size; // Number of DMA nodes
    cam_obj->frame_copy_cnt = cam_obj->recv_size / cam_obj->dma_half_buffer_size; // Number of interrupted copies, ping-pong copy
    if (cam_obj->psram_mode) {
        cam_obj->frame_copy_cnt++;
    }

    ESP_LOGI(TAG, "buffer_size: %d, half_buffer_size: %d, node_buffer_size: %d, node_cnt: %d, total_cnt: %d",
             (int) cam_obj->dma_buffer_size, (int) cam_obj->dma_half_buffer_size, (int) cam_obj->dma_node_buffer_size,
             (int) cam_obj->dma_node_cnt, (int) cam_obj->frame_copy_cnt);

    cam_obj->dma_buffer = NULL;
    cam_obj->dma = NULL;

    cam_obj->frames = (cam_frame_t *)heap_caps_aligned_calloc(alignof(cam_frame_t), 1, cam_obj->frame_cnt * sizeof(cam_frame_t), MALLOC_CAP_DEFAULT);
    CAM_CHECK(cam_obj->frames != NULL, "frames malloc failed", ESP_FAIL);

    uint8_t dma_align = 0;
    size_t fb_size = cam_obj->fb_size;
    if (cam_obj->psram_mode) {
        dma_align = ll_cam_get_dma_align(cam_obj);
        if (cam_obj->fb_size < cam_obj->recv_size) {
            fb_size = cam_obj->recv_size;
        }
        fb_size += cam_obj->dma_half_buffer_size;
    }

    /* Allocate memory for frame buffer */
    size_t alloc_size = fb_size * sizeof(uint8_t) + dma_align;
    uint32_t _caps = MALLOC_CAP_8BIT;
    if (CAMERA_FB_IN_DRAM == config->fb_location) {
        _caps |= MALLOC_CAP_INTERNAL;
    } else {
        _caps |= MALLOC_CAP_SPIRAM;
    }
    for (int x = 0; x < cam_obj->frame_cnt; x++) {
        cam_obj->frames[x].dma = NULL;
        cam_obj->frames[x].fb_offset = 0;
        cam_obj->frames[x].en = 0;
        ESP_LOGI(TAG, "Allocating %d Byte frame buffer in %s", alloc_size, _caps & MALLOC_CAP_SPIRAM ? "PSRAM" : "OnBoard RAM");
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 3, 0)
        // In IDF v4.2 and earlier, memory returned by heap_caps_aligned_alloc must be freed using heap_caps_aligned_free.
        // And heap_caps_aligned_free is deprecated on v4.3.
        cam_obj->frames[x].fb.buf = (uint8_t *)heap_caps_aligned_alloc(16, alloc_size, _caps);
#else
        cam_obj->frames[x].fb.buf = (uint8_t *)heap_caps_malloc(alloc_size, _caps);
#endif
        CAM_CHECK(cam_obj->frames[x].fb.buf != NULL, "frame buffer malloc failed", ESP_FAIL);
        if (cam_obj->psram_mode) {
            //align PSRAM buffer. TODO: save the offset so proper address can be freed later
            cam_obj->frames[x].fb_offset = dma_align - ((uint32_t)cam_obj->frames[x].fb.buf & (dma_align - 1));
            cam_obj->frames[x].fb.buf += cam_obj->frames[x].fb_offset;
            ESP_LOGI(TAG, "Frame[%d]: Offset: %u, Addr: 0x%08X", x, cam_obj->frames[x].fb_offset, (unsigned) cam_obj->frames[x].fb.buf);
            cam_obj->frames[x].dma = allocate_dma_descriptors(cam_obj->dma_node_cnt, cam_obj->dma_node_buffer_size, cam_obj->frames[x].fb.buf);
            CAM_CHECK(cam_obj->frames[x].dma != NULL, "frame dma malloc failed", ESP_FAIL);
        }
        cam_obj->frames[x].en = 1;
    }

    if (!cam_obj->psram_mode) {
        cam_obj->dma_buffer = (uint8_t *)heap_caps_malloc(cam_obj->dma_buffer_size * sizeof(uint8_t), MALLOC_CAP_DMA);
        if(NULL == cam_obj->dma_buffer) {
            ESP_LOGE(TAG,"%s(%d): DMA buffer %d Byte malloc failed, the current largest free block:%d Byte", __FUNCTION__, __LINE__,
                     (int) cam_obj->dma_buffer_size, (int) heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
            return ESP_FAIL;
        }

        cam_obj->dma = allocate_dma_descriptors(cam_obj->dma_node_cnt, cam_obj->dma_node_buffer_size, cam_obj->dma_buffer);
        CAM_CHECK(cam_obj->dma != NULL, "dma malloc failed", ESP_FAIL);
    }

    return ESP_OK;
}

esp_err_t cam_init(const camera_config_t *config)
{
    CAM_CHECK(NULL != config, "config pointer is invalid", ESP_ERR_INVALID_ARG);

    esp_err_t ret = ESP_OK;
    cam_obj = (cam_obj_t *)heap_caps_calloc(1, sizeof(cam_obj_t), MALLOC_CAP_DMA);
    CAM_CHECK(NULL != cam_obj, "lcd_cam object malloc error", ESP_ERR_NO_MEM);

    cam_obj->swap_data = 0;
    cam_obj->vsync_pin = config->pin_vsync;
    cam_obj->vsync_invert = true;

    ll_cam_set_pin(cam_obj, config);
    ret = ll_cam_config(cam_obj, config);
    CAM_CHECK_GOTO(ret == ESP_OK, "ll_cam initialize failed", err);

#if CAMERA_DBG_PIN_ENABLE
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[DBG_PIN_NUM], PIN_FUNC_GPIO);
    gpio_set_direction(DBG_PIN_NUM, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(DBG_PIN_NUM, GPIO_FLOATING);
#endif

    ESP_LOGI(TAG, "cam init ok");
    return ESP_OK;

err:
    free(cam_obj);
    cam_obj = NULL;
    return ESP_FAIL;
}

esp_err_t cam_config(const camera_config_t *config, framesize_t frame_size, uint16_t sensor_pid)
{
    CAM_CHECK(NULL != config, "config pointer is invalid", ESP_ERR_INVALID_ARG);
    esp_err_t ret = ESP_OK;

    ret = ll_cam_set_sample_mode(cam_obj, (pixformat_t)config->pixel_format, config->xclk_freq_hz, sensor_pid);
    CAM_CHECK_GOTO(ret == ESP_OK, "ll_cam_set_sample_mode failed", err);
    
    cam_obj->jpeg_mode = config->pixel_format == PIXFORMAT_JPEG;
#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
    cam_obj->psram_mode = g_psram_dma_mode;
#else
    cam_obj->psram_mode = false;
#endif
    ESP_LOGI(TAG, "PSRAM DMA mode %s", cam_obj->psram_mode ? "enabled" : "disabled");
    cam_obj->frame_cnt = config->fb_count;
    cam_obj->width = resolution[frame_size].width;
    cam_obj->height = resolution[frame_size].height;

    if(cam_obj->jpeg_mode){
#ifdef CONFIG_CAMERA_JPEG_MODE_FRAME_SIZE_AUTO
        cam_obj->recv_size = cam_obj->width * cam_obj->height / 5;
#else
        cam_obj->recv_size = CONFIG_CAMERA_JPEG_MODE_FRAME_SIZE;
#endif
        cam_obj->fb_size = cam_obj->recv_size;
    } else {
        cam_obj->recv_size = cam_obj->width * cam_obj->height * cam_obj->in_bytes_per_pixel;
        cam_obj->fb_size = cam_obj->width * cam_obj->height * cam_obj->fb_bytes_per_pixel;
    }

    ret = cam_dma_config(config);
    CAM_CHECK_GOTO(ret == ESP_OK, "cam_dma_config failed", err);

    size_t queue_size = cam_obj->dma_half_buffer_cnt - 1;
    if (queue_size == 0) {
        queue_size = 1;
    }
    cam_obj->event_queue = xQueueCreate(queue_size, sizeof(cam_event_t));
    CAM_CHECK_GOTO(cam_obj->event_queue != NULL, "event_queue create failed", err);

    size_t frame_buffer_queue_len = cam_obj->frame_cnt;
    if (config->grab_mode == CAMERA_GRAB_LATEST && cam_obj->frame_cnt > 1) {
        frame_buffer_queue_len = cam_obj->frame_cnt - 1;
    }
    cam_obj->frame_buffer_queue = xQueueCreate(frame_buffer_queue_len, sizeof(camera_fb_t*));
    CAM_CHECK_GOTO(cam_obj->frame_buffer_queue != NULL, "frame_buffer_queue create failed", err);

    ret = ll_cam_init_isr(cam_obj);
    CAM_CHECK_GOTO(ret == ESP_OK, "cam intr alloc failed", err);


#if CONFIG_CAMERA_CORE0
    xTaskCreatePinnedToCore(cam_task, "cam_task", CAM_TASK_STACK, NULL, configMAX_PRIORITIES - 2, &cam_obj->task_handle, 0);
#elif CONFIG_CAMERA_CORE1
    xTaskCreatePinnedToCore(cam_task, "cam_task", CAM_TASK_STACK, NULL, configMAX_PRIORITIES - 2, &cam_obj->task_handle, 1);
#else
    xTaskCreate(cam_task, "cam_task", CAM_TASK_STACK, NULL, configMAX_PRIORITIES - 2, &cam_obj->task_handle);
#endif

    ESP_LOGI(TAG, "cam config ok");
    return ESP_OK;

err:
    cam_deinit();
    return ESP_FAIL;
}

esp_err_t cam_deinit(void)
{
    if (!cam_obj) {
        return ESP_FAIL;
    }

    cam_stop();
    if (cam_obj->task_handle) {
        vTaskDelete(cam_obj->task_handle);
    }
    if (cam_obj->event_queue) {
        vQueueDelete(cam_obj->event_queue);
    }
    if (cam_obj->frame_buffer_queue) {
        vQueueDelete(cam_obj->frame_buffer_queue);
    }

    ll_cam_deinit(cam_obj);

    if (cam_obj->dma) {
        free(cam_obj->dma);
    }
    if (cam_obj->dma_buffer) {
        free(cam_obj->dma_buffer);
    }
    if (cam_obj->frames) {
        for (int x = 0; x < cam_obj->frame_cnt; x++) {
            free(cam_obj->frames[x].fb.buf - cam_obj->frames[x].fb_offset);
            if (cam_obj->frames[x].dma) {
                free(cam_obj->frames[x].dma);
            }
        }
        free(cam_obj->frames);
    }

    free(cam_obj);
    cam_obj = NULL;
    return ESP_OK;
}

void cam_stop(void)
{
    ll_cam_vsync_intr_enable(cam_obj, false);
    ll_cam_stop(cam_obj);
}

void cam_start(void)
{
    ll_cam_vsync_intr_enable(cam_obj, true);
}

camera_fb_t *cam_take(TickType_t timeout)
{
    camera_fb_t *dma_buffer = NULL;
    const TickType_t start = xTaskGetTickCount();
#if CONFIG_IDF_TARGET_ESP32S3
    uint16_t dma_reset_counter = 0;
    static const uint8_t MAX_GDMA_RESETS = 3;
#else
    /* throttle repeated NULL frame warnings */
    static uint16_t warn_null_cnt = 0;
#endif
    /* throttle repeated NO-EOI warnings */
    static uint16_t warn_eoi_miss_cnt = 0;

    for (;;)
    {
        TickType_t elapsed = xTaskGetTickCount() - start; /* TickType_t is unsigned so rollover is safe */
        if (elapsed >= timeout) {
            ESP_LOGW(TAG, "Failed to get frame: timeout");
            return NULL;
        }
        TickType_t remaining = timeout - elapsed;

        if (xQueueReceive(cam_obj->frame_buffer_queue, (void *)&dma_buffer, remaining) == pdFALSE) {
            continue;
        }

        if (!dma_buffer) {
            /* Work-around for ESP32-S3 GDMA freeze when Wi-Fi STA starts.
             * See esp32-camera commit 984999f (issue #620). */
#if CONFIG_IDF_TARGET_ESP32S3
            if (dma_reset_counter < MAX_GDMA_RESETS) {
                ll_cam_dma_reset(cam_obj);
                dma_reset_counter++;
                continue; /* retry with queue timeout */
            }
            if (dma_reset_counter == MAX_GDMA_RESETS) {
                ESP_CAMERA_ETS_PRINTF(DRAM_STR("cam_hal: Giving up GDMA reset after %u tries\r\n"),
                                     (unsigned) dma_reset_counter);
                dma_reset_counter++; /* suppress further logs */
            }
#else
            /* Early warning for misbehaving sensors on other chips */
            CAM_WARN_THROTTLE(warn_null_cnt,
                              "Unexpected NULL frame on " CONFIG_IDF_TARGET);
#endif
            vTaskDelay(1); /* immediate yield once resets are done */
            continue;             /* go to top of loop */
        }

        if (cam_obj->jpeg_mode) {
            /* find the end marker for JPEG. Data after that can be discarded */
            int offset_e = -1;
            if (cam_obj->psram_mode) {
                /* Search forward from (JPEG_EOI_MARKER_LEN - 1) bytes before the final
                 * DMA block. We prefer forward search to pick the earliest EOI in the
                 * last DMA node, avoiding stale markers from a larger prior frame. */
                size_t probe_len = eoi_probe_window(cam_obj->dma_node_buffer_size,
                                                   dma_buffer->len);
                if (probe_len < JPEG_EOI_MARKER_LEN) {
                    goto skip_eoi_check;
                }
                uint8_t *probe_start = dma_buffer->buf + dma_buffer->len - probe_len;
                cam_drop_psram_cache(probe_start, probe_len);
                int off = cam_verify_jpeg_eoi(probe_start, probe_len, true);
                if (off >= 0) {
                    offset_e = dma_buffer->len - probe_len + off;
                }
            } else {
                offset_e = cam_verify_jpeg_eoi(dma_buffer->buf, dma_buffer->len, false);
            }

            if (offset_e >= 0) {
                dma_buffer->len = offset_e + JPEG_EOI_MARKER_LEN;
                if (cam_obj->psram_mode) {
                    /* DMA may bypass cache, ensure full frame is visible */
                    cam_drop_psram_cache(dma_buffer->buf, dma_buffer->len);
                }
                return dma_buffer;
            }

skip_eoi_check:

            CAM_WARN_THROTTLE(warn_eoi_miss_cnt,
                              "NO-EOI - JPEG end marker missing");
            cam_give(dma_buffer);
            continue; /* wait for another frame */
        } else if (cam_obj->psram_mode &&
                   cam_obj->in_bytes_per_pixel != cam_obj->fb_bytes_per_pixel) {
            /* currently used only for YUV to GRAYSCALE */
            dma_buffer->len = ll_cam_memcpy(cam_obj, dma_buffer->buf, dma_buffer->buf, dma_buffer->len);
        }

        if (cam_obj->psram_mode) {
            /* DMA may bypass cache, ensure full frame is visible to the app */
            cam_drop_psram_cache(dma_buffer->buf, dma_buffer->len);
        }

        return dma_buffer;
    }
}

void cam_give(camera_fb_t *dma_buffer)
{
    for (int x = 0; x < cam_obj->frame_cnt; x++) {
        if (&cam_obj->frames[x].fb == dma_buffer) {
            cam_obj->frames[x].en = 1;
            break;
        }
    }
}

void cam_give_all(void) {
    for (int x = 0; x < cam_obj->frame_cnt; x++) {
        cam_obj->frames[x].en = 1;
    }
}

bool cam_get_available_frames(void)
{
    return 0 < uxQueueMessagesWaiting(cam_obj->frame_buffer_queue);
}

void cam_set_psram_mode(bool enable)
{
    portENTER_CRITICAL(&g_psram_dma_lock);
    g_psram_dma_mode = enable;
    portEXIT_CRITICAL(&g_psram_dma_lock);
}

bool cam_get_psram_mode(void)
{
    return g_psram_dma_mode;
}
