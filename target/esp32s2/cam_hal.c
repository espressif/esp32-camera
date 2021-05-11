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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp32s2/rom/lldesc.h"
#include "soc/system_reg.h"
#include "soc/i2s_struct.h"
#include "soc/spi_struct.h"
#include "xclk.h"
#include "cam_hal.h"
#include "hal/gpio_ll.h"
#include "esp_log.h"

static const char *TAG = "lcd_cam_s2";

#define CAM_CHECK(a, str, ret) if (!(a)) {                                          \
        ESP_LOGE(TAG,"%s(%d): %s", __FUNCTION__, __LINE__, str);                    \
        return (ret);                                                               \
        }

#define CAM_CHECK_GOTO(a, str, lab) if (!(a)) {                                     \
        ESP_LOGE(TAG,"%s(%d): %s", __FUNCTION__, __LINE__, str);                    \
        goto lab;                                                                   \
        }

#define LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE  (4092)

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
    lldesc_t *dma;
    size_t fb_offset;
} cam_frame_t;

typedef struct {
    uint32_t dma_buffer_size;
    uint32_t dma_half_buffer_size;
    uint32_t dma_half_buffer_cnt;
    uint32_t dma_node_buffer_size;
    uint32_t dma_node_cnt;
    uint32_t frame_copy_cnt;
    
    //for JPEG mode
    lldesc_t *dma;
    uint8_t  *dma_buffer;
    
    cam_frame_t *frames;

    QueueHandle_t event_queue;
    QueueHandle_t frame_buffer_queue;
    TaskHandle_t task_handle;
    intr_handle_t cam_intr_handle;

    uint8_t jpeg_mode;
    uint8_t vsync_pin;
    uint8_t vsync_invert;
    uint32_t frame_cnt;
    uint32_t recv_size;
    bool swap_data;

    //for RGB/YUV modes
    uint16_t width;
    uint16_t height;

    cam_state_t state;
} cam_obj_t;

static cam_obj_t *cam_obj = NULL;

#define I2S_ISR_ENABLE(i) {I2S0.int_clr.i = 1;I2S0.int_ena.i = 1;}
#define I2S_ISR_DISABLE(i) {I2S0.int_ena.i = 0;I2S0.int_clr.i = 1;}

static void IRAM_ATTR ll_cam_vsync_isr(void *arg)
{
    cam_event_t cam_event = CAM_VSYNC_EVENT;
    BaseType_t HPTaskAwoken = pdFALSE;
    // filter
    esp_rom_delay_us(1);
    if (gpio_ll_get_level(&GPIO, cam_obj->vsync_pin) == !cam_obj->vsync_invert) {
        xQueueSendFromISR(cam_obj->event_queue, (void *)&cam_event, &HPTaskAwoken);
    }

    if (HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void IRAM_ATTR ll_cam_i2s_isr(void *arg)
{
    cam_event_t cam_event = CAM_IN_SUC_EOF_EVENT;
    BaseType_t HPTaskAwoken = pdFALSE;
    typeof(I2S0.int_st) status = I2S0.int_st;
    I2S0.int_clr.val = status.val;
    if (status.val == 0) {
        return;
    }

    if (status.in_suc_eof) {
        xQueueSendFromISR(cam_obj->event_queue, (void *)&cam_event, &HPTaskAwoken);
        //ets_printf("s");
    }

    //if (status.in_done) ets_printf("d");
    //if (status.in_err_eof) ets_printf("E");
    //if (status.in_dscr_err) ets_printf("D");
    //if (status.in_dscr_empty) ets_printf("e");
    //if (status.v_sync) ets_printf("v");

    if (HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static bool ll_cam_stop(void)
{
    if (!cam_obj->jpeg_mode || I2S0.int_ena.in_suc_eof == 1) {
        I2S0.conf.rx_start = 0;

        if (cam_obj->jpeg_mode) {
            I2S_ISR_DISABLE(in_suc_eof);
        }
        //I2S_ISR_DISABLE(v_sync);
        //I2S_ISR_DISABLE(in_done);
        //I2S_ISR_DISABLE(in_err_eof);
        //I2S_ISR_DISABLE(in_dscr_err);
        //I2S_ISR_DISABLE(in_dscr_empty);

        I2S0.in_link.stop = 1;
        return true;
    }
    return false;
}

static bool ll_cam_start(int frame_pos)
{
    if (!cam_obj->jpeg_mode || I2S0.int_ena.in_suc_eof == 0) {
        I2S0.conf.rx_start = 0;

        if (cam_obj->jpeg_mode) {
            I2S_ISR_ENABLE(in_suc_eof);
        }
        //I2S_ISR_ENABLE(v_sync);
        //I2S_ISR_ENABLE(in_done);
        //I2S_ISR_ENABLE(in_err_eof);
        //I2S_ISR_ENABLE(in_dscr_err);
        //I2S_ISR_ENABLE(in_dscr_empty);

        I2S0.conf.rx_reset = 1;
        I2S0.conf.rx_reset = 0;
        I2S0.conf.rx_fifo_reset = 1;
        I2S0.conf.rx_fifo_reset = 0;
        I2S0.lc_conf.in_rst = 1;
        I2S0.lc_conf.in_rst = 0;
        I2S0.lc_conf.ahbm_fifo_rst = 1;
        I2S0.lc_conf.ahbm_fifo_rst = 0;
        I2S0.lc_conf.ahbm_rst = 1;
        I2S0.lc_conf.ahbm_rst = 0;

        I2S0.rx_eof_num = cam_obj->dma_half_buffer_size; // Ping pong operation
        if (cam_obj->jpeg_mode) {
            I2S0.in_link.addr = ((uint32_t)&cam_obj->dma[0]) & 0xfffff;
        } else {
            I2S0.in_link.addr = ((uint32_t)&cam_obj->frames[frame_pos].dma[0]) & 0xfffff;
        }
        
        I2S0.in_link.start = 1;
        I2S0.conf.rx_start = 1;
        return true;
    }
    return false;
}

static esp_err_t ll_cam_config()
{
    // Configure the clock
    I2S0.clkm_conf.clkm_div_num = 2; // 160MHz / 2 = 80MHz
    I2S0.clkm_conf.clkm_div_b = 0;
    I2S0.clkm_conf.clkm_div_a = 0;
    I2S0.clkm_conf.clk_sel = 2;
    I2S0.clkm_conf.clk_en = 1;


    I2S0.conf.val = 0;
    I2S0.fifo_conf.val = 0;
    I2S0.fifo_conf.dscr_en = 1;

    I2S0.lc_conf.ahbm_fifo_rst = 1;
    I2S0.lc_conf.ahbm_fifo_rst = 0;
    I2S0.lc_conf.ahbm_rst = 1;
    I2S0.lc_conf.ahbm_rst = 0;
    I2S0.lc_conf.check_owner = 0;
    //I2S0.lc_conf.indscr_burst_en = 1;
    //I2S0.lc_conf.ext_mem_bk_size = 0; // DMA access external memory block size. 0: 16 bytes, 1: 32 bytes, 2:64 bytes, 3:reserved

    I2S0.timing.val = 0;

    I2S0.int_ena.val = 0;
    I2S0.int_clr.val = ~0;

    I2S0.conf2.lcd_en = 1;
    I2S0.conf2.camera_en = 1;

    // Configuration data format
    I2S0.conf.rx_slave_mod = 1;
    I2S0.conf.rx_right_first = 0;
    I2S0.conf.rx_msb_right = cam_obj->swap_data;
    I2S0.conf.rx_short_sync = 0;
    I2S0.conf.rx_mono = 0;
    I2S0.conf.rx_msb_shift = 0;
    I2S0.conf.rx_dma_equal = 1;

    // Configure sampling rate
    I2S0.sample_rate_conf.rx_bck_div_num = 1;
    I2S0.sample_rate_conf.rx_bits_mod = 8;

    I2S0.conf1.rx_pcm_bypass = 1;

    I2S0.conf2.i_v_sync_filter_en = 1;
    I2S0.conf2.i_v_sync_filter_thres = 4;
    I2S0.conf2.cam_sync_fifo_reset = 1;
    I2S0.conf2.cam_sync_fifo_reset = 0;

    I2S0.conf_chan.rx_chan_mod = 1;

    I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
    I2S0.fifo_conf.rx_data_num = 32;
    I2S0.fifo_conf.rx_fifo_mod = 2;

    I2S0.lc_conf.in_rst  = 1;
    I2S0.lc_conf.in_rst  = 0;

    I2S0.conf.rx_start = 1;

    return ESP_OK;
}

static void ll_cam_vsync_intr_enable(bool en)
{
    if (en) {
        gpio_intr_enable(cam_obj->vsync_pin);
    } else {
        gpio_intr_disable(cam_obj->vsync_pin);
    }
}

static const uint16_t JPEG_SOI_MARKER = 0xD8FF;  // written in little-endian for esp32
static const uint16_t JPEG_EOI_MARKER = 0xD9FF;  // written in little-endian for esp32

static int cam_verify_jpeg_soi(const uint8_t *inbuf, uint32_t length)
{
    for (size_t i = 0; i < length; i++) {
        uint16_t sig = *((uint16_t *)inbuf);
        if (JPEG_SOI_MARKER == sig) {
            return i;
        }
        inbuf++;
    }
    return -1;
}

static int cam_verify_jpeg_eoi(const uint8_t *inbuf, uint32_t length)
{
    int offset = -1;
    uint8_t *dptr = (uint8_t *)inbuf + length - 2;
    while (dptr > inbuf) {
        uint16_t sig = *((uint16_t *)dptr);
        if (JPEG_EOI_MARKER == sig) {
            offset = dptr - inbuf;
            return offset;
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
        if(ll_cam_start(*frame_pos)){
            // Vsync the frame manually
            ll_cam_vsync_intr_enable(false);
            gpio_matrix_in(cam_obj->vsync_pin, I2S0I_V_SYNC_IDX, !cam_obj->vsync_invert);
            esp_rom_delay_us(10);
            gpio_matrix_in(cam_obj->vsync_pin, I2S0I_V_SYNC_IDX, cam_obj->vsync_invert);
            ll_cam_vsync_intr_enable(true);
            return true;
        }
    }
    return false;
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
        switch (cam_obj->state) {

            case CAM_STATE_IDLE: {
                if (cam_event == CAM_VSYNC_EVENT) {
                    if(cam_start_frame(&frame_pos)){
                        cam_obj->state = CAM_STATE_READ_BUF;
                    }
                    cnt = 0;
                }
            }
            break;

            case CAM_STATE_READ_BUF: {
                if (cam_event == CAM_IN_SUC_EOF_EVENT) {
                    if(cam_obj->jpeg_mode){
                        memcpy(&cam_obj->frames[frame_pos].fb.buf[cnt * cam_obj->dma_half_buffer_size], &cam_obj->dma_buffer[(cnt % cam_obj->dma_half_buffer_cnt) * cam_obj->dma_half_buffer_size], cam_obj->dma_half_buffer_size);
                    }
                    cnt++;

                } else if (cam_event == CAM_VSYNC_EVENT) {
                    ll_cam_stop();

                    int offset_e = -1;
                    if (cam_obj->jpeg_mode) {
                        // check for the end marker for JPEG in the last buffer
                        offset_e = cam_verify_jpeg_eoi(&cam_obj->frames[frame_pos].fb.buf[(cnt - 1) * cam_obj->dma_half_buffer_size], cam_obj->dma_half_buffer_size);
                        if (offset_e < 0) {
                            memcpy(&cam_obj->frames[frame_pos].fb.buf[cnt * cam_obj->dma_half_buffer_size], &cam_obj->dma_buffer[(cnt % cam_obj->dma_half_buffer_cnt) * cam_obj->dma_half_buffer_size], cam_obj->dma_half_buffer_size);
                            cnt++;
                        } else {
                            offset_e += ((cnt - 1) * cam_obj->dma_half_buffer_size);
                        }
                    }


                    camera_fb_t * frame_buffer_event = &cam_obj->frames[frame_pos].fb;
                    cam_obj->frames[frame_pos].en = 0;

                    if (cam_obj->jpeg_mode) {
                        frame_buffer_event->len = cnt * cam_obj->dma_half_buffer_size;
                        // find the start marker for JPEG.
                        int offset_s = cam_verify_jpeg_soi(frame_buffer_event->buf, 1);
                        if (offset_s == 0) {
                            // find the end marker for JPEG. Data after that can be discarded
                            if (offset_e < 0) {
                                offset_e = cam_verify_jpeg_eoi(frame_buffer_event->buf, frame_buffer_event->len);
                            }
                            if (offset_e >= 0) {
                                // adjust buffer length
                                frame_buffer_event->len = offset_e + sizeof(JPEG_EOI_MARKER);
                                //send frame
                                if (xQueueSend(cam_obj->frame_buffer_queue, (void *)&frame_buffer_event, 0) != pdTRUE) {
                                    cam_obj->frames[frame_pos].en = 1;
                                    ESP_LOGE(TAG, "fbqueue_send_fail\n");
                                }
                            } else {
                                cam_obj->frames[frame_pos].en = 1;
                                //ets_printf("no eoi: %u\n", frame_buffer_event->len);
                            }
                        } else {
                            cam_obj->frames[frame_pos].en = 1;
                            //ets_printf("no soi\n");
                        }
                    } else {
                        frame_buffer_event->len = cam_obj->recv_size;
                        //send frame
                        if (xQueueSend(cam_obj->frame_buffer_queue, (void *)&frame_buffer_event, 0) != pdTRUE) {
                            cam_obj->frames[frame_pos].en = 1;
                            ESP_LOGE(TAG, "fbqueue_send_fail\n");
                        }
                    }

                    if(!cam_start_frame(&frame_pos)){
                        cam_obj->state = CAM_STATE_IDLE;
                    }
                    cnt = 0;
                }
            }
            break;
        }
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
        dma[x].eof = 0;
        dma[x].owner = 1;
        dma[x].buf = (buffer + size * x);
        dma[x].empty = (uint32_t)&dma[(x + 1) % count];
    }
    return dma;
}

static esp_err_t cam_dma_config()
{
    int cnt = 0;
    
    if (cam_obj->jpeg_mode) {
        cam_obj->dma_half_buffer_cnt = 16;
        cam_obj->dma_buffer_size = cam_obj->dma_half_buffer_cnt * 1024;
        cam_obj->dma_half_buffer_size = cam_obj->dma_buffer_size / cam_obj->dma_half_buffer_cnt;
        cam_obj->dma_node_buffer_size = cam_obj->dma_half_buffer_size;
    } else {
        for (cnt = 0; cnt < cam_obj->recv_size; cnt++) { // Find a buffer size that can be divisible by
            if (cam_obj->recv_size % (cam_obj->recv_size - cnt) == 0) {
                break;
            }
        }
        cam_obj->dma_buffer_size = cam_obj->recv_size - cnt;
        cam_obj->dma_half_buffer_cnt = 2;
        cam_obj->dma_half_buffer_size = cam_obj->dma_buffer_size / cam_obj->dma_half_buffer_cnt;

        for (cnt = 0; cnt < LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE; cnt++) { // Find a divisible dma size
            if ((cam_obj->dma_half_buffer_size) % (LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE - cnt) == 0) {
                break;
            }
        }
        cam_obj->dma_node_buffer_size = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE - cnt;
    }

    cam_obj->dma_node_cnt = (cam_obj->dma_buffer_size) / cam_obj->dma_node_buffer_size; // Number of DMA nodes
    cam_obj->frame_copy_cnt = cam_obj->recv_size / cam_obj->dma_half_buffer_size; // Number of interrupted copies, ping-pong copy

    ESP_LOGI(TAG, "buffer_size: %d, half_buffer_size: %d, node_buffer_size: %d, node_cnt: %d, total_cnt: %d\n", cam_obj->dma_buffer_size, cam_obj->dma_half_buffer_size, cam_obj->dma_node_buffer_size, cam_obj->dma_node_cnt, cam_obj->frame_copy_cnt);

    cam_obj->dma_buffer = NULL;
    cam_obj->dma = NULL;

    cam_obj->frames = (cam_frame_t *)heap_caps_malloc(cam_obj->frame_cnt * sizeof(cam_frame_t), MALLOC_CAP_DEFAULT);
    CAM_CHECK(cam_obj->frames != NULL, "frames malloc failed", ESP_FAIL);

    uint8_t dma_align = 0;
    if (!cam_obj->jpeg_mode) {
        dma_align = 16 << I2S0.lc_conf.ext_mem_bk_size;
    }
    for (int x = 0; x < cam_obj->frame_cnt; x++) {
        cam_obj->frames[x].dma = NULL;
        cam_obj->frames[x].fb_offset = 0;
        cam_obj->frames[x].en = 0;
        cam_obj->frames[x].fb.buf = (uint8_t *)heap_caps_malloc(cam_obj->recv_size * sizeof(uint8_t) + dma_align, MALLOC_CAP_SPIRAM);
        CAM_CHECK(cam_obj->frames[x].fb.buf != NULL, "frame buffer malloc failed", ESP_FAIL);
        if (!cam_obj->jpeg_mode) {
            //align PSRAM buffer. TODO: save the offset so proper address can be freed later
            cam_obj->frames[x].fb_offset = dma_align - ((uint32_t)cam_obj->frames[x].fb.buf & (dma_align - 1));
            cam_obj->frames[x].fb.buf += cam_obj->frames[x].fb_offset;
            ESP_LOGI(TAG, "Frame[%d]: Offset: %u, Addr: 0x%08X", x, cam_obj->frames[x].fb_offset, (uint32_t)cam_obj->frames[x].fb.buf);
            cam_obj->frames[x].dma = allocate_dma_descriptors(cam_obj->dma_node_cnt, cam_obj->dma_node_buffer_size, cam_obj->frames[x].fb.buf);
            CAM_CHECK(cam_obj->frames[x].dma != NULL, "frame dma malloc failed", ESP_FAIL);
        }
        cam_obj->frames[x].en = 1;
    }

    if (cam_obj->jpeg_mode) {
        cam_obj->dma_buffer = (uint8_t *)heap_caps_malloc(cam_obj->dma_buffer_size * sizeof(uint8_t), MALLOC_CAP_DMA);
        CAM_CHECK(cam_obj->dma_buffer != NULL, "dma_buffer malloc failed", ESP_FAIL);

        cam_obj->dma = allocate_dma_descriptors(cam_obj->dma_node_cnt, cam_obj->dma_node_buffer_size, cam_obj->dma_buffer);
        CAM_CHECK(cam_obj->dma != NULL, "dma malloc failed", ESP_FAIL);
    }

    return ESP_OK;
}



















static esp_err_t cam_set_pin(const camera_config_t *config)
{
    gpio_config_t io_conf = {0};
    io_conf.intr_type = cam_obj->vsync_invert ? GPIO_PIN_INTR_NEGEDGE : GPIO_PIN_INTR_POSEDGE;
    io_conf.pin_bit_mask = 1ULL << config->pin_vsync;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(config->pin_vsync, ll_cam_vsync_isr, NULL);
    gpio_intr_disable(config->pin_vsync);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin_pclk], PIN_FUNC_GPIO);
    gpio_set_direction(config->pin_pclk, GPIO_MODE_INPUT);
    gpio_set_pull_mode(config->pin_pclk, GPIO_FLOATING);
    gpio_matrix_in(config->pin_pclk, I2S0I_WS_IN_IDX, false);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin_vsync], PIN_FUNC_GPIO);
    gpio_set_direction(config->pin_vsync, GPIO_MODE_INPUT);
    gpio_set_pull_mode(config->pin_vsync, GPIO_FLOATING);
    gpio_matrix_in(config->pin_vsync, I2S0I_V_SYNC_IDX, cam_obj->vsync_invert);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin_href], PIN_FUNC_GPIO);
    gpio_set_direction(config->pin_href, GPIO_MODE_INPUT);
    gpio_set_pull_mode(config->pin_href, GPIO_FLOATING);
    gpio_matrix_in(config->pin_href, I2S0I_H_SYNC_IDX, false);

    int data_pins[8] = {
        config->pin_d0, config->pin_d1, config->pin_d2, config->pin_d3, config->pin_d4, config->pin_d5, config->pin_d6, config->pin_d7,
    };
    for (int i = 0; i < 8; i++) {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[data_pins[i]], PIN_FUNC_GPIO);
        gpio_set_direction(data_pins[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(data_pins[i], GPIO_FLOATING);
        // High bit alignment, IN16 is always the highest bit
        // fifo accesses data by bit, when rx_bits_mod is 8, the data needs to be aligned by 8 bits
        gpio_matrix_in(data_pins[i], I2S0I_DATA_IN0_IDX + 8 + i, false);
    }

    esp_err_t err = camera_enable_out_clock(config);
    if(err != ESP_OK) {
        return err;
    }

    gpio_matrix_in(0x38, I2S0I_H_ENABLE_IDX, false);

    return ESP_OK;
}

esp_err_t cam_init(const camera_config_t *config)
{
    CAM_CHECK(NULL != config, "config pointer is invalid", ESP_ERR_INVALID_ARG);

    esp_err_t ret = ESP_OK;
    cam_obj = (cam_obj_t *)heap_caps_calloc(1, sizeof(cam_obj_t), MALLOC_CAP_DMA);
    CAM_CHECK(NULL != cam_obj, "lcd_cam object malloc error", ESP_ERR_NO_MEM);

    cam_obj->jpeg_mode = config->pixel_format == PIXFORMAT_JPEG;
    cam_obj->vsync_pin = config->pin_vsync;
    cam_obj->vsync_invert = true;
    cam_obj->frame_cnt = config->fb_count;
    cam_obj->width = resolution[config->frame_size].width;
    cam_obj->height = resolution[config->frame_size].height;
    cam_obj->swap_data = 0;

    if(cam_obj->jpeg_mode){
        cam_obj->recv_size = cam_obj->width * cam_obj->height / 5;
    } else {
        cam_obj->recv_size = cam_obj->width * cam_obj->height * 2;
    }
    
    cam_obj->event_queue = xQueueCreate(16, sizeof(cam_event_t));
    CAM_CHECK_GOTO(cam_obj->event_queue != NULL, "event_queue create failed", err);

    cam_obj->frame_buffer_queue = xQueueCreate(cam_obj->frame_cnt, sizeof(camera_fb_t));
    CAM_CHECK_GOTO(cam_obj->frame_buffer_queue != NULL, "frame_buffer_queue create failed", err);

    cam_set_pin(config);
    periph_module_enable(PERIPH_I2S0_MODULE);
    ll_cam_config();

    ret = esp_intr_alloc(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, ll_cam_i2s_isr, NULL, &cam_obj->cam_intr_handle);
    CAM_CHECK_GOTO(ret == ESP_OK, "cam intr alloc failed", err);

    ret = cam_dma_config();
    CAM_CHECK_GOTO(ret == ESP_OK, "cam_dma_config failed", err);

    xTaskCreate(cam_task, "cam_task", 1024, NULL, configMAX_PRIORITIES / 2, &cam_obj->task_handle);

    ESP_LOGI(TAG, "cam init ok");
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
    gpio_isr_handler_remove(cam_obj->vsync_pin);
    if (cam_obj->task_handle) {
        vTaskDelete(cam_obj->task_handle);
    }
    if (cam_obj->event_queue) {
        vQueueDelete(cam_obj->event_queue);
    }
    if (cam_obj->frame_buffer_queue) {
        vQueueDelete(cam_obj->frame_buffer_queue);
    }
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

    if (cam_obj->cam_intr_handle) {
        esp_intr_free(cam_obj->cam_intr_handle);
    }

    free(cam_obj);
    cam_obj = NULL;
    return ESP_OK;
}

void cam_stop(void)
{
    ll_cam_vsync_intr_enable(false);
    ll_cam_stop();
}

void cam_start(void)
{
    ll_cam_vsync_intr_enable(true);
}

camera_fb_t *cam_take(void)
{
    camera_fb_t *dma_buffer;
    xQueueReceive(cam_obj->frame_buffer_queue, (void *)&dma_buffer, portMAX_DELAY);
    return dma_buffer;
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
