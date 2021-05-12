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
#include "esp32s3/rom/lldesc.h"
#include "esp32s3/rom/gpio.h"
#include "soc/periph_defs.h"
#include "soc/io_mux_reg.h"
#include "soc/system_reg.h"
#include "soc/timer_group_struct.h"
#include "soc/lcd_cam_struct.h"
#include "soc/spi_struct.h"
#include "soc/gdma_struct.h"
#include "soc/interrupt_core0_reg.h"
#include "soc/system_reg.h"
#include "driver/gpio.h"
#include "cam_hal.h"
#include "esp_log.h"

static const char *TAG = "s3 camera";

#define CAM_CHECK(a, str, ret) if (!(a)) {                                          \
        ESP_LOGE(TAG,"%s(%d): %s", __FUNCTION__, __LINE__, str);                    \
        return (ret);                                                               \
        }

#define CAM_CHECK_GOTO(a, str, lab) if (!(a)) {                                     \
        ESP_LOGE(TAG,"%s(%d): %s", __FUNCTION__, __LINE__, str);                    \
        goto lab;                                                                   \
        }

#define CAM_DATA_MAX_WIDTH (16)  /*!< Maximum width of CAM data bus */
#define LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE  (4000)
#define LCD_CAM_DMA_MAX_NUM               (5) // Maximum number of DMA channels

static const uint16_t JPEG_SOI_MARKER = 0xD8FF;  // written in little-endian for esp32
static const uint16_t JPEG_EOI_MARKER = 0xD9FF;  // written in little-endian for esp32

typedef enum {
    CAM_IN_SUC_EOF_EVENT = 0,
    CAM_VSYNC_EVENT
} cam_event_t;

typedef struct {
    camera_fb_t fb_event;
    uint8_t is_full;
} frame_buffer_t;

typedef enum {
    CAM_STATE_IDLE = 0,
    CAM_STATE_READ_BUF = 1,
} cam_state_t;

typedef struct {
    uint32_t dma_buffer_size;
    uint32_t dma_half_buffer_size;
    uint32_t dma_node_buffer_size;
    uint32_t dma_node_cnt;
    uint32_t frame_copy_cnt;
    uint32_t frame_cnt;
    uint32_t recv_size;
    lldesc_t *dma;
    uint8_t  *dma_buffer;
    frame_buffer_t *frame_buffer;
    QueueHandle_t event_queue;
    QueueHandle_t frame_buffer_queue;
    TaskHandle_t task_handle;
    uint8_t jpeg_mode;
    uint8_t vsync_pin;
    uint8_t vsync_invert;
    bool swap_data;
} cam_obj_t;

typedef struct {
    camera_config_t cfg;
    cam_obj_t cam;
    uint8_t dma_num;
    intr_handle_t lcd_cam_intr_handle;
    intr_handle_t dma_intr_handle;
} lcd_cam_obj_t;

static lcd_cam_obj_t *lcd_cam_obj = NULL;

// static IRAM_ATTR void io_probe(uint8_t io_num, uint8_t pulse)
// {
//     for (size_t i = 0; i < pulse; i++) {
//         GPIO.out_w1ts = (1 << io_num);
//         GPIO.out_w1tc = (1 << io_num);
//     }
// }

static void IRAM_ATTR lcd_cam_isr(void *arg)
{
    //GPIO.out_w1ts = (1 << 20);

    typeof(LCD_CAM.lc_dma_int_st) status = LCD_CAM.lc_dma_int_st;
    BaseType_t HPTaskAwoken = pdFALSE;
    if (status.val == 0) {
        return;
    }
    LCD_CAM.lc_dma_int_clr.val = status.val;
    if (status.cam_vsync) {
        cam_event_t cam_event = CAM_VSYNC_EVENT;
        xQueueSendFromISR(lcd_cam_obj->cam.event_queue, (void *)&cam_event, &HPTaskAwoken);
    }

    if (HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
    //GPIO.out_w1tc = (1 << 20);
}

static void IRAM_ATTR dma_isr(void *arg)
{
    BaseType_t HPTaskAwoken = pdFALSE;
    typeof(GDMA.in[lcd_cam_obj->dma_num].int_st) status = GDMA.in[lcd_cam_obj->dma_num].int_st;
    if (status.val == 0) {
        return;
    }
    GDMA.in[lcd_cam_obj->dma_num].int_clr.val = status.val;
    // handle RX interrupt */
    if (status.in_suc_eof) {
        cam_event_t cam_event = CAM_IN_SUC_EOF_EVENT;
        xQueueSendFromISR(lcd_cam_obj->cam.event_queue, (void *)&cam_event, &HPTaskAwoken);
    }

    if (HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static esp_err_t cam_frame_buffer_deinit(void)
{
    if (lcd_cam_obj->cam.frame_buffer) {
        for (size_t i = 0; i < lcd_cam_obj->cam.frame_cnt; i++) {
            if (lcd_cam_obj->cam.frame_buffer[i].fb_event.buf) {
                free(lcd_cam_obj->cam.frame_buffer[i].fb_event.buf);
                lcd_cam_obj->cam.frame_buffer[i].fb_event.buf = NULL;
            }
        }
    }
    free(lcd_cam_obj->cam.frame_buffer);
    lcd_cam_obj->cam.frame_buffer = NULL;
    return ESP_OK;
}

static esp_err_t cam_frame_buffer_init(void)
{
    CAM_CHECK(NULL == lcd_cam_obj->cam.frame_buffer, "frame buffer already init", ESP_ERR_INVALID_STATE);

    lcd_cam_obj->cam.frame_buffer = (frame_buffer_t *)heap_caps_calloc(lcd_cam_obj->cam.frame_cnt, sizeof(frame_buffer_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    CAM_CHECK(lcd_cam_obj->cam.frame_buffer != NULL, "frame_buffer malloc failed", ESP_ERR_NO_MEM);

    bool use_spiram = 0;
    if (heap_caps_get_free_size(MALLOC_CAP_INTERNAL) < (20 * 1024 + lcd_cam_obj->cam.recv_size * lcd_cam_obj->cam.frame_cnt)) {
        use_spiram = 1;
    }

    for (size_t i = 0; i < lcd_cam_obj->cam.frame_cnt; i++) {
        uint8_t *buf_temp = NULL;
        if (use_spiram) {
            ESP_LOGI(TAG, "Allocating %d KB frame buffer in PSRAM", lcd_cam_obj->cam.recv_size / 1024);
            buf_temp = (uint8_t *) heap_caps_calloc(1, lcd_cam_obj->cam.recv_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        } else {
            ESP_LOGI(TAG, "Allocating %d KB frame buffer in OnBoard RAM", lcd_cam_obj->cam.recv_size / 1024);
            buf_temp = (uint8_t *) heap_caps_calloc(1, lcd_cam_obj->cam.recv_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        }
        if (NULL == buf_temp) {
            ESP_LOGE(TAG, "Allocating %d KB frame buffer Failed", lcd_cam_obj->cam.recv_size / 1024);
            goto fail;
        }
        lcd_cam_obj->cam.frame_buffer[i].fb_event.buf = buf_temp;
    }
    return ESP_OK;
fail:
    cam_frame_buffer_deinit();
    return ESP_ERR_NO_MEM;
}

static void lcd_cam_config(const camera_config_t *config)
{
    if (REG_GET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_LCD_CAM_CLK_EN) == 0) {
        REG_CLR_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_LCD_CAM_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_LCD_CAM_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_LCD_CAM_RST);
        REG_CLR_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_LCD_CAM_RST);
    }

    LCD_CAM.cam_ctrl.val = 0;
    LCD_CAM.cam_ctrl.cam_clkm_div_b = 0;
    LCD_CAM.cam_ctrl.cam_clkm_div_a = 10;
    LCD_CAM.cam_ctrl.cam_clkm_div_num = 160000000 / config->xclk_freq_hz;
    LCD_CAM.cam_ctrl.cam_clk_sel = 3;
    LCD_CAM.cam_ctrl.cam_stop_en = 0;
    LCD_CAM.cam_ctrl.cam_vsync_filter_thres = 7 - 1; // Filter by LCD_CAM clock
    LCD_CAM.cam_ctrl.cam_update = 0;
    LCD_CAM.cam_ctrl.cam_byte_order = 0;
    LCD_CAM.cam_ctrl.cam_bit_order = 0;
    LCD_CAM.cam_ctrl.cam_line_int_en = 0;
    LCD_CAM.cam_ctrl.cam_vs_eof_en = 0; //1: CAM_VSYNC to generate in_suc_eof. 0: in_suc_eof is controlled by reg_cam_rec_data_cyclelen
    LCD_CAM.cam_ctrl1.val = 0;
    LCD_CAM.cam_ctrl1.cam_rec_data_bytelen = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE - 1; // Cannot be assigned to 0, and it is easy to overflow
    LCD_CAM.cam_ctrl1.cam_line_int_num = 1  - 1; // The number of hsyncs that generate hs interrupts
    LCD_CAM.cam_ctrl1.cam_clk_inv = 0;
    LCD_CAM.cam_ctrl1.cam_vsync_filter_en = 1;
    LCD_CAM.cam_ctrl1.cam_2byte_en = 0;
    LCD_CAM.cam_ctrl1.cam_de_inv = 0;
    LCD_CAM.cam_ctrl1.cam_hsync_inv = 0;
    LCD_CAM.cam_ctrl1.cam_vsync_inv = 0;
    LCD_CAM.cam_ctrl1.cam_vh_de_mode_en = 0;

    LCD_CAM.cam_ctrl.cam_update = 1;
    LCD_CAM.cam_ctrl1.cam_start = 1;
}


static void cam_vsync_intr_enable(bool en)
{
    LCD_CAM.lc_dma_int_clr.cam_vsync = 1;
    if (en) {
        LCD_CAM.lc_dma_int_ena.cam_vsync = 1;
    } else {
        LCD_CAM.lc_dma_int_ena.cam_vsync = 0;
    }
}

static void cam_dma_stop(void)
{
    if (GDMA.in[lcd_cam_obj->dma_num].int_ena.in_suc_eof == 1) {
        GDMA.in[lcd_cam_obj->dma_num].int_ena.in_suc_eof = 0;
        GDMA.in[lcd_cam_obj->dma_num].int_clr.in_suc_eof = 1;
        GDMA.in[lcd_cam_obj->dma_num].link.stop = 1;
        LCD_CAM.cam_ctrl.cam_update = 1;
    }
}

static void cam_dma_start(void)
{
    if (GDMA.in[lcd_cam_obj->dma_num].int_ena.in_suc_eof == 0) {
        LCD_CAM.cam_ctrl1.cam_start = 0;
        GDMA.in[lcd_cam_obj->dma_num].int_clr.in_suc_eof = 1;
        GDMA.in[lcd_cam_obj->dma_num].int_ena.in_suc_eof = 1;
        // LCD_CAM.cam_ctrl1.cam_reset = 1;
        // LCD_CAM.cam_ctrl1.cam_reset = 0;
        LCD_CAM.cam_ctrl1.cam_afifo_reset = 1;
        LCD_CAM.cam_ctrl1.cam_afifo_reset = 0;
        GDMA.in[lcd_cam_obj->dma_num].conf0.in_rst = 1;
        GDMA.in[lcd_cam_obj->dma_num].conf0.in_rst = 0;
        GDMA.in[lcd_cam_obj->dma_num].link.start = 1;
        LCD_CAM.cam_ctrl.cam_update = 1;
        LCD_CAM.cam_ctrl1.cam_start = 1;
    }
}

static void cam_memcpy(uint8_t *out, const uint8_t *in, size_t len)
{
    if (lcd_cam_obj->cam.swap_data) {
        int cnt = len - len % 2;
        for (int x = 0; x < cnt; x += 2) {
            out[x + 1] = in[x + 0];
            out[x + 0] = in[x + 1];
        }
        if (len % 2) {
            out[cnt] = in[cnt];
        }
    } else {
        memcpy(out, in, len);
    }
}

static void cam_set_pin(const camera_config_t *config)
{
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin_href], PIN_FUNC_GPIO);
    gpio_set_direction(config->pin_href, GPIO_MODE_INPUT);
    gpio_set_pull_mode(config->pin_href, GPIO_FLOATING);
    gpio_matrix_in(config->pin_href, CAM_H_ENABLE_IDX, false);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin_vsync], PIN_FUNC_GPIO);
    gpio_set_direction(config->pin_vsync, GPIO_MODE_INPUT);
    gpio_set_pull_mode(config->pin_vsync, GPIO_FLOATING);
    gpio_matrix_in(config->pin_vsync, CAM_V_SYNC_IDX, true);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin_pclk], PIN_FUNC_GPIO);
    gpio_set_direction(config->pin_pclk, GPIO_MODE_INPUT);
    gpio_set_pull_mode(config->pin_pclk, GPIO_FLOATING);
    gpio_matrix_in(config->pin_pclk, CAM_PCLK_IDX, false);

    int data_pins[8] = {
        config->pin_d0, config->pin_d1, config->pin_d2, config->pin_d3, config->pin_d4, config->pin_d5, config->pin_d6, config->pin_d7,
    };
    for (int i = 0; i < 8; i++) {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[data_pins[i]], PIN_FUNC_GPIO);
        gpio_set_direction(data_pins[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(data_pins[i], GPIO_FLOATING);
        gpio_matrix_in(data_pins[i], CAM_DATA_IN0_IDX + i, false);
    }

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin_xclk], PIN_FUNC_GPIO);
    gpio_set_direction(config->pin_xclk, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(config->pin_xclk, GPIO_FLOATING);
    gpio_matrix_out(config->pin_xclk, CAM_CLK_IDX, false, false);
}

static esp_err_t cam_dma_config(uint8_t is_jpeg, size_t max_dma_buffer_size)
{
    int cnt = 0;
    if (is_jpeg) {
        lcd_cam_obj->cam.dma_buffer_size = 2048;
        lcd_cam_obj->cam.dma_half_buffer_size = lcd_cam_obj->cam.dma_buffer_size / 2;
        lcd_cam_obj->cam.dma_node_buffer_size = lcd_cam_obj->cam.dma_half_buffer_size;
    } else {
        if (max_dma_buffer_size / 2.0 > 16384) { // must less than max(cam_rec_data_bytelen)
            max_dma_buffer_size = 16384 * 2;
        }
        for (cnt = 0; cnt < max_dma_buffer_size; cnt++) { // Find a buffer size that can be divisible by
            if (lcd_cam_obj->cam.recv_size % (max_dma_buffer_size - cnt) == 0) {
                break;
            }
        }
        lcd_cam_obj->cam.dma_buffer_size = max_dma_buffer_size - cnt;
        lcd_cam_obj->cam.dma_half_buffer_size = lcd_cam_obj->cam.dma_buffer_size / 2;
        for (cnt = 0; cnt < LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE; cnt++) { // Find a divisible dma size
            if ((lcd_cam_obj->cam.dma_half_buffer_size) % (LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE - cnt) == 0) {
                break;
            }
        }
        lcd_cam_obj->cam.dma_node_buffer_size = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE - cnt;
    }

    lcd_cam_obj->cam.dma_node_cnt = (lcd_cam_obj->cam.dma_buffer_size) / lcd_cam_obj->cam.dma_node_buffer_size; // Number of DMA nodes
    lcd_cam_obj->cam.frame_copy_cnt = lcd_cam_obj->cam.recv_size / lcd_cam_obj->cam.dma_half_buffer_size; // Number of interrupted copies, ping-pong copy

    ESP_LOGI(TAG, "cam_buffer_size: %d, cam_dma_size: %d, cam_dma_node_cnt: %d, cam_total_cnt: %d",
             lcd_cam_obj->cam.dma_buffer_size, lcd_cam_obj->cam.dma_node_buffer_size, lcd_cam_obj->cam.dma_node_cnt, lcd_cam_obj->cam.frame_copy_cnt);

    lcd_cam_obj->cam.dma = (lldesc_t *)heap_caps_malloc(lcd_cam_obj->cam.dma_node_cnt * sizeof(lldesc_t), MALLOC_CAP_DMA);
    CAM_CHECK(NULL != lcd_cam_obj->cam.dma, "dma node malloc failed", ESP_ERR_NO_MEM);
    lcd_cam_obj->cam.dma_buffer = (uint8_t *)heap_caps_malloc(lcd_cam_obj->cam.dma_buffer_size * sizeof(uint8_t), MALLOC_CAP_DMA);
    if (NULL == lcd_cam_obj->cam.dma_buffer) {
        heap_caps_free(lcd_cam_obj->cam.dma);
        CAM_CHECK(false, "dma buffer malloc failed", ESP_ERR_NO_MEM);
    }

    for (int x = 0; x < lcd_cam_obj->cam.dma_node_cnt; x++) {
        lcd_cam_obj->cam.dma[x].size = lcd_cam_obj->cam.dma_node_buffer_size;
        lcd_cam_obj->cam.dma[x].length = lcd_cam_obj->cam.dma_node_buffer_size;
        lcd_cam_obj->cam.dma[x].eof = 0;
        lcd_cam_obj->cam.dma[x].owner = 1;
        lcd_cam_obj->cam.dma[x].buf = (lcd_cam_obj->cam.dma_buffer + lcd_cam_obj->cam.dma_node_buffer_size * x);
        lcd_cam_obj->cam.dma[x].empty = (uint32_t)&lcd_cam_obj->cam.dma[(x + 1) % lcd_cam_obj->cam.dma_node_cnt];
    }

    if (REG_GET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN) == 0) {
        REG_CLR_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_DMA_RST);
        REG_CLR_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_DMA_RST);
    }

    GDMA.in[lcd_cam_obj->dma_num].conf0.val = 0;
    GDMA.in[lcd_cam_obj->dma_num].conf1.val = 0;
    GDMA.in[lcd_cam_obj->dma_num].int_clr.val = ~0;
    GDMA.in[lcd_cam_obj->dma_num].int_ena.val = 0;

    GDMA.in[lcd_cam_obj->dma_num].conf0.in_rst = 1;
    GDMA.in[lcd_cam_obj->dma_num].conf0.in_rst = 0;
    GDMA.in[lcd_cam_obj->dma_num].conf0.indscr_burst_en = 1;
    GDMA.in[lcd_cam_obj->dma_num].conf0.in_data_burst_en = 1;
    GDMA.in[lcd_cam_obj->dma_num].peri_sel.sel = 5;
    GDMA.in[lcd_cam_obj->dma_num].pri.rx_pri = 1;

    GDMA.in[lcd_cam_obj->dma_num].link.addr = ((uint32_t)&lcd_cam_obj->cam.dma[0]) & 0xfffff;
    LCD_CAM.cam_ctrl1.cam_rec_data_bytelen = lcd_cam_obj->cam.dma_half_buffer_size - 1; // Ping pong operation
    return ESP_OK;
}

static int search_soi(const uint8_t *inbuf, uint32_t length)
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

static int search_eoi(const uint8_t *inbuf, uint32_t length)
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

static int find_empty_fb(void)
{
    for (int x = 0; x < lcd_cam_obj->cam.frame_cnt; x++) {
        if (0 == lcd_cam_obj->cam.frame_buffer[x].is_full) {
            return x;
        }
    }
    return -1;
}

// Copy fram from DMA dma_buffer to fram dma_buffer
static void cam_task(void *arg)
{
    int cnt = 0;
    int frame_pos = 0;
    cam_state_t state = CAM_STATE_IDLE;
    cam_event_t cam_event = {0};
    xQueueReset(lcd_cam_obj->cam.event_queue);

    // gpio_pad_select_gpio(20);
    // gpio_set_direction(20, GPIO_MODE_OUTPUT);
    // gpio_set_level(20, 0);

    // gpio_pad_select_gpio(9);
    // gpio_set_direction(9, GPIO_MODE_OUTPUT);
    // gpio_set_level(9, 0);

    while (1) {
        xQueueReceive(lcd_cam_obj->cam.event_queue, (void *)&cam_event, portMAX_DELAY);
        // io_probe(9, cam_event + 1);
        camera_fb_t *fb_event = &lcd_cam_obj->cam.frame_buffer[frame_pos].fb_event;
        frame_buffer_t *frame_buffer = &lcd_cam_obj->cam.frame_buffer[frame_pos];

        switch (state) {
        case CAM_STATE_IDLE: {
            if (cam_event == CAM_VSYNC_EVENT) {
                int fb_index = find_empty_fb();
                if (fb_index >= 0) {
                    frame_pos = fb_index;
                    cam_dma_stop();
                    ets_delay_us(80);
                    cam_dma_start();
                    // io_probe(9, 3);
                    state = CAM_STATE_READ_BUF;
                }
                cnt = 0;
            }
        } break;

        case CAM_STATE_READ_BUF: {
            if (cam_event == CAM_IN_SUC_EOF_EVENT) {

                // if (0 == cnt && 16 == search_soi(fb_event->buf, fb_event->len)) {
                //     cam_memcpy(fb_event->buf + (cnt * lcd_cam_obj->cam.dma_half_buffer_size),
                //                &lcd_cam_obj->cam.dma_buffer[(cnt % 2) * lcd_cam_obj->cam.dma_half_buffer_size] + 16,
                //                lcd_cam_obj->cam.dma_half_buffer_size - 16);
                // } else {
                cam_memcpy(fb_event->buf + (cnt * lcd_cam_obj->cam.dma_half_buffer_size),
                           &lcd_cam_obj->cam.dma_buffer[(cnt % 2) * lcd_cam_obj->cam.dma_half_buffer_size],
                           lcd_cam_obj->cam.dma_half_buffer_size);
                // }
                cnt++;

            } else if (cam_event == CAM_VSYNC_EVENT) {

                fb_event->len = cnt * lcd_cam_obj->cam.dma_half_buffer_size;
                frame_buffer->is_full = 1;

                if (lcd_cam_obj->cam.jpeg_mode) {
                    //Copy the last buffer, the last DMA may not be completed, but the JPEG end mark should be in the buffer.
                    cam_memcpy(fb_event->buf + (cnt * lcd_cam_obj->cam.dma_half_buffer_size),
                               &lcd_cam_obj->cam.dma_buffer[(cnt % 2) * lcd_cam_obj->cam.dma_half_buffer_size],
                               lcd_cam_obj->cam.dma_half_buffer_size);
                    cnt++;
                    fb_event->len = cnt * lcd_cam_obj->cam.dma_half_buffer_size;

                    // find the start marker for JPEG.
                    int offset_s = search_soi(fb_event->buf, fb_event->len);
                    if (offset_s != 0) {
                        // The JPEG start marker is not at the head, it may be damaged.
                        printf("soi_offset=%d/%d\n", offset_s, fb_event->len);
                        frame_buffer->is_full = 0;
                    } else {
                        // find the end marker for JPEG. Data after that can be discarded
                        int offset_e = search_eoi(fb_event->buf, fb_event->len);
                        if (offset_e >= 0) {
                            fb_event->len = offset_e + sizeof(JPEG_EOI_MARKER);
                        } else {
                            printf("no eoi\n");
                            frame_buffer->is_full = 0;
                        }
                    }
                } else { // not jpeg output
                    if (cnt != lcd_cam_obj->cam.frame_copy_cnt) {
                        printf("c1=%d,c2=%d\n", cnt, lcd_cam_obj->cam.frame_copy_cnt);
                    }
                }

                uint64_t us = (uint64_t)esp_timer_get_time();
                fb_event->timestamp.tv_sec = us / 1000000UL;
                fb_event->timestamp.tv_usec = us % 1000000UL;
                // io_probe(9, 4);
                cam_dma_stop();
                if (frame_buffer->is_full) {
                    if (xQueueSend(lcd_cam_obj->cam.frame_buffer_queue, (void *)&fb_event, 0) != pdTRUE) {
                        frame_buffer->is_full = 0;
                        printf("fb send error\n");
                    }
                }
                int fb_index = find_empty_fb();
                if (fb_index < 0) {
                    printf("go idle\n");
                    state = CAM_STATE_IDLE;
                } else {
                    frame_pos = fb_index;
                    cam_dma_start();
                    // io_probe(9, 3);
                    cnt = 0;
                }
            }
        } break;
        default: break;
        }
    }
}


esp_err_t cam_init(const camera_config_t *config)
{
    CAM_CHECK(NULL != config, "config pointer is invalid", ESP_ERR_INVALID_ARG);
    // CAM_CHECK(config->width <= CAM_DATA_MAX_WIDTH && config->width % 8 == 0, "camear data width invaild", ESP_ERR_INVALID_ARG);

    esp_err_t ret = ESP_OK;
    lcd_cam_obj = (lcd_cam_obj_t *)heap_caps_calloc(1, sizeof(lcd_cam_obj_t), MALLOC_CAP_DMA);
    CAM_CHECK(NULL != lcd_cam_obj, "lcd_cam object malloc error", ESP_ERR_NO_MEM);

    for (int x = 0; x < LCD_CAM_DMA_MAX_NUM; x++) {
        if (GDMA.out[x].link.addr == 0x0 && GDMA.in[x].link.addr == 0x0) {
            lcd_cam_obj->dma_num = x;
            ESP_LOGI(TAG, "dma_num=%d", lcd_cam_obj->dma_num);
            break;
        }
        if (x == LCD_CAM_DMA_MAX_NUM - 1) {
            cam_deinit();
            CAM_CHECK(false, "DMA error", ESP_FAIL);
        }
    }

    lcd_cam_obj->cfg = *config;
    lcd_cam_obj->cam.jpeg_mode = PIXFORMAT_JPEG == config->pixel_format ? 1 : 0;
    lcd_cam_obj->cam.vsync_pin = config->pin_vsync;
    lcd_cam_obj->cam.vsync_invert = true;
    lcd_cam_obj->cam.frame_cnt = config->fb_count;
    if(lcd_cam_obj->cam.jpeg_mode){
        lcd_cam_obj->cam.recv_size = resolution[config->frame_size].width * resolution[config->frame_size].height / 5;
    } else {
        lcd_cam_obj->cam.recv_size = resolution[config->frame_size].width * resolution[config->frame_size].height * 2;
    }
    lcd_cam_obj->cam.swap_data = !(lcd_cam_obj->cam.jpeg_mode);

    lcd_cam_config(config);
    cam_set_pin(config);
    ret = cam_frame_buffer_init();
    CAM_CHECK_GOTO(ret == ESP_OK, "frame_buffer initialize failed", err);

    ret = cam_dma_config(lcd_cam_obj->cam.jpeg_mode, 10 * 1024);
    CAM_CHECK_GOTO(ret == ESP_OK, "lcd_cam config failed", err);

    lcd_cam_obj->cam.event_queue = xQueueCreate(2, sizeof(cam_event_t));
    CAM_CHECK_GOTO(lcd_cam_obj->cam.event_queue != NULL, "event_queue create failed", err);

    lcd_cam_obj->cam.frame_buffer_queue = xQueueCreate(lcd_cam_obj->cam.frame_cnt, sizeof(camera_fb_t *));
    CAM_CHECK_GOTO(lcd_cam_obj->cam.frame_buffer_queue != NULL, "frame_buffer_queue create failed", err);

    xTaskCreate(cam_task, "cam_task", 4 * 1024, NULL, configMAX_PRIORITIES / 2, &lcd_cam_obj->cam.task_handle);

    ret |= esp_intr_alloc((ETS_DMA_CH0_INTR_SOURCE + lcd_cam_obj->dma_num), ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, dma_isr, NULL, &lcd_cam_obj->dma_intr_handle);
    ret |= esp_intr_alloc(ETS_LCD_CAM_INTR_SOURCE, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, lcd_cam_isr, NULL, &lcd_cam_obj->lcd_cam_intr_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "lcd_cam intr alloc fail!");
        cam_deinit();
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "cam init ok");
    return ESP_OK;

err:
    cam_deinit();
    return ESP_FAIL;
}

esp_err_t cam_deinit(void)
{
    if (!lcd_cam_obj) {
        return ESP_FAIL;
    }

    cam_stop();
    if (lcd_cam_obj->cam.task_handle) {
        vTaskDelete(lcd_cam_obj->cam.task_handle);
    }
    if (lcd_cam_obj->cam.event_queue) {
        vQueueDelete(lcd_cam_obj->cam.event_queue);
    }
    if (lcd_cam_obj->cam.frame_buffer_queue) {
        vQueueDelete(lcd_cam_obj->cam.frame_buffer_queue);
    }
    if (lcd_cam_obj->cam.dma) {
        free(lcd_cam_obj->cam.dma);
    }
    if (lcd_cam_obj->cam.dma_buffer) {
        free(lcd_cam_obj->cam.dma_buffer);
    }
    cam_frame_buffer_deinit();
    if (lcd_cam_obj->lcd_cam_intr_handle) {
        esp_intr_free(lcd_cam_obj->lcd_cam_intr_handle);
    }
    if (lcd_cam_obj->dma_intr_handle) {
        esp_intr_free(lcd_cam_obj->dma_intr_handle);
    }
    free(lcd_cam_obj);
    lcd_cam_obj = NULL;
    return ESP_OK;
}

void cam_stop(void)
{
    cam_vsync_intr_enable(false);
    cam_dma_stop();
}

void cam_start(void)
{
    cam_vsync_intr_enable(true);
}

camera_fb_t *cam_take(void)
{
    camera_fb_t *frame_buffer_event;
    xQueueReceive(lcd_cam_obj->cam.frame_buffer_queue, (void *)&frame_buffer_event, portMAX_DELAY);
    return frame_buffer_event;
}

void cam_give(camera_fb_t *dma_buffer)
{
    for (int x = 0; x < lcd_cam_obj->cam.frame_cnt; x++) {
        if (&lcd_cam_obj->cam.frame_buffer[x].fb_event == dma_buffer) {
            lcd_cam_obj->cam.frame_buffer[x].is_full = 0;
            break;
        }
    }
}
