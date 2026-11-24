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
//
// ESP32-S31 low-level camera driver.
//
// This is the ESP32-S3 ll_cam.c adapted to the ESP32-S31 differences found
// in ESP-IDF (components/esp_hal_cam/esp32s31, components/esp_hal_dma/
// esp32s31 and components/esp_driver_cam/dvp):
//
//  * DMA: LCD_CAM is fed by AXI GDMA (group 1), not AHB GDMA. Channels are
//    allocated with gdma_new_axi_channel(), the peripheral selector is
//    SOC_GDMA_TRIG_PERIPH_CAM0 (0, CAM0 is on the AXI bus), descriptor
//    addresses are full 32-bit (no 0xfffff mask) and descriptors are the
//    8-byte aligned AXI type (dma_descriptor_align8_t, see ll_cam.h).
//  * Clocks/reset: LCD_CAM and the camera XCLK are no longer in LCD_CAM /
//    SYSTEM registers. They live in HP_SYS_CLKRST and are programmed
//    through the IDF hal/cam_ll.h helpers, exactly as esp_driver_cam does
//    (core clock: PLL120M / 2, XCLK clock: PLL160M / divider).
//  * Register renames: cam_ctrl.cam_update -> cam_update_reg,
//    cam_rgb_yuv.cam_conv_bypass -> cam_conv_enable, interrupt macros
//    LCD_CAM_* -> LCDCAM_* and GPIO matrix signals CAM_*_IDX ->
//    CAM_*_PAD_IN_IDX / CAM_CLK_PAD_OUT_IDX.

#include <stdio.h>
#include <string.h>
#include "soc/lcd_cam_struct.h"
#include "soc/lcd_cam_reg.h"
#include "soc/axi_dma_struct.h"
#include "soc/axi_dma_reg.h"
#include "hal/cam_ll.h"
#include "hal/gdma_channel.h"
#include "esp_clk_tree.h"
#include "esp_private/esp_clk_tree_common.h"
#include "esp_private/periph_ctrl.h"
#include "esp_private/gdma.h"
#include "ll_cam.h"
#include "cam_hal.h"
#include "esp_rom_gpio.h"

#include "driver/gpio.h"
#include "esp_private/gpio.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_periph.h"
#include "soc/io_mux_reg.h"
#define gpio_matrix_in(a,b,c) esp_rom_gpio_connect_in_signal(a,b,c)
#define gpio_matrix_out(a,b,c,d) esp_rom_gpio_connect_out_signal(a,b,c,d)
#define ets_delay_us(a) esp_rom_delay_us(a)

static const char *TAG = "s31 ll_cam";

/* CAM0 is attached to the AXI GDMA bus on ESP32-S31 (see
 * esp_hal_dma/esp32s31/include/hal/gdma_channel.h:
 * SOC_GDMA_TRIG_PERIPH_CAM0_BUS == SOC_GDMA_BUS_AXI). */
#define CAM_DMA_PERI_SEL    SOC_GDMA_TRIG_PERIPH_CAM0

void ll_cam_dma_print_state(cam_obj_t *cam)
{
    esp_rom_printf("axi_dma_in_status[%u]  :\n", cam->dma_num);
    esp_rom_printf("  infifo_status        : 0x%lx\n", AXI_DMA.in[cam->dma_num].conf.infifo_status.val);
    esp_rom_printf("  in_buf_hungry        : %lu\n", AXI_DMA.in[cam->dma_num].conf.infifo_status.in_buf_hungry_chn);
    esp_rom_printf("axi_dma_state[%u]      :\n", cam->dma_num);
    esp_rom_printf("  inlink_dscr_addr     : 0x%lx\n", AXI_DMA.in[cam->dma_num].conf.in_state.inlink_dscr_addr_chn);
    esp_rom_printf("  in_dscr_state        : %lu\n", AXI_DMA.in[cam->dma_num].conf.in_state.in_dscr_state_chn);
    esp_rom_printf("  in_state             : %lu\n", AXI_DMA.in[cam->dma_num].conf.in_state.in_state_chn);
}

void ll_cam_dma_reset(cam_obj_t *cam)
{
    AXI_DMA.in[cam->dma_num].intr.clr.val = ~0;
    AXI_DMA.in[cam->dma_num].intr.ena.val = 0;

    AXI_DMA.in[cam->dma_num].conf.in_conf0.val = 0;
    AXI_DMA.in[cam->dma_num].conf.in_conf0.in_rst_chn = 1;
    AXI_DMA.in[cam->dma_num].conf.in_conf0.in_rst_chn = 0;

    //internal SRAM only
    if (!cam->psram_mode) {
        AXI_DMA.in[cam->dma_num].conf.in_conf0.indscr_burst_en_chn = 1;
    }
    // AXI GDMA always bursts data; select 32 byte bursts (the GDMA default)
    AXI_DMA.in[cam->dma_num].conf.in_conf0.in_burst_size_sel_chn = 2;

    AXI_DMA.in[cam->dma_num].conf.in_conf1.in_check_owner_chn = 0;

    AXI_DMA.in[cam->dma_num].conf.in_peri_sel.peri_in_sel_chn = CAM_DMA_PERI_SEL;
    //AXI_DMA.in[cam->dma_num].conf.in_pri.rx_pri_chn = 1;//rx prio 0-15
}

static void CAMERA_ISR_IRAM_ATTR ll_cam_vsync_isr(void *arg)
{
    cam_obj_t *cam = (cam_obj_t *)arg;
    BaseType_t HPTaskAwoken = pdFALSE;

    typeof(LCD_CAM.lc_dma_int_st) status = LCD_CAM.lc_dma_int_st;
    if (status.val == 0) {
        return;
    }

    LCD_CAM.lc_dma_int_clr.val = status.val;

    if (status.cam_vsync_int_st) {
        ll_cam_send_event(cam, CAM_VSYNC_EVENT, &HPTaskAwoken);
    }

    if (HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void CAMERA_ISR_IRAM_ATTR ll_cam_dma_isr(void *arg)
{
    cam_obj_t *cam = (cam_obj_t *)arg;
    BaseType_t HPTaskAwoken = pdFALSE;

    typeof(AXI_DMA.in[cam->dma_num].intr.st) status = AXI_DMA.in[cam->dma_num].intr.st;
    if (status.val == 0) {
        return;
    }

    AXI_DMA.in[cam->dma_num].intr.clr.val = status.val;

    if (status.in_suc_eof_chn_int_st) {
        ll_cam_send_event(cam, CAM_IN_SUC_EOF_EVENT, &HPTaskAwoken);
    }

    if (HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

bool IRAM_ATTR ll_cam_stop(cam_obj_t *cam)
{
    if (cam->jpeg_mode || !cam->psram_mode) {
        AXI_DMA.in[cam->dma_num].intr.ena.in_suc_eof_chn_int_ena = 0;
        AXI_DMA.in[cam->dma_num].intr.clr.in_suc_eof_chn_int_clr = 1;
    }
    AXI_DMA.in[cam->dma_num].conf.in_link1.inlink_stop_chn = 1;
    return true;
}

bool ll_cam_start(cam_obj_t *cam, int frame_pos)
{
    LCD_CAM.cam_ctrl1.cam_start = 0;

    if (cam->jpeg_mode || !cam->psram_mode) {
        AXI_DMA.in[cam->dma_num].intr.clr.in_suc_eof_chn_int_clr = 1;
        AXI_DMA.in[cam->dma_num].intr.ena.in_suc_eof_chn_int_ena = 1;
    }

    LCD_CAM.cam_ctrl1.cam_reset = 1;
    LCD_CAM.cam_ctrl1.cam_reset = 0;
    LCD_CAM.cam_ctrl1.cam_afifo_reset = 1;
    LCD_CAM.cam_ctrl1.cam_afifo_reset = 0;
    AXI_DMA.in[cam->dma_num].conf.in_conf0.in_rst_chn = 1;
    AXI_DMA.in[cam->dma_num].conf.in_conf0.in_rst_chn = 0;

    LCD_CAM.cam_ctrl1.cam_rec_data_bytelen = cam->dma_half_buffer_size - 1; // Ping pong operation

    if (!cam->psram_mode) {
        AXI_DMA.in[cam->dma_num].conf.in_link2.inlink_addr_chn = (uint32_t)&cam->dma[0];
    } else {
        AXI_DMA.in[cam->dma_num].conf.in_link2.inlink_addr_chn = (uint32_t)&cam->frames[frame_pos].dma[0];
    }

    AXI_DMA.in[cam->dma_num].conf.in_link1.inlink_start_chn = 1;

    LCD_CAM.cam_ctrl.cam_update_reg = 1;
    LCD_CAM.cam_ctrl1.cam_start = 1;
    return true;
}

esp_err_t ll_cam_deinit(cam_obj_t *cam)
{
    if (cam->cam_intr_handle) {
        esp_intr_free(cam->cam_intr_handle);
        cam->cam_intr_handle = NULL;
    }

    if (cam->dma_intr_handle) {
        esp_intr_free(cam->dma_intr_handle);
        cam->dma_intr_handle = NULL;
    }
    if (cam->dma_channel_handle) {
        // gdma_disconnect(cam->dma_channel_handle); // not needed since code never calls gdma_connect() to connect channel to peripheral
        gdma_del_channel(cam->dma_channel_handle);
        cam->dma_channel_handle = NULL;
    }
    LCD_CAM.cam_ctrl1.cam_start = 0;
    LCD_CAM.cam_ctrl1.cam_reset = 1;
    LCD_CAM.cam_ctrl1.cam_reset = 0;

    PERIPH_RCC_ATOMIC() {
        cam_ll_enable_clk(0, false);
    }
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6,2,0)
    esp_clk_tree_enable_src((soc_module_clk_t)CAM_CLK_SRC_DEFAULT, false);
    esp_clk_tree_enable_src((soc_module_clk_t)CAM_CORE_CLK_SRC_DEFAULT, false);
#else
    esp_clk_tree_release_src((soc_module_clk_t)CAM_CLK_SRC_DEFAULT);
    esp_clk_tree_release_src((soc_module_clk_t)CAM_CORE_CLK_SRC_DEFAULT);
#endif
    return ESP_OK;
}

static esp_err_t ll_cam_dma_init(cam_obj_t *cam)
{
    //alloc rx axi gdma channel (LCD_CAM is on the AXI bus on ESP32-S31,
    //this is gdma_new_axi_channel() in IDF's esp_cam_ctlr_dvp_gdma.c)
    gdma_channel_alloc_config_t rx_alloc_config = {0};
    esp_err_t ret = gdma_new_axi_channel(&rx_alloc_config, NULL, &cam->dma_channel_handle);
    if (ret != ESP_OK) {
        cam_deinit();
        ESP_LOGE(TAG, "Can't find available AXI GDMA channel");
        return ESP_FAIL;
    }
    int chan_id = -1;
    ret = gdma_get_channel_id(cam->dma_channel_handle, &chan_id);
    if (ret != ESP_OK) {
        cam_deinit();
        ESP_LOGE(TAG, "Can't get AXI GDMA channel number");
        return ESP_FAIL;
    }
    cam->dma_num = chan_id;
    ESP_LOGI(TAG, "AXI DMA Channel=%d", cam->dma_num);

    // The GDMA driver enables the AXI PDMA bus clock when the channel is
    // allocated, so there is no SYSTEM/DMA register poking here (unlike S3).
    ll_cam_dma_reset(cam);
    return ESP_OK;
}

#if CONFIG_CAMERA_CONVERTER_ENABLED
static esp_err_t ll_cam_converter_config(cam_obj_t *cam, const camera_config_t *config)
{
    esp_err_t ret = ESP_OK;

    switch (config->conv_mode) {
    case YUV422_TO_YUV420:
        if (config->pixel_format != PIXFORMAT_YUV422) {
            ret = ESP_FAIL;
        } else {
            ESP_LOGI(TAG, "YUV422 to YUV420 mode");
            LCD_CAM.cam_rgb_yuv.cam_conv_yuv2yuv_mode = 1;
            LCD_CAM.cam_rgb_yuv.cam_conv_yuv_mode = 0;
            LCD_CAM.cam_rgb_yuv.cam_conv_trans_mode = 1;
        }
        break;
    case YUV422_TO_RGB565:
        if (config->pixel_format != PIXFORMAT_YUV422) {
            ret = ESP_FAIL;
        } else {
            ESP_LOGI(TAG, "YUV422 to RGB565 mode");
            LCD_CAM.cam_rgb_yuv.cam_conv_yuv2yuv_mode = 3;
            LCD_CAM.cam_rgb_yuv.cam_conv_yuv_mode = 0;
            LCD_CAM.cam_rgb_yuv.cam_conv_trans_mode = 0;
        }
        break;
    default:
        break;
    }
#if CONFIG_LCD_CAM_CONV_BT709_ENABLED
    LCD_CAM.cam_rgb_yuv.cam_conv_protocol_mode = 1;
#else
    LCD_CAM.cam_rgb_yuv.cam_conv_protocol_mode = 0;
#endif
#if CONFIG_LCD_CAM_CONV_FULL_RANGE_ENABLED
    LCD_CAM.cam_rgb_yuv.cam_conv_data_out_mode = 1;
    LCD_CAM.cam_rgb_yuv.cam_conv_data_in_mode = 1;
#else
    LCD_CAM.cam_rgb_yuv.cam_conv_data_out_mode = 0;
    LCD_CAM.cam_rgb_yuv.cam_conv_data_in_mode = 0;
#endif
    LCD_CAM.cam_rgb_yuv.cam_conv_mode_8bits_on = 1;
    LCD_CAM.cam_rgb_yuv.cam_conv_enable = 1;
    cam->conv_mode = config->conv_mode;
    return ret;
}
#endif

esp_err_t ll_cam_config(cam_obj_t *cam, const camera_config_t *config)
{
    esp_err_t ret = ESP_OK;

    // LCD_CAM bus clock, reset and core clock (PLL120M / 2), following
    // esp_cam_ctlr_dvp_init() in ESP-IDF. On ESP32-S31 these are in
    // HP_SYS_CLKRST, not in SYSTEM/LCD_CAM registers.
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6,2,0)
    ESP_ERROR_CHECK(esp_clk_tree_enable_src((soc_module_clk_t)CAM_CORE_CLK_SRC_DEFAULT, true));
#else
    ESP_ERROR_CHECK(esp_clk_tree_acquire_src((soc_module_clk_t)CAM_CORE_CLK_SRC_DEFAULT, NULL));
#endif
    PERIPH_RCC_ACQUIRE_ATOMIC(PERIPH_LCD_CAM_MODULE, ref_count) {
        if (ref_count == 0) {
            cam_ll_enable_bus_clock(0, true);
            cam_ll_reset_register(0);
            cam_ll_select_core_clk_src(0, CAM_CORE_CLK_SRC_DEFAULT);
            cam_ll_set_core_clock_divider(0, 2, 0, 0);
        }
    }

    // Camera (XCLK) clock: PLL160M divided down to xclk_freq_hz
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6,2,0)
    ESP_ERROR_CHECK(esp_clk_tree_enable_src((soc_module_clk_t)CAM_CLK_SRC_DEFAULT, true));
#else
    ESP_ERROR_CHECK(esp_clk_tree_acquire_src((soc_module_clk_t)CAM_CLK_SRC_DEFAULT, NULL));
#endif
    PERIPH_RCC_ATOMIC() {
        cam_ll_enable_clk(0, true);
        cam_ll_select_clk_src(0, CAM_CLK_SRC_DEFAULT);
        cam_ll_set_group_clock_coeff(0, 160000000 / config->xclk_freq_hz, 0, 0);
    }

    LCD_CAM.cam_ctrl.val = 0;

    LCD_CAM.cam_ctrl.cam_stop_en = 0;
    LCD_CAM.cam_ctrl.cam_vsync_filter_thres = 4; // Filter by LCD_CAM clock
    LCD_CAM.cam_ctrl.cam_update_reg = 0;
    LCD_CAM.cam_ctrl.cam_byte_order = cam->swap_data;
    LCD_CAM.cam_ctrl.cam_bit_order = 0;
    LCD_CAM.cam_ctrl.cam_line_int_en = 0;
    LCD_CAM.cam_ctrl.cam_vs_eof_en = 0; //1: CAM_VSYNC to generate in_suc_eof. 0: in_suc_eof is controlled by reg_cam_rec_data_cyclelen

    LCD_CAM.cam_ctrl1.val = 0;
    LCD_CAM.cam_ctrl1.cam_rec_data_bytelen = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE - 1; // Cannot be assigned to 0, and it is easy to overflow
    LCD_CAM.cam_ctrl1.cam_line_int_num = 0; // The number of hsyncs that generate hs interrupts
    LCD_CAM.cam_ctrl1.cam_clk_inv = 0;
    LCD_CAM.cam_ctrl1.cam_vsync_filter_en = 1;
    LCD_CAM.cam_ctrl1.cam_2byte_en = 0;
    LCD_CAM.cam_ctrl1.cam_de_inv = 0;
    LCD_CAM.cam_ctrl1.cam_hsync_inv = 0;
    LCD_CAM.cam_ctrl1.cam_vsync_inv = 0;
    LCD_CAM.cam_ctrl1.cam_vh_de_mode_en = 0;

    LCD_CAM.cam_rgb_yuv.val = 0;

#if CONFIG_CAMERA_CONVERTER_ENABLED
    if (config->conv_mode) {
        ret = ll_cam_converter_config(cam, config);
        if(ret != ESP_OK) {
            return ret;
        }
    }
#endif

    LCD_CAM.cam_ctrl.cam_update_reg = 1;
    LCD_CAM.cam_ctrl1.cam_start = 1;

    ret = ll_cam_dma_init(cam);

    return ret;
}

void ll_cam_vsync_intr_enable(cam_obj_t *cam, bool en)
{
    LCD_CAM.lc_dma_int_clr.cam_vsync_int_clr = 1;
    if (en) {
        LCD_CAM.lc_dma_int_ena.cam_vsync_int_ena = 1;
    } else {
        LCD_CAM.lc_dma_int_ena.cam_vsync_int_ena = 0;
    }
}

esp_err_t ll_cam_set_pin(cam_obj_t *cam, const camera_config_t *config)
{
    gpio_func_sel(config->pin_pclk, PIN_FUNC_GPIO);
    gpio_set_direction(config->pin_pclk, GPIO_MODE_INPUT);
    gpio_set_pull_mode(config->pin_pclk, GPIO_FLOATING);
    gpio_matrix_in(config->pin_pclk, CAM_PCLK_PAD_IN_IDX, false);

    gpio_func_sel(config->pin_vsync, PIN_FUNC_GPIO);
    gpio_set_direction(config->pin_vsync, GPIO_MODE_INPUT);
    gpio_set_pull_mode(config->pin_vsync, GPIO_FLOATING);
    gpio_matrix_in(config->pin_vsync, CAM_V_SYNC_PAD_IN_IDX, cam->vsync_invert);

    gpio_func_sel(config->pin_href, PIN_FUNC_GPIO);
    gpio_set_direction(config->pin_href, GPIO_MODE_INPUT);
    gpio_set_pull_mode(config->pin_href, GPIO_FLOATING);
    gpio_matrix_in(config->pin_href, CAM_H_ENABLE_PAD_IN_IDX, false);

    int data_pins[8] = {
        config->pin_d0, config->pin_d1, config->pin_d2, config->pin_d3, config->pin_d4, config->pin_d5, config->pin_d6, config->pin_d7,
    };
    for (int i = 0; i < 8; i++) {
        gpio_func_sel(data_pins[i], PIN_FUNC_GPIO);
        gpio_set_direction(data_pins[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(data_pins[i], GPIO_FLOATING);
        gpio_matrix_in(data_pins[i], CAM_DATA0_IN_PAD_IN_IDX + i, false);
    }
    if (config->pin_xclk >= 0) {
        gpio_func_sel(config->pin_xclk, PIN_FUNC_GPIO);
        gpio_set_direction(config->pin_xclk, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(config->pin_xclk, GPIO_FLOATING);
        gpio_matrix_out(config->pin_xclk, CAM_CLK_PAD_OUT_IDX, false, false);
    }

    return ESP_OK;
}

esp_err_t ll_cam_init_isr(cam_obj_t *cam)
{
    esp_err_t ret = ESP_OK;
    ret = esp_intr_alloc_intrstatus(ETS_AXI_PDMA_IN_CH0_INTR_SOURCE + cam->dma_num,
                                     ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_SHARED | CAMERA_ISR_IRAM_FLAG,
                                     (uint32_t)&AXI_DMA.in[cam->dma_num].intr.st, AXI_DMA_IN_SUC_EOF_CH0_INT_ST_M,
                                     ll_cam_dma_isr, cam, &cam->dma_intr_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AXI DMA interrupt allocation of camera failed");
        return ret;
    }

    ret = esp_intr_alloc_intrstatus(ETS_LCD_CAM_INTR_SOURCE,
                                     ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_SHARED | CAMERA_ISR_IRAM_FLAG,
                                     (uint32_t)&LCD_CAM.lc_dma_int_st.val, LCDCAM_CAM_VSYNC_INT_ST_M,
                                     ll_cam_vsync_isr, cam, &cam->cam_intr_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD_CAM interrupt allocation of camera failed");
        return ret;
    }
    return ESP_OK;
}

void ll_cam_do_vsync(cam_obj_t *cam)
{
    gpio_matrix_in(cam->vsync_pin, CAM_V_SYNC_PAD_IN_IDX, !cam->vsync_invert);
    ets_delay_us(10);
    gpio_matrix_in(cam->vsync_pin, CAM_V_SYNC_PAD_IN_IDX, cam->vsync_invert);
}

uint8_t ll_cam_get_dma_align(cam_obj_t *cam)
{
    gdma_channel_alignment_info_t align_info = {0};
    if (cam->dma_channel_handle &&
        gdma_get_channel_alignment_constraints(cam->dma_channel_handle, &align_info) == ESP_OK &&
        align_info.ext_enc_mem_alignment > 1) {
        return align_info.ext_enc_mem_alignment;
    }
    return 64;
}

static bool ll_cam_calc_rgb_dma(cam_obj_t *cam){
    size_t node_max = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE / cam->dma_bytes_per_item;
    size_t line_width = cam->width * cam->in_bytes_per_pixel;
    size_t node_size = node_max;
    size_t nodes_per_line = 1;
    size_t lines_per_node = 1;

    // Calculate DMA Node Size so that it's divisible by or divisor of the line width
    if(line_width >= node_max){
        // One or more nodes will be required for one line
        for(size_t i = node_max; i > 0; i=i-1){
            if ((line_width % i) == 0) {
                node_size = i;
                nodes_per_line = line_width / node_size;
                break;
            }
        }
    } else {
        // One or more lines can fit into one node
        for(size_t i = node_max; i > 0; i=i-1){
            if ((i % line_width) == 0) {
                node_size = i;
                lines_per_node = node_size / line_width;
                while((cam->height % lines_per_node) != 0){
                    lines_per_node = lines_per_node - 1;
                    node_size = lines_per_node * line_width;
                }
                break;
            }
        }
    }

    ESP_LOGI(TAG, "node_size: %4u, nodes_per_line: %u, lines_per_node: %u",
            (unsigned) (node_size * cam->dma_bytes_per_item), (unsigned) nodes_per_line, (unsigned) lines_per_node);

    cam->dma_node_buffer_size = node_size * cam->dma_bytes_per_item;

    size_t dma_half_buffer_max = CONFIG_CAMERA_DMA_BUFFER_SIZE_MAX / 2 / cam->dma_bytes_per_item;
    if (line_width > dma_half_buffer_max) {
        ESP_LOGE(TAG, "Resolution too high");
        return 0;
    }

    // Calculate minimum EOF size = max(mode_size, line_size)
    size_t dma_half_buffer_min = node_size * nodes_per_line;

    // Calculate max EOF size divisible by node size
    size_t dma_half_buffer = (dma_half_buffer_max / dma_half_buffer_min) * dma_half_buffer_min;

    // Adjust EOF size so that height will be divisible by the number of lines in each EOF
    size_t lines_per_half_buffer = dma_half_buffer / line_width;
    while((cam->height % lines_per_half_buffer) != 0){
        dma_half_buffer = dma_half_buffer - dma_half_buffer_min;
        lines_per_half_buffer = dma_half_buffer / line_width;
    }

    // Calculate DMA size
    size_t dma_buffer_max = 2 * dma_half_buffer_max;
    if (cam->psram_mode) {
        dma_buffer_max = cam->recv_size / cam->dma_bytes_per_item;
    }
    size_t dma_buffer_size = dma_buffer_max;
    if (!cam->psram_mode) {
        dma_buffer_size =(dma_buffer_max / dma_half_buffer) * dma_half_buffer;
    }

    ESP_LOGI(TAG, "dma_half_buffer_min: %5u, dma_half_buffer: %5u, lines_per_half_buffer: %2u, dma_buffer_size: %5u",
            (unsigned) (dma_half_buffer_min * cam->dma_bytes_per_item), (unsigned) (dma_half_buffer * cam->dma_bytes_per_item),
            (unsigned) lines_per_half_buffer, (unsigned) (dma_buffer_size * cam->dma_bytes_per_item));

    cam->dma_buffer_size = dma_buffer_size * cam->dma_bytes_per_item;
    cam->dma_half_buffer_size = dma_half_buffer * cam->dma_bytes_per_item;
    cam->dma_half_buffer_cnt = cam->dma_buffer_size / cam->dma_half_buffer_size;
    return 1;
}

bool ll_cam_dma_sizes(cam_obj_t *cam)
{
    cam->dma_bytes_per_item = 1;
    if (cam->jpeg_mode) {
        if (cam->psram_mode) {
            cam->dma_buffer_size = cam->recv_size;
            cam->dma_half_buffer_size = 1024;
            cam->dma_half_buffer_cnt = cam->dma_buffer_size / cam->dma_half_buffer_size;
            cam->dma_node_buffer_size = cam->dma_half_buffer_size;
        } else {
            cam->dma_half_buffer_cnt = 16;
            cam->dma_buffer_size = cam->dma_half_buffer_cnt * 1024;
            cam->dma_half_buffer_size = cam->dma_buffer_size / cam->dma_half_buffer_cnt;
            cam->dma_node_buffer_size = cam->dma_half_buffer_size;
        }
    } else {
        return ll_cam_calc_rgb_dma(cam);
    }
    return 1;
}

size_t IRAM_ATTR ll_cam_memcpy(cam_obj_t *cam, uint8_t *out, const uint8_t *in, size_t len)
{
    // YUV to Grayscale
    if (cam->in_bytes_per_pixel == 2 && cam->fb_bytes_per_pixel == 1) {
        size_t end = len / 8;
        for (size_t i = 0; i < end; ++i) {
            out[0] = in[0];
            out[1] = in[2];
            out[2] = in[4];
            out[3] = in[6];
            out += 4;
            in += 8;
        }
        return len / 2;
    }

    // just memcpy
    memcpy(out, in, len);
    return len;
}

esp_err_t ll_cam_set_sample_mode(cam_obj_t *cam, pixformat_t pix_format, uint32_t xclk_freq_hz, uint16_t sensor_pid)
{
    if (pix_format == PIXFORMAT_GRAYSCALE) {
        if (sensor_pid == OV3660_PID || sensor_pid == OV3640_PID || sensor_pid == OV5640_PID || sensor_pid == NT99141_PID || sensor_pid == SC031GS_PID || sensor_pid == BF20A6_PID || sensor_pid == GC0308_PID || sensor_pid == HM0360_PID) {
            cam->in_bytes_per_pixel = 1;       // camera sends Y8
        } else {
            cam->in_bytes_per_pixel = 2;       // camera sends YU/YV
        }
        cam->fb_bytes_per_pixel = 1;       // frame buffer stores Y8
    } else if (pix_format == PIXFORMAT_YUV422 || pix_format == PIXFORMAT_RGB565) {
#if CONFIG_CAMERA_CONVERTER_ENABLED
        switch (cam->conv_mode) {
        case YUV422_TO_YUV420:
            cam->in_bytes_per_pixel = 1.5;       // for DMA receive
            cam->fb_bytes_per_pixel = 1.5;       // frame buffer stores YUV420
            break;
        case YUV422_TO_RGB565:
        default:
            cam->in_bytes_per_pixel = 2;       // for DMA receive
            cam->fb_bytes_per_pixel = 2;       // frame buffer stores YU/YV/RGB565
            break;
        }
#else
        cam->in_bytes_per_pixel = 2;       // for DMA receive
        cam->fb_bytes_per_pixel = 2;       // frame buffer stores YU/YV/RGB565
#endif
    } else if (pix_format == PIXFORMAT_JPEG) {
        cam->in_bytes_per_pixel = 1;
        cam->fb_bytes_per_pixel = 1;
    } else {
        ESP_LOGE(TAG, "Requested format is not supported");
        return ESP_ERR_NOT_SUPPORTED;
    }
    return ESP_OK;
}

// implements function from xclk.c to allow dynamic XCLK change
esp_err_t xclk_timer_conf(int ledc_timer, int xclk_freq_hz)
{
    PERIPH_RCC_ATOMIC() {
        cam_ll_set_group_clock_coeff(0, 160000000 / xclk_freq_hz, 0, 0);
    }
    LCD_CAM.cam_ctrl.cam_update_reg = 1;
    return ESP_OK;
}
