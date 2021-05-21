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
#include "soc/system_reg.h"
#include "soc/lcd_cam_struct.h"
#include "soc/gdma_struct.h"
#include "ll_cam.h"
#include "cam_hal.h"

static const char *TAG = "s3 ll_cam";

static void IRAM_ATTR ll_cam_vsync_isr(void *arg)
{
    //DBG_PIN_SET(1);
    cam_obj_t *cam = (cam_obj_t *)arg;
    BaseType_t HPTaskAwoken = pdFALSE;

    typeof(LCD_CAM.lc_dma_int_st) status = LCD_CAM.lc_dma_int_st;
    if (status.val == 0) {
        return;
    }

    LCD_CAM.lc_dma_int_clr.val = status.val;

    if (status.cam_vsync) {
        ll_cam_send_event(cam, CAM_VSYNC_EVENT, &HPTaskAwoken);
    }

    if (HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
    //DBG_PIN_SET(0);
}

static void IRAM_ATTR ll_cam_dma_isr(void *arg)
{
    cam_obj_t *cam = (cam_obj_t *)arg;
    BaseType_t HPTaskAwoken = pdFALSE;

    typeof(GDMA.in[cam->dma_num].int_st) status = GDMA.in[cam->dma_num].int_st;
    if (status.val == 0) {
        return;
    }

    GDMA.in[cam->dma_num].int_clr.val = status.val;

    if (status.in_suc_eof) {
        ll_cam_send_event(cam, CAM_IN_SUC_EOF_EVENT, &HPTaskAwoken);
    }

    if (HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

bool ll_cam_stop(cam_obj_t *cam)
{
    if (cam->jpeg_mode || !cam->psram_mode) {
        GDMA.in[cam->dma_num].int_ena.in_suc_eof = 0;
        GDMA.in[cam->dma_num].int_clr.in_suc_eof = 1;
    }
    GDMA.in[cam->dma_num].link.stop = 1;
    return true;
}

bool ll_cam_start(cam_obj_t *cam, int frame_pos)
{
    LCD_CAM.cam_ctrl1.cam_start = 0;

    if (cam->jpeg_mode || !cam->psram_mode) {
        GDMA.in[cam->dma_num].int_clr.in_suc_eof = 1;
        GDMA.in[cam->dma_num].int_ena.in_suc_eof = 1;
    }

    LCD_CAM.cam_ctrl1.cam_reset = 1;
    LCD_CAM.cam_ctrl1.cam_reset = 0;
    LCD_CAM.cam_ctrl1.cam_afifo_reset = 1;
    LCD_CAM.cam_ctrl1.cam_afifo_reset = 0;
    GDMA.in[cam->dma_num].conf0.in_rst = 1;
    GDMA.in[cam->dma_num].conf0.in_rst = 0;

    LCD_CAM.cam_ctrl1.cam_rec_data_bytelen = cam->dma_half_buffer_size - 1; // Ping pong operation

    if (!cam->psram_mode) {
        GDMA.in[cam->dma_num].link.addr = ((uint32_t)&cam->dma[0]) & 0xfffff;
    } else {
        GDMA.in[cam->dma_num].link.addr = ((uint32_t)&cam->frames[frame_pos].dma[0]) & 0xfffff;
    }

    GDMA.in[cam->dma_num].link.start = 1;

    LCD_CAM.cam_ctrl.cam_update = 1;
    LCD_CAM.cam_ctrl1.cam_start = 1;
    return true;
}

static esp_err_t ll_cam_dma_init(cam_obj_t *cam)
{
	#define LCD_CAM_DMA_MAX_NUM               (5) // Maximum number of DMA channels
    for (int x = (LCD_CAM_DMA_MAX_NUM - 1); x >= 0; x--) {
        if (GDMA.out[x].link.addr == 0x0 && GDMA.in[x].link.addr == 0x0) {
            cam->dma_num = x;
            ESP_LOGI(TAG, "dma_num=%d", cam->dma_num);
            break;
        }
        if (x == 0) {
            cam_deinit();
            ESP_LOGE(TAG, "DMA error");
			return ESP_FAIL;
        }
    }

    if (REG_GET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN) == 0) {
        REG_CLR_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_DMA_RST);
        REG_CLR_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_DMA_RST);
    }

    GDMA.in[cam->dma_num].int_clr.val = ~0;
    GDMA.in[cam->dma_num].int_ena.val = 0;

    GDMA.in[cam->dma_num].conf0.val = 0;
    GDMA.in[cam->dma_num].conf0.in_rst = 1;
    GDMA.in[cam->dma_num].conf0.in_rst = 0;

    //internal SRAM only
    if (!cam->psram_mode) {
        GDMA.in[cam->dma_num].conf0.indscr_burst_en = 1;
        GDMA.in[cam->dma_num].conf0.in_data_burst_en = 1;
    }

    GDMA.in[cam->dma_num].conf1.in_check_owner = 0;

    GDMA.in[cam->dma_num].peri_sel.sel = 5;
    //GDMA.in[cam->dma_num].pri.rx_pri = 1;//rx prio 0-15
    //GDMA.in[cam->dma_num].sram_size.in_size = 6;//This register is used to configure the size of L2 Tx FIFO for Rx channel. 0:16 bytes, 1:24 bytes, 2:32 bytes, 3: 40 bytes, 4: 48 bytes, 5:56 bytes, 6: 64 bytes, 7: 72 bytes, 8: 80 bytes.
    //GDMA.in[cam->dma_num].wight.rx_weight = 7;//The weight of Rx channel 0-15
    return ESP_OK;
}

esp_err_t ll_cam_config(cam_obj_t *cam, const camera_config_t *config)
{
    if (REG_GET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_LCD_CAM_CLK_EN) == 0) {
        REG_CLR_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_LCD_CAM_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_LCD_CAM_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_LCD_CAM_RST);
        REG_CLR_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_LCD_CAM_RST);
    }

    LCD_CAM.cam_ctrl.val = 0;
    
    LCD_CAM.cam_ctrl.cam_clkm_div_b = 0;
    LCD_CAM.cam_ctrl.cam_clkm_div_a = 0;
    LCD_CAM.cam_ctrl.cam_clkm_div_num = 160000000 / config->xclk_freq_hz;
    LCD_CAM.cam_ctrl.cam_clk_sel = 3;//Select Camera module source clock. 0: no clock. 1: APLL. 2: CLK160. 3: no clock.

    LCD_CAM.cam_ctrl.cam_stop_en = 0;
    LCD_CAM.cam_ctrl.cam_vsync_filter_thres = 4; // Filter by LCD_CAM clock
    LCD_CAM.cam_ctrl.cam_update = 0;
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

    LCD_CAM.cam_ctrl.cam_update = 1;
    LCD_CAM.cam_ctrl1.cam_start = 1;

    esp_err_t err = ll_cam_dma_init(cam);
    if(err != ESP_OK) {
        return err;
    }
    
    return ESP_OK;
}

void ll_cam_vsync_intr_enable(cam_obj_t *cam, bool en)
{
    LCD_CAM.lc_dma_int_clr.cam_vsync = 1;
    if (en) {
        LCD_CAM.lc_dma_int_ena.cam_vsync = 1;
    } else {
        LCD_CAM.lc_dma_int_ena.cam_vsync = 0;
    }
}

esp_err_t ll_cam_set_pin(cam_obj_t *cam, const camera_config_t *config)
{
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin_pclk], PIN_FUNC_GPIO);
    gpio_set_direction(config->pin_pclk, GPIO_MODE_INPUT);
    gpio_set_pull_mode(config->pin_pclk, GPIO_FLOATING);
    gpio_matrix_in(config->pin_pclk, CAM_PCLK_IDX, false);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin_vsync], PIN_FUNC_GPIO);
    gpio_set_direction(config->pin_vsync, GPIO_MODE_INPUT);
    gpio_set_pull_mode(config->pin_vsync, GPIO_FLOATING);
    gpio_matrix_in(config->pin_vsync, CAM_V_SYNC_IDX, cam->vsync_invert);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin_href], PIN_FUNC_GPIO);
    gpio_set_direction(config->pin_href, GPIO_MODE_INPUT);
    gpio_set_pull_mode(config->pin_href, GPIO_FLOATING);
    gpio_matrix_in(config->pin_href, CAM_H_ENABLE_IDX, false);

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

    return ESP_OK;
}

esp_err_t ll_cam_init_isr(cam_obj_t *cam)
{
	esp_err_t ret = ESP_OK;
	ret = esp_intr_alloc((ETS_DMA_IN_CH0_INTR_SOURCE + cam->dma_num), ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, ll_cam_dma_isr, cam, &cam->dma_intr_handle);
	if (ret != ESP_OK) {
		return ret;
	}
	return esp_intr_alloc(ETS_LCD_CAM_INTR_SOURCE, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, ll_cam_vsync_isr, cam, &cam->cam_intr_handle);
}

void ll_cam_do_vsync(cam_obj_t *cam)
{
    gpio_matrix_in(cam->vsync_pin, CAM_V_SYNC_IDX, !cam->vsync_invert);
    ets_delay_us(10);
    gpio_matrix_in(cam->vsync_pin, CAM_V_SYNC_IDX, cam->vsync_invert);
}

uint8_t ll_cam_get_dma_align(cam_obj_t *cam)
{
    return 16 << GDMA.in[cam->dma_num].conf1.in_ext_mem_bk_size;
}

void ll_cam_dma_sizes(cam_obj_t *cam)
{
    int cnt = 0;
    
    cam->dma_bytes_per_item = 1;
    if (cam->jpeg_mode) {
        cam->dma_half_buffer_cnt = 16;
        cam->dma_buffer_size = cam->dma_half_buffer_cnt * 1024;
        cam->dma_half_buffer_size = cam->dma_buffer_size / cam->dma_half_buffer_cnt;
        cam->dma_node_buffer_size = cam->dma_half_buffer_size;
    } else {
        int max_cam_rec_data_bytelen = 16384;
        for (cnt = 0; cnt < max_cam_rec_data_bytelen; cnt++) {
            if (cam->recv_size % (max_cam_rec_data_bytelen - cnt) == 0) {
                break;
            }
        }
        cam->dma_buffer_size = cam->recv_size;
        cam->dma_half_buffer_size = max_cam_rec_data_bytelen - cnt;
        cam->dma_half_buffer_cnt = cam->dma_buffer_size / cam->dma_half_buffer_size;

        for (cnt = 0; cnt < LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE; cnt++) { // Find a divisible dma size
            if ((cam->dma_half_buffer_size) % (LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE - cnt) == 0) {
                break;
            }
        }
        cam->dma_node_buffer_size = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE - cnt;
    }
}

size_t ll_cam_memcpy(uint8_t *out, const uint8_t *in, size_t len)
{
    memcpy(out, in, len);
    return len;
}

esp_err_t ll_cam_set_sample_mode(cam_obj_t *cam, pixformat_t pix_format, uint32_t xclk_freq_hz, uint8_t sensor_pid)
{
    if (pix_format == PIXFORMAT_GRAYSCALE) {
        if (sensor_pid == OV3660_PID || sensor_pid == OV5640_PID || sensor_pid == NT99141_PID) {
            cam->in_bytes_per_pixel = 1;       // camera sends Y8
        } else {
            cam->in_bytes_per_pixel = 2;       // camera sends YU/YV
        }
        cam->fb_bytes_per_pixel = 1;       // frame buffer stores Y8
    } else if (pix_format == PIXFORMAT_YUV422 || pix_format == PIXFORMAT_RGB565) {
            cam->in_bytes_per_pixel = 2;       // camera sends YU/YV
            cam->fb_bytes_per_pixel = 2;       // frame buffer stores YU/YV/RGB565
    } else if (pix_format == PIXFORMAT_JPEG) {
        if (sensor_pid != OV2640_PID && sensor_pid != OV3660_PID && sensor_pid != OV5640_PID  && sensor_pid != NT99141_PID) {
            ESP_LOGE(TAG, "JPEG format is not supported on this sensor");
            return ESP_ERR_NOT_SUPPORTED;
        }
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
    LCD_CAM.cam_ctrl.cam_clkm_div_b = 0;
    LCD_CAM.cam_ctrl.cam_clkm_div_a = 0;
    LCD_CAM.cam_ctrl.cam_clkm_div_num = 160000000 / xclk_freq_hz;
    LCD_CAM.cam_ctrl.cam_clk_sel = 3;//Select Camera module source clock. 0: no clock. 1: APLL. 2: CLK160. 3: no clock.
    LCD_CAM.cam_ctrl.cam_update = 1;
    return ESP_OK;
}
