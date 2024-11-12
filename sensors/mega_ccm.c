// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sccb.h"
#include "mega_ccm.h"
#include "mega_ccm_regs.h"
#include "mega_ccm_settings.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
static const char *TAG = "mega_ccm";
#endif

#define H8(v) ((v)>>8)
#define L8(v) ((v)&0xff)

//#define REG_DEBUG_ON


static int read_reg(uint8_t slv_addr, const uint16_t reg){
    int ret = SCCB_Read16(slv_addr, reg);
#ifdef REG_DEBUG_ON
    if (ret < 0) {
        ESP_LOGE(TAG, "READ REG 0x%04x FAILED: %d", reg, ret);
    }
#endif
    return ret;
}

static int write_reg(uint8_t slv_addr, const uint16_t reg, uint8_t value){
    int ret = 0;
    ret = SCCB_Write16(slv_addr, reg, value);
    return ret;
}


static int reset(sensor_t *sensor)
{
    int ret;
    ret = write_reg(sensor->slv_addr, CAMERA_RST_REG, 0x00);
    ret += write_reg(sensor->slv_addr, CAMERA_RST_REG, 0x01);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    return ret;
}

static int set_pixformat(sensor_t *sensor, pixformat_t pixformat)
{
    int ret = 0;
    switch (pixformat) {
    case PIXFORMAT_JPEG:
        ret = write_reg(sensor->slv_addr, PIXEL_FMT_REG, 0x01);
        break;
    case PIXFORMAT_RGB565:
        ret = write_reg(sensor->slv_addr, PIXEL_FMT_REG, 0x02);
        break;
    case PIXFORMAT_YUV422:
        ret = write_reg(sensor->slv_addr, PIXEL_FMT_REG, 0x03);
        break;
    default:
        ESP_LOGW(TAG, "unsupport format");
        ret = -1;
        break;
    }
    if (ret == 0) {
        sensor->pixformat = pixformat;
        ESP_LOGD(TAG, "Set pixformat to: %u", pixformat);
    }

    return ret;
}

static int set_framesize(sensor_t *sensor, framesize_t framesize)
{
    ESP_LOGI(TAG, "set_framesize");
    int ret = 0;
    if (framesize > FRAMESIZE_5MP) {
        ESP_LOGW(TAG, "Invalid framesize: %u", framesize);
        framesize = FRAMESIZE_5MP;
    }
    sensor->status.framesize = framesize;
    uint16_t w = resolution[framesize].width;
    uint16_t h = resolution[framesize].height;
    switch (framesize){
        case FRAMESIZE_QVGA:
             ret = write_reg(sensor->slv_addr, RESOLUTION_REG, 0x01); //320x240
             ret += write_reg(sensor->slv_addr, SYSTEM_CLK_DIV_REG, 0x02); // set system clk 
             ret += write_reg(sensor->slv_addr, SYSTEM_PLL_DIV_REG, 0x01); // set system pll
        break;
        case FRAMESIZE_VGA:
             ret = write_reg(sensor->slv_addr, RESOLUTION_REG, 0x02); //640x480
             ret += write_reg(sensor->slv_addr, SYSTEM_CLK_DIV_REG, 0x02); // set system clk 
             ret += write_reg(sensor->slv_addr, SYSTEM_PLL_DIV_REG, 0x01); // set system pll
        break;
        case FRAMESIZE_HD:
            ret = write_reg(sensor->slv_addr, RESOLUTION_REG, 0x03); //1280x720
            ret += write_reg(sensor->slv_addr, SYSTEM_CLK_DIV_REG, 0x02); // set system clk 
            ret += write_reg(sensor->slv_addr, SYSTEM_PLL_DIV_REG, 0x01); // set system pll
        break;
        case FRAMESIZE_UXGA:
            ret = write_reg(sensor->slv_addr, RESOLUTION_REG, 0x04); //1600x1200
            ret += write_reg(sensor->slv_addr, SYSTEM_CLK_DIV_REG, 0x02); // set system clk 
             ret += write_reg(sensor->slv_addr, SYSTEM_PLL_DIV_REG, 0x01); // set system pll
        break;
        case FRAMESIZE_FHD:
            ret = write_reg(sensor->slv_addr, RESOLUTION_REG, 0x05); //1920x1080
            ret += write_reg(sensor->slv_addr, SYSTEM_CLK_DIV_REG, 0x02); // set system clk 
            ret += write_reg(sensor->slv_addr, SYSTEM_PLL_DIV_REG, 0x01); // set system pll
        break;
        case FRAMESIZE_5MP:
            ret = write_reg(sensor->slv_addr, RESOLUTION_REG, 0x06); //2592x1944
        break;
        case FRAMESIZE_96X96:
            ret = write_reg(sensor->slv_addr, RESOLUTION_REG, 0x07); //96x96
        break;
        case FRAMESIZE_128X128:
            ret = write_reg(sensor->slv_addr, RESOLUTION_REG, 0x08); //128x128
        break;
        case FRAMESIZE_320X320:
            ret = write_reg(sensor->slv_addr, RESOLUTION_REG, 0x09); //320x320
        break;
         default:
        ESP_LOGW(TAG, "unsupport framesize");
        ret = -1;
        break;
    }
    if (ret == 0) {
        ESP_LOGD(TAG, "Set framesize to: %ux%u", w, h);
    }
    return ret;
}

static int set_hmirror(sensor_t *sensor, int enable)
{
    int ret = 0;
    sensor->status.hmirror = enable;
    ret = write_reg(sensor->slv_addr, IMAGE_MIRROR_REG, enable); 
    if (ret == 0) {
        ESP_LOGD(TAG, "Set h-mirror to: %d", enable);
    }
    return ret;
}

static int set_vflip(sensor_t *sensor, int enable)
{
    int ret = 0;
    sensor->status.vflip = enable;
    ret = write_reg(sensor->slv_addr, IMAGE_FLIP_REG, enable); 
    if (ret == 0) {
        ESP_LOGD(TAG, "Set v-flip to: %d", enable);
    }
    return ret;
}
static int set_quality(sensor_t *sensor, int qs)
{
    int ret = 0;
    ret = write_reg(sensor->slv_addr, IMAGE_QUALITY_REG, qs);
    if (ret == 0) {
        sensor->status.quality = qs;
        ESP_LOGD(TAG, "Set quality to: %d", qs);
    }
    return ret;
}


static int set_brightness(sensor_t *sensor, int level)
{
    int ret = 0;
    if(level < 0) {
        level = 0;
    } else if(level > 8) {
        level = 8;
    }
    ret = write_reg(sensor->slv_addr, BRIGHTNESS_REG, level);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set brightness to: %d", level);
        sensor->status.brightness = level;
    }
    return ret;
}

static int set_contrast (sensor_t *sensor, int level)
{
     int ret = 0;
    if(level < 0) {
        level = 0;
    } else if(level > 6) {
        level = 6;
    }
    ret = write_reg(sensor->slv_addr, CONTRAST_REG, level);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set contrast to: %d", level);
        sensor->status.contrast = level;
    }
    return ret;
}

static int set_saturation (sensor_t *sensor, int level)
{
     int ret = 0;
    if(level < 0) {
        level = 0;
    } else if(level > 6) {
        level = 6;
    }
    ret = write_reg(sensor->slv_addr, SATURATION_REG, level);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set saturation to: %d", level);
        sensor->status.saturation = level;
    }
    return ret;
}
static int set_agc_mode (sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = write_reg(sensor->slv_addr, AGC_MODE_REG, enable);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set agc mode to: %d", enable);
        sensor->status.aec = enable;
    }
    return ret;
}


static int set_wb_mode (sensor_t *sensor, int mode)
{
  int ret = 0;
    if(mode < 0) {
        mode = 0;
    } else if(mode > 5) {
        mode = 5;
    }
    ret = write_reg(sensor->slv_addr, AWB_MODE_REG, mode);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set wb_mode to: %d", mode);
        sensor->status.wb_mode = mode;
    }
    return ret;

}

static int set_special_effect (sensor_t *sensor, int effect)
{

     int ret = 0;
    if(effect < 0) {
        effect = 0;
    } else if(effect > 6) {
        effect = 6;
    }
    ret = write_reg(sensor->slv_addr, SPECIAL_REG, effect);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set special_effect to: %d", effect);
        sensor->status.special_effect = effect;
    }
    return ret;

}


static int analog_gain (sensor_t *sensor, int val)
{

     int ret = 0;
    ret = write_reg(sensor->slv_addr, MANUAL_AGC_REG, val);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set analog gain to: %d", val);
    }
    return ret;

}


static int exposure_line (sensor_t *sensor, int val)
{

     int ret = 0;
    ret = write_reg(sensor->slv_addr, MANUAL_EXP_H_REG, val>>8);
    ret += write_reg(sensor->slv_addr, MANUAL_EXP_L_REG, val>>8);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set exposure_line to: %d", val);
    }
    return ret;

}


static int init_status(sensor_t *sensor)
{
    sensor->status.brightness = 0;
    sensor->status.contrast = 0;
    sensor->status.saturation = 0;
    sensor->status.sharpness = 0;
    sensor->status.denoise = 0;
    sensor->status.ae_level = 0;
    sensor->status.gainceiling = 0;
    sensor->status.awb = 0;
    sensor->status.dcw = 0;
    sensor->status.agc = 0;
    sensor->status.aec = 0;
    sensor->status.hmirror = 0;
    sensor->status.vflip = 0;
    sensor->status.colorbar = 0;
    sensor->status.bpc = 0;
    sensor->status.wpc = 0;
    sensor->status.raw_gma = 0;
    sensor->status.lenc = 0;
    sensor->status.quality = 0;
    sensor->status.special_effect = 0;
    sensor->status.wb_mode = 0;
    sensor->status.awb_gain = 0;
    sensor->status.agc_gain = 0;
    sensor->status.aec_value = 0;
    sensor->status.aec2 = 0;
    return 0;
}

static int set_dummy(sensor_t *sensor, int val)
{
    ESP_LOGW(TAG, "Unsupported");
    return -1;
}
static int set_gainceiling_dummy(sensor_t *sensor, gainceiling_t val)
{
    ESP_LOGW(TAG, "Unsupported");
    return -1;
}

int mega_ccm_detect(int slv_addr, sensor_id_t *id)
{
    if (MEGA_CCM_SCCB_ADDR == slv_addr) {
        uint8_t h = read_reg(slv_addr, SENSOR_ID_HIGH);
        uint8_t l = read_reg(slv_addr, SENSOR_ID_LOW);
        uint16_t PID = (h<<8) | l;
        if (MEGA_CCM_PID == PID) {
            id->PID = PID;
            return PID;
        } else {
            ESP_LOGI(TAG, "Mismatch PID=0x%x", PID);
        }
    }
    return 0;
}

int mega_ccm_init(sensor_t *sensor)
{
    sensor->init_status = init_status;
    sensor->reset = reset;
    sensor->set_pixformat = set_pixformat;
    sensor->set_framesize = set_framesize;
    sensor->set_contrast = set_contrast;
    sensor->set_brightness = set_brightness;
    sensor->set_saturation = set_saturation;
    sensor->set_sharpness = set_dummy;
    sensor->set_denoise = set_dummy;
    sensor->set_gainceiling = set_gainceiling_dummy;
    sensor->set_quality = set_quality;
    sensor->set_colorbar = set_dummy;
    sensor->set_whitebal = set_dummy;
    sensor->set_gain_ctrl = set_dummy;
    sensor->set_exposure_ctrl = set_dummy;
    sensor->set_hmirror = set_hmirror;
    sensor->set_vflip = set_vflip;

    sensor->set_aec2 = set_agc_mode;
    sensor->set_awb_gain = set_dummy;
    sensor->set_agc_gain = analog_gain;
    sensor->set_aec_value = exposure_line;

    sensor->set_special_effect = set_special_effect;
    sensor->set_wb_mode = set_wb_mode;
    sensor->set_ae_level = set_dummy;

    sensor->set_dcw = set_dummy;
    sensor->set_bpc = set_dummy;
    sensor->set_wpc = set_dummy;

    sensor->set_raw_gma = set_dummy;
    sensor->set_lenc = set_dummy;

    sensor->get_reg = NULL;
    sensor->set_reg = NULL;
    sensor->set_res_raw = NULL;
    sensor->set_pll = NULL;
    sensor->set_xclk = NULL;

    ESP_LOGD(TAG, "MEGA_CCM Attached");
    return 0;
}
