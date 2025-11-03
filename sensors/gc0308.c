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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sccb.h"
#include "gc0308.h"
#include "gc0308_regs.h"
#include "gc0308_settings.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
static const char *TAG = "gc0308";
#endif

#define H8(v) ((v)>>8)
#define L8(v) ((v)&0xff)

//#define REG_DEBUG_ON

static int read_reg(uint8_t slv_addr, const uint16_t reg)
{
    int ret = SCCB_Read(slv_addr, reg);
#ifdef REG_DEBUG_ON
    if (ret < 0) {
        ESP_LOGE(TAG, "READ REG 0x%04x FAILED: %d", reg, ret);
    }
#endif
    return ret;
}

static int write_reg(uint8_t slv_addr, const uint16_t reg, uint8_t value)
{
    int ret = 0;
#ifndef REG_DEBUG_ON
    ret = SCCB_Write(slv_addr, reg, value);
#else
    int old_value = read_reg(slv_addr, reg);
    if (old_value < 0) {
        return old_value;
    }
    if ((uint8_t)old_value != value) {
        ESP_LOGI(TAG, "NEW REG 0x%04x: 0x%02x to 0x%02x", reg, (uint8_t)old_value, value);
        ret = SCCB_Write(slv_addr, reg, value);
    } else {
        ESP_LOGD(TAG, "OLD REG 0x%04x: 0x%02x", reg, (uint8_t)old_value);
        ret = SCCB_Write(slv_addr, reg, value);//maybe not?
    }
    if (ret < 0) {
        ESP_LOGE(TAG, "WRITE REG 0x%04x FAILED: %d", reg, ret);
    }
#endif
    return ret;
}

static int check_reg_mask(uint8_t slv_addr, uint16_t reg, uint8_t mask)
{
    return (read_reg(slv_addr, reg) & mask) == mask;
}

static int set_reg_bits(uint8_t slv_addr, uint16_t reg, uint8_t offset, uint8_t mask, uint8_t value)
{
    int ret = 0;
    uint8_t c_value, new_value;
    ret = read_reg(slv_addr, reg);
    if (ret < 0) {
        return ret;
    }
    c_value = ret;
    new_value = (c_value & ~(mask << offset)) | ((value & mask) << offset);
    ret = write_reg(slv_addr, reg, new_value);
    return ret;
}

static int write_regs(uint8_t slv_addr, const uint8_t (*regs)[2], size_t regs_size)
{
    int i = 0, ret = 0;
    while (!ret && (i < regs_size)) {
        if (regs[i][0] == REG_DLY) {
            vTaskDelay(regs[i][1] / portTICK_PERIOD_MS);
        } else {
            ret = write_reg(slv_addr, regs[i][0], regs[i][1]);
        }
        i++;
    }
    return ret;
}

static void print_regs(uint8_t slv_addr)
{
#ifdef DEBUG_PRINT_REG
    ESP_LOGI(TAG, "REG list look ======================");
    for (size_t i = 0xf0; i <= 0xfe; i++) {
        ESP_LOGI(TAG, "reg[0x%02x] = 0x%02x", i, read_reg(slv_addr, i));
    }
    ESP_LOGI(TAG, "\npage 0 ===");
    write_reg(slv_addr, 0xfe, 0x00); // page 0
    for (size_t i = 0x03; i <= 0xa2; i++) {
        ESP_LOGI(TAG, "p0 reg[0x%02x] = 0x%02x", i, read_reg(slv_addr, i));
    }

    ESP_LOGI(TAG, "\npage 3 ===");
    write_reg(slv_addr, 0xfe, 0x03); // page 3
    for (size_t i = 0x01; i <= 0x43; i++) {
        ESP_LOGI(TAG, "p3 reg[0x%02x] = 0x%02x", i, read_reg(slv_addr, i));
    }
#endif
}

static int reset(sensor_t *sensor)
{
    int ret = 0;
    // Software Reset: clear all registers and reset them to their default values
    ret = write_reg(sensor->slv_addr, RESET_RELATED, 0xf0);
    if (ret) {
        ESP_LOGE(TAG, "Software Reset FAILED!");
        return ret;
    }

    vTaskDelay(80 / portTICK_PERIOD_MS);
    ret = write_regs(sensor->slv_addr, gc0308_sensor_default_regs, sizeof(gc0308_sensor_default_regs)/(sizeof(uint8_t) * 2));
    if (ret == 0) {
        ESP_LOGD(TAG, "Camera defaults loaded");
        vTaskDelay(80 / portTICK_PERIOD_MS);
        write_reg(sensor->slv_addr, 0xfe, 0x00);
#ifdef CONFIG_IDF_TARGET_ESP32
        set_reg_bits(sensor->slv_addr, 0x28, 4, 0x07, 1);  //frequency division for esp32, ensure pclk <= 15MHz
#endif
    }
    return ret;
}

static int set_pixformat(sensor_t *sensor, pixformat_t pixformat)
{
    int ret = 0;

    switch (pixformat) {
    case PIXFORMAT_RGB565:
        write_reg(sensor->slv_addr, 0xfe, 0x00);
        ret = set_reg_bits(sensor->slv_addr, 0x24, 0, 0x0f, 6);  //RGB565
        break;

    case PIXFORMAT_YUV422:
        write_reg(sensor->slv_addr, 0xfe, 0x00);
        ret = set_reg_bits(sensor->slv_addr, 0x24, 0, 0x0f, 2); //yuv422 Y Cb Y Cr
        break;
    case PIXFORMAT_GRAYSCALE:
        write_reg(sensor->slv_addr, 0xfe, 0x00);
        ret = write_reg(sensor->slv_addr, 0x24, 0xb1);
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
    int ret = 0;
    if (framesize > FRAMESIZE_VGA) {
        ESP_LOGW(TAG, "Invalid framesize: %u", framesize);
        framesize = FRAMESIZE_VGA;
    }
    sensor->status.framesize = framesize;
    uint16_t w = resolution[framesize].width;
    uint16_t h = resolution[framesize].height;
    uint16_t row_s = (resolution[FRAMESIZE_VGA].height - h) / 2;
    uint16_t col_s = (resolution[FRAMESIZE_VGA].width - w) / 2;
    (void)row_s;
    (void)col_s;

#if CONFIG_GC_SENSOR_SUBSAMPLE_MODE
    struct subsample_cfg {
        uint16_t ratio_numerator;
        uint16_t ratio_denominator;
        uint8_t reg0x54;
        uint8_t reg0x56;
        uint8_t reg0x57;
        uint8_t reg0x58;
        uint8_t reg0x59;
    };
    const struct subsample_cfg subsample_cfgs[] = { // define some subsample ratio
        {84, 420, 0x55, 0x00, 0x00, 0x00, 0x00}, //1/5
        {105, 420, 0x44, 0x00, 0x00, 0x00, 0x00},//1/4
        {140, 420, 0x33, 0x00, 0x00, 0x00, 0x00},//1/3
        {210, 420, 0x22, 0x00, 0x00, 0x00, 0x00},//1/2
        {240, 420, 0x77, 0x02, 0x46, 0x02, 0x46},//4/7
        {252, 420, 0x55, 0x02, 0x04, 0x02, 0x04},//3/5
        {280, 420, 0x33, 0x02, 0x00, 0x02, 0x00},//2/3
        {420, 420, 0x11, 0x00, 0x00, 0x00, 0x00},//1/1
    };
    uint16_t win_w = 640;
    uint16_t win_h = 480;
    const struct subsample_cfg *cfg = NULL;
    /**
     * Strategy: try to keep the maximum perspective
     */
    for (size_t i = 0; i < sizeof(subsample_cfgs) / sizeof(struct subsample_cfg); i++) {
        cfg = &subsample_cfgs[i];
        if ((win_w * cfg->ratio_numerator / cfg->ratio_denominator >= w) && (win_h * cfg->ratio_numerator / cfg->ratio_denominator >= h)) {
            win_w = w * cfg->ratio_denominator / cfg->ratio_numerator;
            win_h = h * cfg->ratio_denominator / cfg->ratio_numerator;
            row_s = (resolution[FRAMESIZE_VGA].height - win_h) / 2;
            col_s = (resolution[FRAMESIZE_VGA].width - win_w) / 2;
            ESP_LOGI(TAG, "subsample win:%dx%d, ratio:%f", win_w, win_h, (float)cfg->ratio_numerator / (float)cfg->ratio_denominator);
            break;
        }
    }

    write_reg(sensor->slv_addr, 0xfe, 0x00);

    write_reg(sensor->slv_addr, 0x05, H8(row_s));
    write_reg(sensor->slv_addr, 0x06, L8(row_s));
    write_reg(sensor->slv_addr, 0x07, H8(col_s));
    write_reg(sensor->slv_addr, 0x08, L8(col_s));
    write_reg(sensor->slv_addr, 0x09, H8(win_h + 8));
    write_reg(sensor->slv_addr, 0x0a, L8(win_h + 8));
    write_reg(sensor->slv_addr, 0x0b, H8(win_w + 8));
    write_reg(sensor->slv_addr, 0x0c, L8(win_w + 8));

    write_reg(sensor->slv_addr, 0xfe, 0x01);
    set_reg_bits(sensor->slv_addr, 0x53, 7, 0x01, 1);
    set_reg_bits(sensor->slv_addr, 0x55, 0, 0x01, 1);
    write_reg(sensor->slv_addr, 0x54, cfg->reg0x54);
    write_reg(sensor->slv_addr, 0x56, cfg->reg0x56);
    write_reg(sensor->slv_addr, 0x57, cfg->reg0x57);
    write_reg(sensor->slv_addr, 0x58, cfg->reg0x58);
    write_reg(sensor->slv_addr, 0x59, cfg->reg0x59);

    write_reg(sensor->slv_addr, 0xfe, 0x00);

#elif CONFIG_GC_SENSOR_WINDOWING_MODE
    write_reg(sensor->slv_addr, 0xfe, 0x00);

    write_reg(sensor->slv_addr, 0xf7, col_s / 4);
    write_reg(sensor->slv_addr, 0xf8, row_s / 4);
    write_reg(sensor->slv_addr, 0xf9, (col_s + w) / 4);
    write_reg(sensor->slv_addr, 0xfa, (row_s + h) / 4);

    write_reg(sensor->slv_addr, 0x05, H8(row_s));
    write_reg(sensor->slv_addr, 0x06, L8(row_s));
    write_reg(sensor->slv_addr, 0x07, H8(col_s));
    write_reg(sensor->slv_addr, 0x08, L8(col_s));

    write_reg(sensor->slv_addr, 0x09, H8(h + 8));
    write_reg(sensor->slv_addr, 0x0a, L8(h + 8));
    write_reg(sensor->slv_addr, 0x0b, H8(w + 8));
    write_reg(sensor->slv_addr, 0x0c, L8(w + 8));

#endif
    if (ret == 0) {
        ESP_LOGD(TAG, "Set framesize to: %ux%u", w, h);
    }
    return 0;
}

static int set_contrast(sensor_t *sensor, int level)
{
    int ret = 0;
    // GC0308 contrast range: -2 to +2 (mapped to register values)
    if (level < -2 || level > 2) {
        return -1;
    }
    ret = write_reg(sensor->slv_addr, 0xfe, 0x00);
    // Adjust contrast (CONTRAST register)
    // Default value is 0x40, adjust based on level
    uint8_t contrast_val = 0x40 + (level * 0x10);
    ret |= write_reg(sensor->slv_addr, CONTRAST, contrast_val);
    if (ret == 0) {
        sensor->status.contrast = level;
        ESP_LOGD(TAG, "Set contrast to: %d", level);
    }
    return ret;
}

static int set_brightness(sensor_t *sensor, int level)
{
    int ret = 0;
    // GC0308 brightness range: -2 to +2 (mapped to register values)
    if (level < -2 || level > 2) {
        return -1;
    }
    ret = write_reg(sensor->slv_addr, 0xfe, 0x00);
    // Adjust brightness via LSC target Y base (LSC_RED_B2)
    // Default value is 0x48, adjust based on level
    uint8_t brightness_val = 0x48 + (level * 0x10);
    ret |= write_reg(sensor->slv_addr, LSC_RED_B2, brightness_val);
    if (ret == 0) {
        sensor->status.brightness = level;
        ESP_LOGD(TAG, "Set brightness to: %d", level);
    }
    return ret;
}

static int set_saturation(sensor_t *sensor, int level)
{
    int ret = 0;
    // GC0308 saturation range: -2 to +2
    if (level < -2 || level > 2) {
        return -1;
    }
    ret = write_reg(sensor->slv_addr, 0xfe, 0x00);
    // Adjust saturation via Cb and Cr gain (SATURATION_CB1, SATURATION_CR1)
    // Default values: SATURATION_CB=0xcb, SATURATION_CB1=0x10, SATURATION_CR1=0x90
    uint8_t sat_base = 0x40 + (level * 0x10);
    ret |= write_reg(sensor->slv_addr, SATURATION_CB1, sat_base);
    ret |= write_reg(sensor->slv_addr, SATURATION_CR1, sat_base + 0x50);
    if (ret == 0) {
        sensor->status.saturation = level;
        ESP_LOGD(TAG, "Set saturation to: %d", level);
    }
    return ret;
}

static int set_sharpness(sensor_t *sensor, int level)
{
    int ret = 0;
    // GC0308 sharpness range: -2 to +2
    if (level < -2 || level > 2) {
        return -1;
    }
    ret = write_reg(sensor->slv_addr, 0xfe, 0x00);
    // Adjust sharpness via edge enhancement (EDGE_DEC_SA1-SA3)
    // Lower values = less sharp, higher values = more sharp
    uint8_t sharp_val = 0x20 + (level * 0x08);
    ret |= write_reg(sensor->slv_addr, EDGE_DEC_SA1, sharp_val);
    ret |= write_reg(sensor->slv_addr, EDGE_DEC_SA2, sharp_val);
    ret |= write_reg(sensor->slv_addr, EDGE_DEC_SA3, sharp_val);
    if (ret == 0) {
        sensor->status.sharpness = level;
        ESP_LOGD(TAG, "Set sharpness to: %d", level);
    }
    return ret;
}

static int set_whitebal(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = write_reg(sensor->slv_addr, 0xfe, 0x00);
    // AWB enable/disable via AEC_MODE1 register, AWB_ENABLE bit
    ret |= set_reg_bits(sensor->slv_addr, AEC_MODE1, 1, 0x01, enable != 0);
    if (ret == 0) {
        sensor->status.awb = enable;
        ESP_LOGD(TAG, "Set AWB to: %d", enable);
    }
    return ret;
}

static int set_gain_ctrl(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = write_reg(sensor->slv_addr, 0xfe, 0x00);
    // AGC enable/disable via AEC_MODE1 register, AGC_ENABLE bit
    ret |= set_reg_bits(sensor->slv_addr, AEC_MODE1, 2, 0x01, enable != 0);
    if (ret == 0) {
        sensor->status.agc = enable;
        ESP_LOGD(TAG, "Set AGC to: %d", enable);
    }
    return ret;
}

static int set_exposure_ctrl(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = write_reg(sensor->slv_addr, 0xfe, 0x00);
    // AEC enable/disable via AEC_MODE1 register, AEC_ENABLE bit
    ret |= set_reg_bits(sensor->slv_addr, AEC_MODE1, 0, 0x01, enable != 0);
    if (ret == 0) {
        sensor->status.aec = enable;
        ESP_LOGD(TAG, "Set AEC to: %d", enable);
    }
    return ret;
}

static int set_hmirror(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = write_reg(sensor->slv_addr, 0xfe, 0x00);
    ret |= set_reg_bits(sensor->slv_addr, CISCTL_MODE1, 0, 0x01, enable != 0);
    if (ret == 0) {
        sensor->status.hmirror = enable;
        ESP_LOGD(TAG, "Set h-mirror to: %d", enable);
    }
    return ret;
}

static int set_vflip(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = write_reg(sensor->slv_addr, 0xfe, 0x00);
    ret |= set_reg_bits(sensor->slv_addr, CISCTL_MODE1, 1, 0x01, enable != 0);
    if (ret == 0) {
        sensor->status.vflip = enable;
        ESP_LOGD(TAG, "Set v-flip to: %d", enable);
    }
    return ret;
}


static int set_agc_gain(sensor_t *sensor, int gain)
{
    int ret = 0;
    // GC0308 AGC gain range: 0-30 (standard sensor API range)
    // Maps to hardware register values 0x00-0x3F (6-bit effective range)
    // Hardware default is 0x14 (20)
    if (gain < 0) {
        gain = 0;
    } else if (gain > 30) {
        gain = 30;
    }
    // Map API range 0-30 to hardware range 0-63 (approximately 2x multiplier)
    uint8_t gain_value = (gain * 63) / 30;
    ret = write_reg(sensor->slv_addr, 0xfe, 0x00);
    ret |= write_reg(sensor->slv_addr, GLOBAL_GAIN, gain_value);
    if (ret == 0) {
        sensor->status.agc_gain = gain;
        ESP_LOGD(TAG, "Set AGC gain to: %d (hw: 0x%02x)", gain, gain_value);
    }
    return ret;
}

static int set_colorbar(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = write_reg(sensor->slv_addr, 0xfe, 0x00);
    ret |= set_reg_bits(sensor->slv_addr, OUT_CTRL, 0, 0x01, enable);
    if (ret == 0) {
        sensor->status.colorbar = enable;
        ESP_LOGD(TAG, "Set colorbar to: %d", enable);
    }
    return ret;
}

static int set_special_effect(sensor_t *sensor, int effect)
{
    int ret = 0;
    // Effect values: 0=Normal, 1=Negative, 2=Grayscale, 3=Red Tint, 4=Green Tint, 5=Blue Tint, 6=Sepia
    if (effect < 0 || effect > 6) {
        return -1;
    }
    ret = write_reg(sensor->slv_addr, 0xfe, 0x00);
    switch (effect) {
        case 0: // Normal
            ret |= set_reg_bits(sensor->slv_addr, SPECIAL_EFFECT, 0, 0x03, EFFECT_NORMAL);
            ret |= write_reg(sensor->slv_addr, EFFECT_MODE, 0x0a);
            ret |= write_reg(sensor->slv_addr, FIXED_CB, 0x7f);
            ret |= write_reg(sensor->slv_addr, FIXED_CR, 0xfa);
            break;
        case 1: // Negative
            ret |= set_reg_bits(sensor->slv_addr, SPECIAL_EFFECT, 0, 0x03, EFFECT_NEGATIVE);
            break;
        case 2: // Grayscale (B&W)
            ret |= set_reg_bits(sensor->slv_addr, SPECIAL_EFFECT, 0, 0x03, EFFECT_GRAYSCALE);
            ret |= write_reg(sensor->slv_addr, EFFECT_MODE, 0x0a);
            ret |= write_reg(sensor->slv_addr, FIXED_CB, 0x7f);
            ret |= write_reg(sensor->slv_addr, FIXED_CR, 0xfa);
            break;
        case 3: // Red Tint
            ret |= set_reg_bits(sensor->slv_addr, SPECIAL_EFFECT, 0, 0x03, EFFECT_GRAYSCALE);
            ret |= write_reg(sensor->slv_addr, EFFECT_MODE, 0x0a);
            ret |= write_reg(sensor->slv_addr, FIXED_CB, 0x80);
            ret |= write_reg(sensor->slv_addr, FIXED_CR, 0xc0);
            break;
        case 4: // Green Tint
            ret |= set_reg_bits(sensor->slv_addr, SPECIAL_EFFECT, 0, 0x03, EFFECT_GRAYSCALE);
            ret |= write_reg(sensor->slv_addr, EFFECT_MODE, 0x0a);
            ret |= write_reg(sensor->slv_addr, FIXED_CB, 0x50);
            ret |= write_reg(sensor->slv_addr, FIXED_CR, 0x50);
            break;
        case 5: // Blue Tint
            ret |= set_reg_bits(sensor->slv_addr, SPECIAL_EFFECT, 0, 0x03, EFFECT_GRAYSCALE);
            ret |= write_reg(sensor->slv_addr, EFFECT_MODE, 0x0a);
            ret |= write_reg(sensor->slv_addr, FIXED_CB, 0xa0);
            ret |= write_reg(sensor->slv_addr, FIXED_CR, 0x80);
            break;
        case 6: // Sepia
            ret |= set_reg_bits(sensor->slv_addr, SPECIAL_EFFECT, 0, 0x03, EFFECT_GRAYSCALE);
            ret |= write_reg(sensor->slv_addr, EFFECT_MODE, 0x0a);
            ret |= write_reg(sensor->slv_addr, FIXED_CB, 0xd0);
            ret |= write_reg(sensor->slv_addr, FIXED_CR, 0x28);
            break;
        default:
            ret = -1;
            break;
    }
    if (ret == 0) {
        sensor->status.special_effect = effect;
        ESP_LOGD(TAG, "Set special effect to: %d", effect);
    }
    return ret;
}

static int set_wb_mode(sensor_t *sensor, int mode)
{
    int ret = 0;
    // WB modes: 0=Auto, 1=Sunny, 2=Cloudy, 3=Office, 4=Home
    if (mode < 0 || mode > 4) {
        return -1;
    }
    ret = write_reg(sensor->slv_addr, 0xfe, 0x00);
    if (mode == 0) {
        // Auto WB - enable AWB via AEC_MODE1
        ret |= set_reg_bits(sensor->slv_addr, AEC_MODE1, 1, 0x01, 1);
    } else {
        // Manual WB - disable AWB and set specific gains
        ret |= set_reg_bits(sensor->slv_addr, AEC_MODE1, 1, 0x01, 0);
        switch (mode) {
            case 1: // Sunny
                ret |= write_reg(sensor->slv_addr, AWB_R_GAIN, 0x74);
                ret |= write_reg(sensor->slv_addr, AWB_G_GAIN, 0x52);
                ret |= write_reg(sensor->slv_addr, AWB_B_GAIN, 0x40);
                break;
            case 2: // Cloudy
                ret |= write_reg(sensor->slv_addr, AWB_R_GAIN, 0x8c);
                ret |= write_reg(sensor->slv_addr, AWB_G_GAIN, 0x50);
                ret |= write_reg(sensor->slv_addr, AWB_B_GAIN, 0x40);
                break;
            case 3: // Office
                ret |= write_reg(sensor->slv_addr, AWB_R_GAIN, 0x48);
                ret |= write_reg(sensor->slv_addr, AWB_G_GAIN, 0x40);
                ret |= write_reg(sensor->slv_addr, AWB_B_GAIN, 0x5c);
                break;
            case 4: // Home
                ret |= write_reg(sensor->slv_addr, AWB_R_GAIN, 0x40);
                ret |= write_reg(sensor->slv_addr, AWB_G_GAIN, 0x54);
                ret |= write_reg(sensor->slv_addr, AWB_B_GAIN, 0x70);
                break;
            default:
                ret = -1;
                break;
        }
    }
    if (ret == 0) {
        sensor->status.wb_mode = mode;
        ESP_LOGD(TAG, "Set WB mode to: %d", mode);
    }
    return ret;
}

static int set_ae_level(sensor_t *sensor, int level)
{
    int ret = 0;
    // AE level range: -2 to +2
    if (level < -2 || level > 2) {
        return -1;
    }
    ret = write_reg(sensor->slv_addr, 0xfe, 0x01);
    // Adjust AE target via AEC_TARGET_Y register (Page 1)
    // Default is around 0x19, adjust based on level
    uint8_t ae_target = 0x19 + (level * 0x08);
    ret |= write_reg(sensor->slv_addr, AEC_TARGET_Y, ae_target);
    ret |= write_reg(sensor->slv_addr, 0xfe, 0x00);
    if (ret == 0) {
        sensor->status.ae_level = level;
        ESP_LOGD(TAG, "Set AE level to: %d", level);
    }
    return ret;
}

static int get_reg(sensor_t *sensor, int reg, int mask)
{
    int ret = 0;
    if (mask > 0xFF) {
        ESP_LOGE(TAG, "mask should not more than 0xff");
    } else {
        ret = read_reg(sensor->slv_addr, reg);
    }
    if (ret > 0) {
        ret &= mask;
    }
    return ret;
}

static int set_reg(sensor_t *sensor, int reg, int mask, int value)
{
    int ret = 0;
    if (mask > 0xFF) {
        ESP_LOGE(TAG, "mask should not more than 0xff");
    } else {
        ret = read_reg(sensor->slv_addr, reg);
    }
    if (ret < 0) {
        return ret;
    }
    value = (ret & ~mask) | (value & mask);

    if (mask > 0xFF) {

    } else {
        ret = write_reg(sensor->slv_addr, reg, value);
    }
    return ret;
}

static int init_status(sensor_t *sensor)
{
    int ret;
    ret = write_reg(sensor->slv_addr, 0xfe, 0x00);

    // Read brightness from LSC_RED_B2 register (0xd6)
    // Default is 0x48, map back to level: (reg_val - 0x48) / 0x10
    ret = read_reg(sensor->slv_addr, LSC_RED_B2);
    if (ret >= 0) {
        int brightness_level = (ret - 0x48) / 0x10;
        sensor->status.brightness = (brightness_level < -2) ? -2 : (brightness_level > 2) ? 2 : brightness_level;
    } else {
        sensor->status.brightness = 0;
    }

    // Read contrast from CONTRAST register (0xb3)
    // Default is 0x40, map back to level: (reg_val - 0x40) / 0x10
    ret = read_reg(sensor->slv_addr, CONTRAST);
    if (ret >= 0) {
        int contrast_level = (ret - 0x40) / 0x10;
        sensor->status.contrast = (contrast_level < -2) ? -2 : (contrast_level > 2) ? 2 : contrast_level;
    } else {
        sensor->status.contrast = 0;
    }

    // Read saturation from SATURATION_CB1 register (0x54)
    // Default is 0x40, map back to level: (reg_val - 0x40) / 0x10
    ret = read_reg(sensor->slv_addr, SATURATION_CB1);
    if (ret >= 0) {
        int saturation_level = (ret - 0x40) / 0x10;
        sensor->status.saturation = (saturation_level < -2) ? -2 : (saturation_level > 2) ? 2 : saturation_level;
    } else {
        sensor->status.saturation = 0;
    }

    // Read sharpness from EDGE_DEC_SA1 register (0x8b)
    // Default is 0x20, map back to level: (reg_val - 0x20) / 0x08
    ret = read_reg(sensor->slv_addr, EDGE_DEC_SA1);
    if (ret >= 0) {
        int sharpness_level = (ret - 0x20) / 0x08;
        sensor->status.sharpness = (sharpness_level < -2) ? -2 : (sharpness_level > 2) ? 2 : sharpness_level;
    } else {
        sensor->status.sharpness = 0;
    }

    // Denoise not supported by GC0308
    sensor->status.denoise = 0;

    // Read AE level from AEC_TARGET_Y register (Page 1, 0xf7)
    ret = write_reg(sensor->slv_addr, 0xfe, 0x01);
    ret = read_reg(sensor->slv_addr, AEC_TARGET_Y);
    ret = write_reg(sensor->slv_addr, 0xfe, 0x00);
    if (ret >= 0) {
        int ae_level = (ret - 0x19) / 0x08;
        sensor->status.ae_level = (ae_level < -2) ? -2 : (ae_level > 2) ? 2 : ae_level;
    } else {
        sensor->status.ae_level = 0;
    }

    // Gainceiling not supported by GC0308
    sensor->status.gainceiling = 0;

    // Read AWB, AGC, AEC status from AEC_MODE1 register (0xd2)
    sensor->status.awb = check_reg_mask(sensor->slv_addr, AEC_MODE1, AWB_ENABLE);
    sensor->status.agc = check_reg_mask(sensor->slv_addr, AEC_MODE1, AGC_ENABLE);
    sensor->status.aec = check_reg_mask(sensor->slv_addr, AEC_MODE1, AEC_ENABLE);

    // DCW not supported by GC0308
    sensor->status.dcw = 0;

    // Read hmirror and vflip from CISCTL_MODE1 register (0x14)
    sensor->status.hmirror = check_reg_mask(sensor->slv_addr, CISCTL_MODE1, HMIRROR_MASK);
    sensor->status.vflip = check_reg_mask(sensor->slv_addr, CISCTL_MODE1, VFLIP_MASK);

    // Read colorbar status from OUT_CTRL register (0x2e)
    sensor->status.colorbar = check_reg_mask(sensor->slv_addr, OUT_CTRL, 0x01);

    // BPC, WPC not specifically tracked by GC0308 driver
    sensor->status.bpc = 0;
    sensor->status.wpc = 0;

    // Raw GMA, LENC not supported by GC0308
    sensor->status.raw_gma = 0;
    sensor->status.lenc = 0;

    // Quality not supported by GC0308
    sensor->status.quality = 0;

    // Read special effect from SPECIAL_EFFECT register (0x23)
    ret = read_reg(sensor->slv_addr, SPECIAL_EFFECT);
    if (ret >= 0) {
        // Map hardware register value to effect enum
        // Bit[1:0] of SPECIAL_EFFECT register holds the effect mode
        uint8_t effect_bits = ret & 0x03;
        if (effect_bits == EFFECT_NORMAL) {
            sensor->status.special_effect = 0; // Normal
        } else if (effect_bits == EFFECT_NEGATIVE) {
            sensor->status.special_effect = 1; // Negative
        } else if (effect_bits == EFFECT_GRAYSCALE) {
            // Could be Grayscale (2) or tint effects (3-6) - requires checking FIXED_CB/CR
            // For simplicity, default to Grayscale when in EFFECT_GRAYSCALE mode
            sensor->status.special_effect = 2; // Grayscale/B&W
        } else {
            sensor->status.special_effect = 0; // Default to Normal
        }
    } else {
        sensor->status.special_effect = 0;
    }

    // Read white balance mode from AWB registers
    // This is simplified - full WB mode detection would need to check multiple registers
    if (sensor->status.awb) {
        sensor->status.wb_mode = 0; // Auto
    } else {
        // Check if specific preset is active (simplified detection)
        sensor->status.wb_mode = 0; // Default to Auto
    }

    // AWB gain not specifically tracked
    sensor->status.awb_gain = 0;

    // Read AGC gain from GLOBAL_GAIN register (0x50)
    // Hardware range is 0-63 (6-bit), map to 0-30 for API compatibility
    ret = read_reg(sensor->slv_addr, GLOBAL_GAIN);
    if (ret >= 0) {
        // Reverse map from hardware (0-63) to API (0-30)
        sensor->status.agc_gain = (ret * 30) / 63;
        if (sensor->status.agc_gain > 30) sensor->status.agc_gain = 30;
    } else {
        sensor->status.agc_gain = 0;
    }

    // AEC value not specifically tracked by GC0308
    sensor->status.aec_value = 0;

    // AEC2 not supported by GC0308
    sensor->status.aec2 = 0;

    print_regs(sensor->slv_addr);
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

int esp32_camera_gc0308_detect(int slv_addr, sensor_id_t *id)
{
    if (GC0308_SCCB_ADDR == slv_addr) {
        write_reg(slv_addr, 0xfe, 0x00);
        uint8_t PID = SCCB_Read(slv_addr, 0x00);
        if (GC0308_PID == PID) {
            id->PID = PID;
            return PID;
        } else {
            ESP_LOGI(TAG, "Mismatch PID=0x%x", PID);
        }
    }
    return 0;
}

int esp32_camera_gc0308_init(sensor_t *sensor)
{
    sensor->init_status = init_status;
    sensor->reset = reset;
    sensor->set_pixformat = set_pixformat;
    sensor->set_framesize = set_framesize;
    sensor->set_contrast = set_contrast;
    sensor->set_brightness = set_brightness;
    sensor->set_saturation = set_saturation;
    sensor->set_sharpness = set_sharpness;
    sensor->set_denoise = set_dummy;
    sensor->set_gainceiling = set_gainceiling_dummy;
    sensor->set_quality = set_dummy;
    sensor->set_colorbar = set_colorbar;
    sensor->set_whitebal = set_whitebal;
    sensor->set_gain_ctrl = set_gain_ctrl;
    sensor->set_exposure_ctrl = set_exposure_ctrl;
    sensor->set_hmirror = set_hmirror;
    sensor->set_vflip = set_vflip;

    sensor->set_aec2 = set_dummy;
    sensor->set_awb_gain = set_dummy;
    sensor->set_agc_gain = set_agc_gain;
    sensor->set_aec_value = set_dummy;

    sensor->set_special_effect = set_special_effect;
    sensor->set_wb_mode = set_wb_mode;
    sensor->set_ae_level = set_ae_level;

    sensor->set_dcw = set_dummy;
    sensor->set_bpc = set_dummy;
    sensor->set_wpc = set_dummy;

    sensor->set_raw_gma = set_dummy;
    sensor->set_lenc = set_dummy;

    sensor->get_reg = get_reg;
    sensor->set_reg = set_reg;
    sensor->set_res_raw = NULL;
    sensor->set_pll = NULL;
    sensor->set_xclk = NULL;

    ESP_LOGD(TAG, "GC0308 Attached");
    return 0;
}
