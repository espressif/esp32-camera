/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * OV3640 driver.
 *
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "sccb.h"
#include "xclk.h"
#include "ov3640.h"
#include "ov3640_regs.h"
#include "ov3640_settings.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
static const char *TAG = "ov3640";
#endif

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

static int check_reg_mask(uint8_t slv_addr, uint16_t reg, uint8_t mask){
    return (read_reg(slv_addr, reg) & mask) == mask;
}

static int read_reg16(uint8_t slv_addr, const uint16_t reg){
    int ret = 0, ret2 = 0;
    ret = read_reg(slv_addr, reg);
    if (ret >= 0) {
        ret = (ret & 0xFF) << 8;
        ret2 = read_reg(slv_addr, reg+1);
        if (ret2 < 0) {
            ret = ret2;
        } else {
            ret |= ret2 & 0xFF;
        }
    }
    return ret;
}


static int write_reg(uint8_t slv_addr, const uint16_t reg, uint8_t value){
    int ret = 0;
#ifndef REG_DEBUG_ON
    ret = SCCB_Write16(slv_addr, reg, value);
#else
    int old_value = read_reg(slv_addr, reg);
    if (old_value < 0) {
        return old_value;
    }
    if ((uint8_t)old_value != value) {
        ESP_LOGI(TAG, "NEW REG 0x%04x: 0x%02x to 0x%02x", reg, (uint8_t)old_value, value);
        ret = SCCB_Write16(slv_addr, reg, value);
    } else {
        ESP_LOGD(TAG, "OLD REG 0x%04x: 0x%02x", reg, (uint8_t)old_value);
        ret = SCCB_Write16(slv_addr, reg, value);//maybe not?
    }
    if (ret < 0) {
        ESP_LOGE(TAG, "WRITE REG 0x%04x FAILED: %d", reg, ret);
    }
#endif
    return ret;
}

static int set_reg_bits(uint8_t slv_addr, uint16_t reg, uint8_t offset, uint8_t mask, uint8_t value)
{
    int ret = 0;
    uint8_t c_value, new_value;
    ret = read_reg(slv_addr, reg);
    if(ret < 0) {
        return ret;
    }
    c_value = ret;
    new_value = (c_value & ~(mask << offset)) | ((value & mask) << offset);
    ret = write_reg(slv_addr, reg, new_value);
    return ret;
}

static int write_regs(uint8_t slv_addr, const uint16_t (*regs)[2])
{
    int i = 0, ret = 0;
    while (!ret && regs[i][0] != REGLIST_TAIL) {
        if (regs[i][0] == REG_DLY) {
            vTaskDelay(regs[i][1] / portTICK_PERIOD_MS);
        } else {
            ret = write_reg(slv_addr, regs[i][0], regs[i][1]);
        }
        i++;
    }
    return ret;
}

static int write_reg16(uint8_t slv_addr, const uint16_t reg, uint16_t value)
{
    if (write_reg(slv_addr, reg, value >> 8) || write_reg(slv_addr, reg + 1, value)) {
        return -1;
    }
    return 0;
}

static int write_addr_reg(uint8_t slv_addr, const uint16_t reg, uint16_t x_value, uint16_t y_value)
{
    if (write_reg16(slv_addr, reg, x_value) || write_reg16(slv_addr, reg + 2, y_value)) {
        return -1;
    }
    return 0;
}

#define write_reg_bits(slv_addr, reg, mask, enable) set_reg_bits(slv_addr, reg, 0, mask, enable?mask:0)

static int calc_sysclk(int xclk, bool pll_bypass, int pll_multiplier, int pll_sys_div, int pll_pre_div, bool pll_root_2x, int pll_seld5, bool pclk_manual, int pclk_div)
{
    const int pll_pre_div2x_map[] = { 2, 3, 4, 6 };//values are multiplied by two to avoid floats
    const int pll_seld52x_map[] = { 2, 2, 4, 5 };

    if(!pll_sys_div) {
        pll_sys_div = 1;
    }

    int pll_pre_div2x = pll_pre_div2x_map[pll_pre_div];
    int pll_root_div = pll_root_2x?2:1;
    int pll_seld52x = pll_seld52x_map[pll_seld5];

    int VCO = (xclk / 1000) * pll_multiplier * pll_root_div * 2 / pll_pre_div2x;
    int PLLCLK = pll_bypass?(xclk):(VCO * 1000 * 2 / pll_sys_div / pll_seld52x);
    int PCLK = PLLCLK / 2 / ((pclk_manual && pclk_div)?pclk_div:1);
    int SYSCLK = PLLCLK / 4;

    ESP_LOGI(TAG, "Calculated VCO: %d Hz, PLLCLK: %d Hz, SYSCLK: %d Hz, PCLK: %d Hz", VCO*1000, PLLCLK, SYSCLK, PCLK);
    return SYSCLK;
}

static int set_pll(sensor_t *sensor, bool bypass, uint8_t multiplier, uint8_t sys_div, uint8_t pre_div, bool root_2x, uint8_t seld5, bool pclk_manual, uint8_t pclk_div){
    int ret = 0;
    if(multiplier > 31 || sys_div > 15 || pre_div > 3 || pclk_div > 31 || seld5 > 3){
        ESP_LOGE(TAG, "Invalid arguments");
        return -1;
    }

    calc_sysclk(sensor->xclk_freq_hz, bypass, multiplier, sys_div, pre_div, root_2x, seld5, pclk_manual, pclk_div);

    ret = write_reg(sensor->slv_addr, SC_PLLS_CTRL0, bypass?0x80:0x00);
    if (ret == 0) {
        ret = write_reg(sensor->slv_addr, SC_PLLS_CTRL1, multiplier & 0x1f);
    }
    if (ret == 0) {
        ret = write_reg(sensor->slv_addr, SC_PLLS_CTRL2, 0x10 | (sys_div & 0x0f));
    }
    if (ret == 0) {
        ret = write_reg(sensor->slv_addr, SC_PLLS_CTRL3, (pre_div & 0x3) << 4 | seld5 | (root_2x?0x40:0x00));
    }
    if (ret == 0) {
        ret = write_reg(sensor->slv_addr, PCLK_RATIO, pclk_div & 0x1f);
    }
    if (ret == 0) {
        ret = write_reg(sensor->slv_addr, VFIFO_CTRL0C, pclk_manual?0x22:0x20);
    }
    if(ret){
        ESP_LOGE(TAG, "set_sensor_pll FAILED!");
    }
    return ret;
}

static int set_ae_level(sensor_t *sensor, int level);

/** OV3640 mirror/flip via 307c/3090/3023 (3820/3821 does not apply). */
static int ov3640_apply_mirror_flip(sensor_t *sensor)
{
    uint8_t reg307c, reg3090, reg3023;

    if (sensor->status.vflip && sensor->status.hmirror) {
        reg307c = 0x13;
        reg3023 = 0x09;
        reg3090 = 0xc8;
    } else if (sensor->status.vflip) {
        reg307c = 0x11;
        reg3023 = 0x09;
        reg3090 = 0xc0;
    } else if (sensor->status.hmirror) {
        reg307c = 0x12;
        reg3023 = 0x0a;
        reg3090 = 0xc8;
    } else {
        reg307c = 0x10;
        reg3023 = 0x0a;
        reg3090 = 0xc0;
    }

    if (write_reg(sensor->slv_addr, OV3640_TIMING_CTRL, reg307c)
        || write_reg(sensor->slv_addr, OV3640_ARRAY_CTRL, reg3090)
        || write_reg(sensor->slv_addr, OV3640_ARRAY_VSTART, reg3023)) {
        ESP_LOGE(TAG, "Mirror/flip failed (307c=0x%02x)", reg307c);
        return -1;
    }
    ESP_LOGI(TAG, "Mirror/flip: hmirror=%d vflip=%d (307c=0x%02x)",
             sensor->status.hmirror, sensor->status.vflip, reg307c);
    return 0;
}

/** 0x30A9 PWCOM1 bit3: bypass on-chip regulator / external power path. */
static int ov3640_enable_regulator_bypass(sensor_t *sensor)
{
    int ret = read_reg(sensor->slv_addr, PWCOM1);
    if (ret < 0) {
        ESP_LOGE(TAG, "Read PWCOM1(0x30A9) failed");
        return ret;
    }
    uint8_t val = (uint8_t)ret;
    uint8_t new_val = val | 0x08;
    if (new_val == val) {
        ESP_LOGD(TAG, "PWCOM1 already in external power mode: 0x%02x", val);
        return 0;
    }
    ret = write_reg(sensor->slv_addr, PWCOM1, new_val);
    if (ret == 0) {
        ESP_LOGI(TAG, "PWCOM1 external power: 0x%02x -> 0x%02x", val, new_val);
    }
    return ret;
}

/**
 * Module vendor power-save: disable embedded MCU and EIS.
 * Register sequence is written in the JPEG/MCU bank.
 */
static int ov3640_apply_vendor_power_save(sensor_t *sensor)
{
    static const struct {
        uint16_t reg;
        uint8_t val;
    } mcu_regs[] = {
        { OV3640_MCU_EIS_CTRL,  0x00 },
        { OV3640_MCU_SLEEP,     0x00 },
        { OV3640_MCU_IQ_CTRL,   0x00 },
        { OV3640_MCU_FIFO_CTRL, 0x18 },
    };
    int ret;

    ret = write_reg(sensor->slv_addr, OV3640_BANK_SEL, OV3640_BANK_JPEG_MCU);
    if (ret) {
        ESP_LOGE(TAG, "Enter JPEG/MCU bank failed");
        return ret;
    }

    for (size_t i = 0; i < sizeof(mcu_regs) / sizeof(mcu_regs[0]); i++) {
        ret = write_reg(sensor->slv_addr, mcu_regs[i].reg, mcu_regs[i].val);
        if (ret) {
            ESP_LOGE(TAG, "Vendor reg 0x%04x write failed", mcu_regs[i].reg);
            return ret;
        }
    }

    ret = write_reg(sensor->slv_addr, OV3640_BANK_SEL, OV3640_BANK_DEFAULT);
    if (ret) {
        ESP_LOGE(TAG, "Leave JPEG/MCU bank failed");
        return ret;
    }

    ESP_LOGI(TAG, "Vendor power save applied (MCU sleep, EIS off)");
    return 0;
}

static int ov3640_apply_vendor_tuning(sensor_t *sensor)
{
    int ret = ov3640_enable_regulator_bypass(sensor);
    if (ret == 0) {
        ret = ov3640_apply_vendor_power_save(sensor);
    }
    return ret;
}

/** Enable SDE block (required before brightness/contrast/saturation/effects). */
static int ov3640_enable_sde(sensor_t *sensor)
{
    return write_reg(sensor->slv_addr, OV3640_SDE_CTRL, 0xef);
}

/** Init Simple AWB and defect-pixel correction (Application Notes). */
static int ov3640_init_isp_defaults(sensor_t *sensor)
{
    int ret = write_reg(sensor->slv_addr, OV3640_DPC_CTRL, 0xde);
    if (ret == 0) {
        ret = write_reg(sensor->slv_addr, OV3640_SIMPLE_AWB, 0xa5);
    }
    if (ret == 0) {
        ret = write_reg(sensor->slv_addr, OV3640_AWB_CTRL, 0x00);
    }
    if (ret == 0) {
        ESP_LOGI(TAG, "ISP defaults: Simple AWB + DPC enabled");
    }
    return ret;
}

static int reset(sensor_t *sensor)
{
    int ret = 0;
    // Software Reset: clear all registers and reset them to their default values
    ret = write_reg(sensor->slv_addr, SYSTEM_CTROL0, 0x82);
    if(ret){
        ESP_LOGE(TAG, "Software Reset FAILED!");
        return ret;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ret = write_regs(sensor->slv_addr, sensor_default_regs);
    if (ret == 0) {
        ESP_LOGD(TAG, "Camera defaults loaded");
        ret = ov3640_apply_vendor_tuning(sensor);
        if (ret == 0) {
            ret = ov3640_init_isp_defaults(sensor);
        }
        if (ret == 0) {
            ret = set_ae_level(sensor, 0);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    return ret;
}

static int set_pixformat(sensor_t *sensor, pixformat_t pixformat)
{
    int ret = 0;
    const uint16_t (*regs)[2];

    switch (pixformat) {
    case PIXFORMAT_YUV422:
        regs = sensor_fmt_yuv422;
        break;

    case PIXFORMAT_GRAYSCALE:
        regs = sensor_fmt_grayscale;
        break;

    case PIXFORMAT_RGB565:
    case PIXFORMAT_RGB888:
        regs = sensor_fmt_rgb565;
        break;

    case PIXFORMAT_JPEG:
        regs = sensor_fmt_jpeg;
        break;

    case PIXFORMAT_RAW:
        regs = sensor_fmt_raw;
        break;

    default:
        ESP_LOGE(TAG, "Unsupported pixformat: %u", pixformat);
        return -1;
    }

    ret = write_regs(sensor->slv_addr, regs);
    if(ret == 0) {
        sensor->pixformat = pixformat;
        ESP_LOGD(TAG, "Set pixformat to: %u", pixformat);
    }
    return ret;
}

static int set_image_options(sensor_t *sensor)
{
    int ret = 0;
    uint8_t reg20 = 0;
    uint8_t reg21 = 0;
    uint8_t reg4514 = 0;
    uint8_t reg4514_test = 0;

    // compression
    if (sensor->pixformat == PIXFORMAT_JPEG) {
        reg21 |= 0x20;
    }

    // binning
    if (sensor->status.binning) {
        reg20 |= 0x01;
        reg21 |= 0x01;
        reg4514_test |= 4;
    } else {
        reg20 |= 0x40;
    }

    // Mirror/flip is applied via ov3640_apply_mirror_flip() / 307c.
    if (sensor->status.vflip) {
        reg4514_test |= 1;
    }
    if (sensor->status.hmirror) {
        reg4514_test |= 2;
    }

    switch (reg4514_test) {
        //no binning
        case 0: reg4514 = 0x88; break;//normal
        case 1: reg4514 = 0x88; break;//v-flip
        case 2: reg4514 = 0xbb; break;//h-mirror
        case 3: reg4514 = 0xbb; break;//v-flip+h-mirror
        //binning
        case 4: reg4514 = 0xaa; break;//normal
        case 5: reg4514 = 0xbb; break;//v-flip
        case 6: reg4514 = 0xbb; break;//h-mirror
        case 7: reg4514 = 0xaa; break;//v-flip+h-mirror
    }

    if(write_reg(sensor->slv_addr, TIMING_TC_REG20, reg20)
        || write_reg(sensor->slv_addr, TIMING_TC_REG21, reg21)
        || write_reg(sensor->slv_addr, 0x4514, reg4514)){
        ESP_LOGE(TAG, "Setting Image Options Failed");
        ret = -1;
    }

    if (sensor->status.binning) {
        ret  = write_reg(sensor->slv_addr, 0x4520, 0x0b)
            || write_reg(sensor->slv_addr, X_INCREMENT, 0x31)//odd:3, even: 1
            || write_reg(sensor->slv_addr, Y_INCREMENT, 0x31);//odd:3, even: 1
    } else {
        ret  = write_reg(sensor->slv_addr, 0x4520, 0xb0)
            || write_reg(sensor->slv_addr, X_INCREMENT, 0x11)//odd:1, even: 1
            || write_reg(sensor->slv_addr, Y_INCREMENT, 0x11);//odd:1, even: 1
    }

    if (ret == 0) {
        ret = ov3640_apply_mirror_flip(sensor);
    }

    ESP_LOGD(TAG, "Set Image Options: Compression: %u, Binning: %u, V-Flip: %u, H-Mirror: %u, Reg-4514: 0x%02x",
        sensor->pixformat == PIXFORMAT_JPEG, sensor->status.binning, sensor->status.vflip, sensor->status.hmirror, reg4514);
    return ret;
}

static int set_framesize(sensor_t *sensor, framesize_t framesize)
{
    int ret = 0;

    if(framesize > FRAMESIZE_QXGA){
        ESP_LOGW(TAG, "Invalid framesize: %u", framesize);
        framesize = FRAMESIZE_QXGA;
    }
    framesize_t old_framesize = sensor->status.framesize;
    sensor->status.framesize = framesize;
    uint16_t w = resolution[framesize].width;
    uint16_t h = resolution[framesize].height;
    aspect_ratio_t ratio = resolution[sensor->status.framesize].aspect_ratio;
    ratio_settings_t settings = ratio_table[ratio];

    sensor->status.binning = (w <= (settings.max_width / 2) && h <= (settings.max_height / 2));
    sensor->status.scale = !((w == settings.max_width && h == settings.max_height)
        || (w == (settings.max_width / 2) && h == (settings.max_height / 2)));

    ret  = write_addr_reg(sensor->slv_addr, X_ADDR_ST_H, settings.start_x, settings.start_y)
        || write_addr_reg(sensor->slv_addr, X_ADDR_END_H, settings.end_x, settings.end_y)
        || write_addr_reg(sensor->slv_addr, X_OUTPUT_SIZE_H, w, h);

    if (ret) {
        goto fail;
    }

    if (sensor->status.binning) {
        ret  = write_addr_reg(sensor->slv_addr, X_TOTAL_SIZE_H, settings.total_x, (settings.total_y / 2) + 1)
            || write_addr_reg(sensor->slv_addr, X_OFFSET_H, 8, 2);
    } else {
        ret  = write_addr_reg(sensor->slv_addr, X_TOTAL_SIZE_H, settings.total_x, settings.total_y)
            || write_addr_reg(sensor->slv_addr, X_OFFSET_H, 16, 6);
    }

    if (ret == 0) {
        ret = write_reg_bits(sensor->slv_addr, ISP_CONTROL_01, 0x20, sensor->status.scale);
    }

    if (ret == 0) {
        ret = set_image_options(sensor);
    }

    if (ret) {
        goto fail;
    }

    if (sensor->pixformat == PIXFORMAT_JPEG) {
        if (framesize == FRAMESIZE_QXGA || sensor->xclk_freq_hz == 16000000) {
            //40MHz SYSCLK and 10MHz PCLK
            ret = set_pll(sensor, false, 24, 1, 3, false, 0, true, 8);
        } else {
            //50MHz SYSCLK and 10MHz PCLK
            ret = set_pll(sensor, false, 30, 1, 3, false, 0, true, 10);
        }
    } else {
        //tuned for 16MHz XCLK and 8MHz PCLK
        if (framesize > FRAMESIZE_HVGA) {
            //8MHz SYSCLK and 8MHz PCLK (4.44 FPS)
            ret = set_pll(sensor, false, 4, 1, 0, false, 2, true, 2);
        } else if (framesize >= FRAMESIZE_QVGA) {
            //16MHz SYSCLK and 8MHz PCLK (10.25 FPS)
            ret = set_pll(sensor, false, 8, 1, 0, false, 2, true, 4);
        } else {
            //32MHz SYSCLK and 8MHz PCLK (17.77 FPS)
            ret = set_pll(sensor, false, 8, 1, 0, false, 0, true, 8);
        }
    }

    if (ret == 0) {
        ESP_LOGD(TAG, "Set framesize to: %ux%u", w, h);
    }
    return ret;

fail:
    sensor->status.framesize = old_framesize;
    ESP_LOGE(TAG, "Setting framesize to: %ux%u failed", w, h);
    return ret;
}

static int set_hmirror(sensor_t *sensor, int enable)
{
    int ret = 0;
    sensor->status.hmirror = enable;
    ret = set_image_options(sensor);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set h-mirror to: %d", enable);
    }
    return ret;
}

static int set_vflip(sensor_t *sensor, int enable)
{
    int ret = 0;
    sensor->status.vflip = enable;
    ret = set_image_options(sensor);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set v-flip to: %d", enable);
    }
    return ret;
}

static int set_quality(sensor_t *sensor, int qs)
{
    int ret = 0;
    ret = write_reg(sensor->slv_addr, COMPRESSION_CTRL07, qs & 0x3f);
    if (ret == 0) {
        sensor->status.quality = qs;
        ESP_LOGD(TAG, "Set quality to: %d", qs);
    }
    return ret;
}

static int set_colorbar(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = write_reg_bits(sensor->slv_addr, PRE_ISP_TEST_SETTING_1, TEST_COLOR_BAR, enable);
    if (ret == 0) {
        sensor->status.colorbar = enable;
        ESP_LOGD(TAG, "Set colorbar to: %d", enable);
    }
    return ret;
}

static int set_gain_ctrl(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = write_reg_bits(sensor->slv_addr, AEC_PK_MANUAL, AEC_PK_MANUAL_AGC_MANUALEN, !enable);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set gain_ctrl to: %d", enable);
        sensor->status.agc = enable;
    }
    return ret;
}

static int set_exposure_ctrl(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = write_reg_bits(sensor->slv_addr, AEC_PK_MANUAL, AEC_PK_MANUAL_AEC_MANUALEN, !enable);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set exposure_ctrl to: %d", enable);
        sensor->status.aec = enable;
    }
    return ret;
}

static int set_whitebal(sensor_t *sensor, int enable)
{
    int ret;
    if (enable) {
        ret = write_reg(sensor->slv_addr, OV3640_AWB_CTRL, 0x00);
    } else {
        ret = write_reg(sensor->slv_addr, OV3640_AWB_CTRL, 0x08);
    }
    if (ret == 0) {
        ESP_LOGD(TAG, "Set awb to: %d", enable);
        sensor->status.awb = enable;
    }
    return ret;
}

//Advanced AWB
static int set_dcw_dsp(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = write_reg_bits(sensor->slv_addr, 0x5183, 0x80, !enable);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set dcw to: %d", enable);
        sensor->status.dcw = enable;
    }
    return ret;
}

//night mode enable
static int set_aec2(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = write_reg_bits(sensor->slv_addr, 0x3a00, 0x04, enable);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set aec2 to: %d", enable);
        sensor->status.aec2 = enable;
    }
    return ret;
}

static int set_bpc_dsp(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = write_reg_bits(sensor->slv_addr, 0x5000, 0x04, enable);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set bpc to: %d", enable);
        sensor->status.bpc = enable;
    }
    return ret;
}

static int set_wpc_dsp(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = write_reg_bits(sensor->slv_addr, 0x5000, 0x02, enable);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set wpc to: %d", enable);
        sensor->status.wpc = enable;
    }
    return ret;
}

//Gamma enable
static int set_raw_gma_dsp(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = write_reg_bits(sensor->slv_addr, 0x5000, 0x20, enable);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set raw_gma to: %d", enable);
        sensor->status.raw_gma = enable;
    }
    return ret;
}

static int set_lenc_dsp(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = write_reg_bits(sensor->slv_addr, 0x5000, 0x80, enable);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set lenc to: %d", enable);
        sensor->status.lenc = enable;
    }
    return ret;
}

static int get_agc_gain(sensor_t *sensor)
{
    int ra = read_reg(sensor->slv_addr, 0x350a);
    if (ra < 0) {
        return 0;
    }
    int rb = read_reg(sensor->slv_addr, 0x350b);
    if (rb < 0) {
        return 0;
    }
    int res = (rb & 0xF0) >> 4 | (ra & 0x03) << 4;
    if (rb & 0x0F) {
        res += 1;
    }
    return res;
}

//real gain
static int set_agc_gain(sensor_t *sensor, int gain)
{
    int ret = 0;
    if(gain < 0) {
        gain = 0;
    } else if(gain > 64) {
        gain = 64;
    }

    //gain value is 6.4 bits float
    //in order to use the max range, we deduct 1/16
    int gainv = gain << 4;
    if(gainv){
        gainv -= 1;
    }

    ret = write_reg(sensor->slv_addr, 0x350a, gainv >> 8) || write_reg(sensor->slv_addr, 0x350b, gainv & 0xff);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set agc_gain to: %d", gain);
        sensor->status.agc_gain = gain;
    }
    return ret;
}

static int get_aec_value(sensor_t *sensor)
{
    int ra = read_reg(sensor->slv_addr, 0x3500);
    if (ra < 0) {
        return 0;
    }
    int rb = read_reg(sensor->slv_addr, 0x3501);
    if (rb < 0) {
        return 0;
    }
    int rc = read_reg(sensor->slv_addr, 0x3502);
    if (rc < 0) {
        return 0;
    }
    int res = (ra & 0x0F) << 12 | (rb & 0xFF) << 4 | (rc & 0xF0) >> 4;
    return res;
}

static int set_aec_value(sensor_t *sensor, int value)
{
    int ret = 0, max_val = 0;
    max_val = read_reg16(sensor->slv_addr, 0x380e);
    if (max_val < 0) {
        ESP_LOGE(TAG, "Could not read max aec_value");
        return -1;
    }
    if (value > max_val) {
        value =max_val;
    }

    ret =  write_reg(sensor->slv_addr, 0x3500, (value >> 12) & 0x0F)
        || write_reg(sensor->slv_addr, 0x3501, (value >> 4) & 0xFF)
        || write_reg(sensor->slv_addr, 0x3502, (value << 4) & 0xF0);

    if (ret == 0) {
        ESP_LOGD(TAG, "Set aec_value to: %d / %d", value, max_val);
        sensor->status.aec_value = value;
    }
    return ret;
}

static int set_ae_level(sensor_t *sensor, int level)
{
    if (level < -5 || level > 5) {
        return -1;
    }
    const ov3640_ae_entry_t *entry = &ov3640_ae_levels[level + 5];
    int ret = write_reg(sensor->slv_addr, OV3640_AEC_ALGO, 0x00);
    if (ret == 0) {
        ret = write_reg(sensor->slv_addr, OV3640_AEC_18, entry->reg3018);
    }
    if (ret == 0) {
        ret = write_reg(sensor->slv_addr, OV3640_AEC_19, entry->reg3019);
    }
    if (ret == 0) {
        ret = write_reg(sensor->slv_addr, OV3640_AEC_1A, entry->reg301a);
    }
    if (ret == 0) {
        ESP_LOGD(TAG, "Set ae_level to: %d", level);
        sensor->status.ae_level = level;
    }
    return ret;
}

static int set_wb_mode(sensor_t *sensor, int mode)
{
    int ret = 0;
    if (mode < 0 || mode > 4) {
        return -1;
    }

    if (mode == 0) {
        ret = write_reg(sensor->slv_addr, OV3640_AWB_CTRL, 0x00);
    } else {
        const ov3640_wb_preset_t *p = &ov3640_wb_presets[mode];
        ret = write_reg(sensor->slv_addr, OV3640_AWB_CTRL, 0x08);
        if (ret == 0) {
            ret = write_reg(sensor->slv_addr, OV3640_AWB_R_GAIN, p->awb_r);
        }
        if (ret == 0) {
            ret = write_reg(sensor->slv_addr, OV3640_AWB_G_GAIN, p->awb_g);
        }
        if (ret == 0) {
            ret = write_reg(sensor->slv_addr, OV3640_AWB_B_GAIN, p->awb_b);
        }
    }

    if (ret == 0) {
        ESP_LOGD(TAG, "Set wb_mode to: %d", mode);
        sensor->status.wb_mode = mode;
        sensor->status.awb = (mode == 0);
    }
    return ret;
}

static int set_awb_gain_dsp(sensor_t *sensor, int enable)
{
    int ret = 0;
    int old_mode = sensor->status.wb_mode;
    int mode = enable?old_mode:0;

    ret = set_wb_mode(sensor, mode);

    if (ret == 0) {
        sensor->status.wb_mode = old_mode;
        ESP_LOGD(TAG, "Set awb_gain to: %d", enable);
        sensor->status.awb_gain = enable;
    }
    return ret;
}

static int set_special_effect(sensor_t *sensor, int effect)
{
    int ret = 0;
    if (effect < 0 || effect > 6) {
        return -1;
    }

    const ov3640_effect_entry_t *e = &ov3640_special_effects[effect];
    ret = ov3640_enable_sde(sensor);
    if (ret == 0) {
        ret = write_reg(sensor->slv_addr, OV3640_SDE_ENABLE, e->reg3355);
    }
    if (ret == 0 && e->reg3355 == 0x18) {
        ret = write_reg(sensor->slv_addr, OV3640_EFFECT_C1, e->reg335a)
           || write_reg(sensor->slv_addr, OV3640_EFFECT_C2, e->reg335b);
    }

    if (ret == 0) {
        ESP_LOGD(TAG, "Set special_effect to: %d", effect);
        sensor->status.special_effect = effect;
    }
    return ret;
}

static int set_brightness(sensor_t *sensor, int level)
{
    int ret = 0;
    if (level < -3 || level > 3) {
        return -1;
    }

    const ov3640_brightness_entry_t *b = &ov3640_brightness_levels[level + 3];
    ret = ov3640_enable_sde(sensor);
    if (ret == 0) {
        ret = write_reg(sensor->slv_addr, OV3640_SDE_ENABLE, 0x04);
    }
    if (ret == 0) {
        ret = write_reg(sensor->slv_addr, OV3640_SDE_OFFSET, b->reg3354);
    }
    if (ret == 0) {
        ret = write_reg(sensor->slv_addr, OV3640_BRIGHTNESS, b->reg335e);
    }

    if (ret == 0) {
        ESP_LOGD(TAG, "Set brightness to: %d", level);
        sensor->status.brightness = level;
    }
    return ret;
}

static int set_contrast(sensor_t *sensor, int level)
{
    int ret = 0;
    if (level < -3 || level > 3) {
        return -1;
    }

    const ov3640_contrast_entry_t *c = &ov3640_contrast_levels[level + 3];
    ret = ov3640_enable_sde(sensor);
    if (ret == 0) {
        ret = write_reg(sensor->slv_addr, OV3640_SDE_ENABLE, 0x04);
    }
    if (ret == 0) {
        ret = write_reg(sensor->slv_addr, OV3640_SDE_OFFSET, 0x01);
    }
    if (ret == 0) {
        ret = write_reg(sensor->slv_addr, OV3640_CONTRAST_Y1, c->y1)
           || write_reg(sensor->slv_addr, OV3640_CONTRAST_Y2, c->y2);
    }

    if (ret == 0) {
        ESP_LOGD(TAG, "Set contrast to: %d", level);
        sensor->status.contrast = level;
    }
    return ret;
}

static int set_saturation(sensor_t *sensor, int level)
{
    int ret = 0;
    if (level < -4 || level > 4) {
        return -1;
    }
    if (level < -2) {
        level = -2;
    } else if (level > 2) {
        level = 2;
    }

    const uint8_t *s = ov3640_saturation_levels[level + 2];
    ret = ov3640_enable_sde(sensor);
    if (ret == 0) {
        ret = write_reg(sensor->slv_addr, OV3640_SDE_ENABLE, 0x02);
    }
    if (ret == 0) {
        ret = write_reg(sensor->slv_addr, OV3640_SATURATION_C, s[0])
           || write_reg(sensor->slv_addr, OV3640_SATURATION_M, s[1]);
    }

    if (ret == 0) {
        ESP_LOGD(TAG, "Set saturation to: %d", level);
        sensor->status.saturation = level;
    }
    return ret;
}

static int set_sharpness(sensor_t *sensor, int level)
{
    int ret = 0;
    if (level < -3 || level > 3) {
        return -1;
    }

    uint8_t sharp = ov3640_sharpness_levels[level + 3];
    ret = write_reg(sensor->slv_addr, OV3640_SHARPNESS, sharp);
    if (ret == 0 && sharp == 0x60) {
        ret = write_reg(sensor->slv_addr, OV3640_SHARPNESS_AUTO, 0x03);
    }

    if (ret == 0) {
        ESP_LOGD(TAG, "Set sharpness to: %d (332d=0x%02x)", level, sharp);
        sensor->status.sharpness = level;
    }
    return ret;
}

static int set_gainceiling(sensor_t *sensor, gainceiling_t level)
{
    int ret = 0, l = (int)level;

    ret = write_reg(sensor->slv_addr, 0x3A18, (l >> 8) & 3)
       || write_reg(sensor->slv_addr, 0x3A19, l & 0xFF);

    if (ret == 0) {
        ESP_LOGD(TAG, "Set gainceiling to: %d", l);
        sensor->status.gainceiling = l;
    }
    return ret;
}

static int get_denoise(sensor_t *sensor)
{
    if (!check_reg_mask(sensor->slv_addr, 0x5308, 0x10)) {
        return 0;
    }
    return (read_reg(sensor->slv_addr, 0x5306) / 4) + 1;
}

static int set_denoise(sensor_t *sensor, int level)
{
    int ret = 0;
    if (level < 0 || level > 8) {
        return -1;
    }

    ret = write_reg_bits(sensor->slv_addr, 0x5308, 0x10, level > 0);
    if (ret == 0 && level > 0) {
        ret = write_reg(sensor->slv_addr, 0x5306, (level - 1) * 4);
    }

    if (ret == 0) {
        ESP_LOGD(TAG, "Set denoise to: %d", level);
        sensor->status.denoise = level;
    }
    return ret;
}

static int get_reg(sensor_t *sensor, int reg, int mask)
{
    int ret = 0, ret2 = 0;
    if(mask > 0xFF){
        ret = read_reg16(sensor->slv_addr, reg);
        if(ret >= 0 && mask > 0xFFFF){
            ret2 = read_reg(sensor->slv_addr, reg+2);
            if(ret2 >= 0){
                ret = (ret << 8) | ret2 ;
            } else {
                ret = ret2;
            }
        }
    } else {
        ret = read_reg(sensor->slv_addr, reg);
    }
    if(ret > 0){
        ret &= mask;
    }
    return ret;
}

static int set_reg(sensor_t *sensor, int reg, int mask, int value)
{
    int ret = 0, ret2 = 0;
    if(mask > 0xFF){
        ret = read_reg16(sensor->slv_addr, reg);
        if(ret >= 0 && mask > 0xFFFF){
            ret2 = read_reg(sensor->slv_addr, reg+2);
            if(ret2 >= 0){
                ret = (ret << 8) | ret2 ;
            } else {
                ret = ret2;
            }
        }
    } else {
        ret = read_reg(sensor->slv_addr, reg);
    }
    if(ret < 0){
        return ret;
    }
    value = (ret & ~mask) | (value & mask);
    if(mask > 0xFFFF){
        ret = write_reg16(sensor->slv_addr, reg, value >> 8);
        if(ret >= 0){
            ret = write_reg(sensor->slv_addr, reg+2, value & 0xFF);
        }
    } else if(mask > 0xFF){
        ret = write_reg16(sensor->slv_addr, reg, value);
    } else {
        ret = write_reg(sensor->slv_addr, reg, value);
    }
    return ret;
}

static int set_res_raw(sensor_t *sensor, int startX, int startY, int endX, int endY, int offsetX, int offsetY, int totalX, int totalY, int outputX, int outputY, bool scale, bool binning)
{
    int ret = 0;
    ret  = write_addr_reg(sensor->slv_addr, X_ADDR_ST_H, startX, startY)
        || write_addr_reg(sensor->slv_addr, X_ADDR_END_H, endX, endY)
        || write_addr_reg(sensor->slv_addr, X_OFFSET_H, offsetX, offsetY)
        || write_addr_reg(sensor->slv_addr, X_TOTAL_SIZE_H, totalX, totalY)
        || write_addr_reg(sensor->slv_addr, X_OUTPUT_SIZE_H, outputX, outputY)
        || write_reg_bits(sensor->slv_addr, ISP_CONTROL_01, 0x20, scale);
    if(!ret){
        sensor->status.scale = scale;
        sensor->status.binning = binning;
        ret = set_image_options(sensor);
    }
    return ret;
}

static int _set_pll(sensor_t *sensor, int bypass, int multiplier, int sys_div, int root_2x, int pre_div, int seld5, int pclk_manual, int pclk_div)
{
    return set_pll(sensor, bypass > 0, multiplier, sys_div, pre_div, root_2x > 0, seld5, pclk_manual > 0, pclk_div);
}

static int set_xclk(sensor_t *sensor, int timer, int xclk)
{
    int ret = 0;
    sensor->xclk_freq_hz = xclk * 1000000U;
    ret = xclk_timer_conf(timer, sensor->xclk_freq_hz);
    return ret;
}

static int init_status(sensor_t *sensor)
{
    sensor->status.brightness = 0;
    sensor->status.contrast = 0;
    sensor->status.saturation = 0;
    sensor->status.sharpness = 0;
    {
        int sharp = read_reg(sensor->slv_addr, OV3640_SHARPNESS);
        if (sharp >= 0x41 && sharp <= 0x48) {
            sensor->status.sharpness = sharp - 0x43;
        }
    }
    sensor->status.denoise = get_denoise(sensor);
    sensor->status.ae_level = 0;
    sensor->status.gainceiling = read_reg16(sensor->slv_addr, 0x3A18) & 0x3FF;
    sensor->status.awb = check_reg_mask(sensor->slv_addr, ISP_CONTROL_01, 0x01);
    sensor->status.dcw = !check_reg_mask(sensor->slv_addr, 0x5183, 0x80);
    sensor->status.agc = !check_reg_mask(sensor->slv_addr, AEC_PK_MANUAL, AEC_PK_MANUAL_AGC_MANUALEN);
    sensor->status.aec = !check_reg_mask(sensor->slv_addr, AEC_PK_MANUAL, AEC_PK_MANUAL_AEC_MANUALEN);
    sensor->status.hmirror = false;
    sensor->status.vflip = false;
    {
        int mode = read_reg(sensor->slv_addr, OV3640_TIMING_CTRL);
        if (mode >= 0) {
            switch (mode & 0x1f) {
            case 0x11:
                sensor->status.vflip = true;
                break;
            case 0x12:
                sensor->status.hmirror = true;
                break;
            case 0x13:
                sensor->status.vflip = true;
                sensor->status.hmirror = true;
                break;
            default:
                break;
            }
        }
    }
    sensor->status.colorbar = check_reg_mask(sensor->slv_addr, PRE_ISP_TEST_SETTING_1, TEST_COLOR_BAR);
    sensor->status.bpc = check_reg_mask(sensor->slv_addr, 0x5000, 0x04);
    sensor->status.wpc = check_reg_mask(sensor->slv_addr, 0x5000, 0x02);
    sensor->status.raw_gma = check_reg_mask(sensor->slv_addr, 0x5000, 0x20);
    sensor->status.lenc = check_reg_mask(sensor->slv_addr, 0x5000, 0x80);
    sensor->status.quality = read_reg(sensor->slv_addr, COMPRESSION_CTRL07) & 0x3f;
    sensor->status.special_effect = 0;
    sensor->status.wb_mode = 0;
    sensor->status.awb = !check_reg_mask(sensor->slv_addr, OV3640_AWB_CTRL, 0x08);
    sensor->status.awb_gain = (sensor->status.wb_mode != 0);
    sensor->status.agc_gain = get_agc_gain(sensor);
    sensor->status.aec_value = get_aec_value(sensor);
    sensor->status.aec2 = check_reg_mask(sensor->slv_addr, 0x3a00, 0x04);
    return 0;
}

int esp32_camera_ov3640_detect(int slv_addr, sensor_id_t *id)
{
    if (OV3640_SCCB_ADDR == slv_addr) {
        uint8_t h = SCCB_Read16(slv_addr, 0x300A);
        uint8_t l = SCCB_Read16(slv_addr, 0x300B);
        uint16_t PID = (h << 8) | l;
        /* OV3640 family PID: 0x3640, 0x364C, etc. (upper 12 bits == 0x364) */
        if ((PID & 0xFFF0) == 0x3640) {
            id->PID = OV3640_PID;
            ESP_LOGI(TAG, "Detected OV3640 family PID=0x%x", PID);
            return OV3640_PID;
        } else {
            ESP_LOGI(TAG, "Mismatch PID=0x%x", PID);
        }
    }
    return 0;
}

int esp32_camera_ov3640_init(sensor_t *sensor)
{
    sensor->reset = reset;
    sensor->set_pixformat = set_pixformat;
    sensor->set_framesize = set_framesize;
    sensor->set_contrast = set_contrast;
    sensor->set_brightness = set_brightness;
    sensor->set_saturation = set_saturation;
    sensor->set_sharpness = set_sharpness;
    sensor->set_gainceiling = set_gainceiling;
    sensor->set_quality = set_quality;
    sensor->set_colorbar = set_colorbar;
    sensor->set_gain_ctrl = set_gain_ctrl;
    sensor->set_exposure_ctrl = set_exposure_ctrl;
    sensor->set_whitebal = set_whitebal;
    sensor->set_hmirror = set_hmirror;
    sensor->set_vflip = set_vflip;
    sensor->init_status = init_status;
    sensor->set_aec2 = set_aec2;
    sensor->set_aec_value = set_aec_value;
    sensor->set_special_effect = set_special_effect;
    sensor->set_wb_mode = set_wb_mode;
    sensor->set_ae_level = set_ae_level;
    sensor->set_dcw = set_dcw_dsp;
    sensor->set_bpc = set_bpc_dsp;
    sensor->set_wpc = set_wpc_dsp;
    sensor->set_awb_gain = set_awb_gain_dsp;
    sensor->set_agc_gain = set_agc_gain;
    sensor->set_raw_gma = set_raw_gma_dsp;
    sensor->set_lenc = set_lenc_dsp;
    sensor->set_denoise = set_denoise;

    sensor->get_reg = get_reg;
    sensor->set_reg = set_reg;
    sensor->set_res_raw = set_res_raw;
    sensor->set_pll = _set_pll;
    sensor->set_xclk = set_xclk;

    // No autofocus support
    sensor->af_is_supported = NULL;
    sensor->af_init = NULL;
    sensor->af_set_mode = NULL;
    sensor->af_trigger = NULL;
    sensor->af_get_status = NULL;
    sensor->af_set_manual_position = NULL;

    return 0;
}
