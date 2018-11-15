/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * OV2640 driver.
 *
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "sccb.h"
#include "ov2640.h"
#include "ov2640_regs.h"
#include "ov2640_settings.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
static const char* TAG = "ov2640";
#endif

static volatile ov2640_bank_t reg_bank = BANK_MAX;
static int set_bank(sensor_t *sensor, ov2640_bank_t bank)
{
    int res = 0;
    if (bank != reg_bank) {
        reg_bank = bank;
        res = SCCB_Write(sensor->slv_addr, BANK_SEL, bank);
    }
    return res;
}

static int write_regs(sensor_t *sensor, const uint8_t (*regs)[2])
{
    int i=0, res = 0;
    while (regs[i][0]) {
        if (regs[i][0] == BANK_SEL) {
            res = set_bank(sensor, regs[i][1]);
        } else {
            res = SCCB_Write(sensor->slv_addr, regs[i][0], regs[i][1]);
        }
        if (res) {
            return res;
        }
        i++;
    }
    return res;
}

static int write_reg(sensor_t *sensor, ov2640_bank_t bank, uint8_t reg, uint8_t value)
{
    int ret = set_bank(sensor, bank);
    if(!ret) {
        ret = SCCB_Write(sensor->slv_addr, reg, value);
    }
    return ret;
}

static int set_reg_bits(sensor_t *sensor, uint8_t bank, uint8_t reg, uint8_t offset, uint8_t mask, uint8_t value)
{
    int ret = 0;
    uint8_t c_value, new_value;

    ret = set_bank(sensor, bank);
    if(ret) {
        return ret;
    }
    c_value = SCCB_Read(sensor->slv_addr, reg);
    new_value = (c_value & ~(mask << offset)) | ((value & mask) << offset);
    ret = SCCB_Write(sensor->slv_addr, reg, new_value);
    return ret;
}

//Function is not needed currently
#if 0
static uint8_t get_reg_bits(sensor_t *sensor, uint8_t bank, uint8_t reg, uint8_t offset, uint8_t mask)
{
    uint8_t ret = 0;
    if(set_bank(sensor, bank)){
        return ret;
    }
    ret = (SCCB_Read(sensor->slv_addr, reg) >> offset) & mask;
    return ret;
}
#endif

static int write_reg_bits(sensor_t *sensor, uint8_t bank, uint8_t reg, uint8_t mask, int enable)
{
    return set_reg_bits(sensor, bank, reg, 0, mask, enable?mask:0);
}

#define WRITE_REGS_OR_RETURN(regs) ret = write_regs(sensor, regs); if(ret){return ret;}
#define WRITE_REG_OR_RETURN(bank, reg, val) ret = write_reg(sensor, bank, reg, val); if(ret){return ret;}
#define SET_REG_BITS_OR_RETURN(bank, reg, offset, mask, val) ret = set_reg_bits(sensor, bank, reg, offset, mask, val); if(ret){return ret;}

static int reset(sensor_t *sensor)
{
    int ret = 0;
    WRITE_REG_OR_RETURN(BANK_SENSOR, COM7, COM7_SRST);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    WRITE_REGS_OR_RETURN(ov2640_settings_cif);
    return ret;
}

static int set_pixformat(sensor_t *sensor, pixformat_t pixformat)
{
    int ret = 0;
    sensor->pixformat = pixformat;
    switch (pixformat) {
    case PIXFORMAT_RGB565:
    case PIXFORMAT_RGB888:
        WRITE_REGS_OR_RETURN(ov2640_settings_rgb565);
        break;
    case PIXFORMAT_YUV422:
    case PIXFORMAT_GRAYSCALE:
        WRITE_REGS_OR_RETURN(ov2640_settings_yuv422);
        break;
    case PIXFORMAT_JPEG:
        WRITE_REGS_OR_RETURN(ov2640_settings_jpeg3);
        break;
    default:
        ret = -1;
        break;
    }
    if(!ret) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    return ret;
}

//Functions are not needed currently
#if 0
//Set the sensor output window
int set_output_window(sensor_t *sensor, uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
    int ret = 0;
    uint16_t endx, endy;
    uint8_t com1, reg32;

    endy = y + height / 2;
    com1 = read_reg(sensor, BANK_SENSOR, COM1);
    WRITE_REG_OR_RETURN(BANK_SENSOR, COM1, (com1 & 0XF0) | (((endy & 0X03) << 2) | (y & 0X03)));
    WRITE_REG_OR_RETURN(BANK_SENSOR, VSTART, y >> 2);
    WRITE_REG_OR_RETURN(BANK_SENSOR, VSTOP, endy >> 2);

    endx = x + width / 2;
    reg32 = read_reg(sensor, BANK_SENSOR, REG32);
    WRITE_REG_OR_RETURN(BANK_SENSOR, REG32, (reg32 & 0XC0) | (((endx & 0X07) << 3) | (x & 0X07)));
    WRITE_REG_OR_RETURN(BANK_SENSOR, HSTART, x >> 3);
    WRITE_REG_OR_RETURN(BANK_SENSOR, HSTOP, endx >> 3);

    return ret;
}

// Set the image output size (final output resolution)
int set_output_size(sensor_t *sensor, uint16_t width, uint16_t height)
{
    int ret = 0;
    uint16_t h, w;

    if(width % 4) {
        return -1;
    }
    if(height % 4 ) {
        return -2;
    }

    w = width / 4;
    h = height / 4;
    //WRITE_REG_OR_RETURN(BANK_DSP, RESET, RESET_DVP);
    WRITE_REG_OR_RETURN(BANK_DSP, ZMOW, w & 0XFF);
    WRITE_REG_OR_RETURN(BANK_DSP, ZMOH, h & 0XFF);
    WRITE_REG_OR_RETURN(BANK_DSP, ZMHH, ((w >> 8) & 0X03) | ((h >> 6) & 0X04));
    //WRITE_REG_OR_RETURN(BANK_DSP, RESET, 0X00);

    return ret;
}

//Set the image window size >= output size
int set_window_size(sensor_t *sensor, uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
    int ret = 0;
    uint16_t w, h;

    if(width % 4) {
        return -1;
    }
    if(height % 4) {
        return -2;
    }

    w = width / 4;
    h = height / 4;
    //WRITE_REG_OR_RETURN(BANK_DSP, RESET, RESET_DVP);
    WRITE_REG_OR_RETURN(BANK_DSP, HSIZE, w & 0XFF);
    WRITE_REG_OR_RETURN(BANK_DSP, VSIZE, h & 0XFF);
    WRITE_REG_OR_RETURN(BANK_DSP, XOFFL, x & 0XFF);
    WRITE_REG_OR_RETURN(BANK_DSP, YOFFL, y & 0XFF);
    WRITE_REG_OR_RETURN(BANK_DSP, VHYX, ((h >> 1) & 0X80) | ((y >> 4) & 0X70) | ((w >> 5) & 0X08) | ((x >> 8) & 0X07));
    WRITE_REG_OR_RETURN(BANK_DSP, TEST, (w >> 2) & 0X80);
    //WRITE_REG_OR_RETURN(BANK_DSP, RESET, 0X00);

    return ret;
}

//Set the sensor resolution (UXGA, SVGA, CIF)
int set_image_size(sensor_t *sensor, uint16_t width, uint16_t height)
{
    int ret = 0;
    //WRITE_REG_OR_RETURN(BANK_DSP, RESET, RESET_DVP);
    WRITE_REG_OR_RETURN(BANK_DSP, HSIZE8, (width >> 3) & 0XFF);
    WRITE_REG_OR_RETURN(BANK_DSP, VSIZE8, (height >> 3) & 0XFF);
    WRITE_REG_OR_RETURN(BANK_DSP, SIZEL, ((width & 0X07) << 3) | ((width >> 4) & 0X80) | (height & 0X07));
    //WRITE_REG_OR_RETURN(BANK_DSP, RESET, 0X00);

    return ret;
}
#endif

static int set_framesize(sensor_t *sensor, framesize_t framesize)
{
    int ret = 0;
    uint16_t w = resolution[framesize][0];
    uint16_t h = resolution[framesize][1];
    const uint8_t (*regs)[2];

    sensor->framesize = framesize;

    if (framesize <= FRAMESIZE_CIF) {
        regs = ov2640_settings_to_cif;
    } else if (framesize <= FRAMESIZE_SVGA) {
        regs = ov2640_settings_to_svga;
    } else {
        regs = ov2640_settings_to_uxga;
    }

    WRITE_REG_OR_RETURN(BANK_DSP, R_BYPASS, R_BYPASS_DSP_BYPAS);
    WRITE_REGS_OR_RETURN(regs);
    WRITE_REG_OR_RETURN(BANK_DSP, ZMOW, (w>>2)&0xFF); // OUTW[7:0] (real/4)
    WRITE_REG_OR_RETURN(BANK_DSP, ZMOH, (h>>2)&0xFF); // OUTH[7:0] (real/4)
    WRITE_REG_OR_RETURN(BANK_DSP, ZMHH, ((h>>8)&0x04)|((w>>10)&0x03)); // OUTH[8]/OUTW[9:8]
    WRITE_REG_OR_RETURN(BANK_DSP, RESET, 0x00);
    WRITE_REG_OR_RETURN(BANK_DSP, R_BYPASS, R_BYPASS_DSP_EN);

    vTaskDelay(10 / portTICK_PERIOD_MS);
    //required when changing resolution
    set_pixformat(sensor, sensor->pixformat);

    return ret;
}

static int set_contrast(sensor_t *sensor, int level)
{
    int ret=0;
    level += 3;
    if (level <= 0 || level > NUM_CONTRAST_LEVELS) {
        return -1;
    }
    for (int i=0; i<7; i++) {
        WRITE_REG_OR_RETURN(BANK_DSP, contrast_regs[0][i], contrast_regs[level][i]);
    }
    return ret;
}

static int set_brightness(sensor_t *sensor, int level)
{
    int ret=0;
    level += 3;
    if (level <= 0 || level > NUM_BRIGHTNESS_LEVELS) {
        return -1;
    }
    for (int i=0; i<5; i++) {
        WRITE_REG_OR_RETURN(BANK_DSP, brightness_regs[0][i], brightness_regs[level][i]);
    }
    return ret;
}

static int set_saturation(sensor_t *sensor, int level)
{
    int ret=0;
    level += 3;
    if (level <= 0 || level > NUM_SATURATION_LEVELS) {
        return -1;
    }
    for (int i=0; i<5; i++) {
        WRITE_REG_OR_RETURN(BANK_DSP, saturation_regs[0][i], saturation_regs[level][i]);
    }
    return ret;
}

static int set_special_effect(sensor_t *sensor, int effect)
{
    int ret=0;
    effect++;
    if (effect <= 0 || effect > NUM_SPECIAL_EFFECTS) {
        return -1;
    }
    for (int i=0; i<5; i++) {
        WRITE_REG_OR_RETURN(BANK_DSP, special_effects_regs[0][i], special_effects_regs[effect][i]);
    }
    return ret;
}

static int set_wb_mode(sensor_t *sensor, int mode)
{
    int ret=0;
    if (mode < 0 || mode > NUM_WB_MODES) {
        return -1;
    }
    SET_REG_BITS_OR_RETURN(BANK_DSP, 0XC7, 6, 1, mode?1:0);
    if(mode) {
        for (int i=0; i<3; i++) {
            WRITE_REG_OR_RETURN(BANK_DSP, wb_modes_regs[0][i], wb_modes_regs[mode][i]);
        }
    }
    return ret;
}

static int set_ae_level(sensor_t *sensor, int level)
{
    int ret=0;
    level += 3;
    if (level <= 0 || level > NUM_AE_LEVELS) {
        return -1;
    }
    for (int i=0; i<3; i++) {
        WRITE_REG_OR_RETURN(BANK_SENSOR, ae_levels_regs[0][i], ae_levels_regs[level][i]);
    }
    return ret;
}

static int set_agc_gain(sensor_t *sensor, int gain)
{
    const uint8_t gain_tbl[31] = {
        0x00, 0x10, 0x18, 0x30, 0x34, 0x38, 0x3C, 0x70, 0x72, 0x74, 0x76, 0x78, 0x7A, 0x7C, 0x7E, 0xF0,
        0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF
    };
    if(gain < 0) {
        gain = 0;
    } else if(gain > 30) {
        gain = 30;
    }
    return write_reg(sensor, BANK_SENSOR, GAIN, gain_tbl[gain]);
}

static int set_quality(sensor_t *sensor, int qs)
{
    return write_reg(sensor, BANK_DSP, QS, qs);
}

static int set_colorbar(sensor_t *sensor, int enable)
{
    return write_reg_bits(sensor, BANK_SENSOR, COM7, COM7_COLOR_BAR, enable);
}

static int set_gainceiling_sensor(sensor_t *sensor, gainceiling_t gainceiling)
{
    return write_reg(sensor, BANK_SENSOR, COM9, COM9_AGC_SET(gainceiling));
}

static int set_agc_sensor(sensor_t *sensor, int enable)
{
    return write_reg_bits(sensor, BANK_SENSOR, COM8, COM8_AGC_EN, enable);
}

static int set_aec_sensor(sensor_t *sensor, int enable)
{
    return write_reg_bits(sensor, BANK_SENSOR, COM8, COM8_AEC_EN, enable);
}

static int set_hmirror_sensor(sensor_t *sensor, int enable)
{
    return write_reg_bits(sensor, BANK_SENSOR, REG04, REG04_HFLIP_IMG, enable);
}

static int set_vflip_sensor(sensor_t *sensor, int enable)
{
    return write_reg_bits(sensor, BANK_SENSOR, REG04, REG04_VFLIP_IMG, enable);
}

static int set_aec_dsp(sensor_t *sensor, int enable)
{
    return set_reg_bits(sensor, BANK_DSP, CTRL0, 6, 1, enable?0:1);
}

static int set_raw_gma_dsp(sensor_t *sensor, int enable)
{
    return set_reg_bits(sensor, BANK_DSP, CTRL1, 5, 1, enable?1:0);
}

static int set_awb_dsp(sensor_t *sensor, int enable)
{
    return set_reg_bits(sensor, BANK_DSP, CTRL1, 3, 1, enable?1:0);
}

static int set_awb_gain_dsp(sensor_t *sensor, int enable)
{
    return set_reg_bits(sensor, BANK_DSP, CTRL1, 2, 1, enable?1:0);
}

static int set_lenc_dsp(sensor_t *sensor, int enable)
{
    return set_reg_bits(sensor, BANK_DSP, CTRL1, 1, 1, enable?1:0);
}

static int set_pre_dsp(sensor_t *sensor, int enable)
{
    return set_reg_bits(sensor, BANK_DSP, CTRL1, 0, 1, enable?1:0);
}

static int set_dcw_dsp(sensor_t *sensor, int enable)
{
    return set_reg_bits(sensor, BANK_DSP, CTRL2, 5, 1, enable?1:0);
}

static int set_bpc_dsp(sensor_t *sensor, int enable)
{
    return set_reg_bits(sensor, BANK_DSP, CTRL3, 7, 1, enable?1:0);
}

static int set_wpc_dsp(sensor_t *sensor, int enable)
{
    return set_reg_bits(sensor, BANK_DSP, CTRL3, 6, 1, enable?1:0);
}

static int set_aec_value(sensor_t *sensor, int value)
{
    if(value < 0) {
        value = 0;
    } else if(value > 0xFFFF) {
        value = 0xFFFF;
    }
    return set_reg_bits(sensor, BANK_SENSOR, REG04, 0, 3, value & 0x3)
           || write_reg(sensor, BANK_SENSOR, AEC, (value >> 2) & 0xFF)
           || set_reg_bits(sensor, BANK_SENSOR, REG45, 0, 0x3F, value >> 10);
}

int ov2640_init(sensor_t *sensor)
{
    sensor->reset = reset;
    sensor->set_pixformat = set_pixformat;
    sensor->set_framesize = set_framesize;
    sensor->set_contrast  = set_contrast;
    sensor->set_brightness= set_brightness;
    sensor->set_saturation= set_saturation;

    sensor->set_quality = set_quality;
    sensor->set_colorbar = set_colorbar;

    sensor->set_gainceiling = set_gainceiling_sensor;
    sensor->set_gain_ctrl = set_agc_sensor;
    sensor->set_exposure_ctrl = set_aec_sensor;
    sensor->set_hmirror = set_hmirror_sensor;
    sensor->set_vflip = set_vflip_sensor;

    sensor->set_whitebal = set_awb_dsp;
    sensor->set_aec2 = set_aec_dsp;
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
    sensor->set_pre = set_pre_dsp;

    return 0;
}
