/*
 *
 * HM0360 driver.
 *
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "sccb.h"
#include "xclk.h"
#include "hm0360.h"
#include "hm0360_regs.h"
#include "hm0360_settings.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
static const char *TAG = "HM0360";
#endif

// #define REG_DEBUG_ON

static int _set_pll(sensor_t *sensor, int bypass, int multiplier, int sys_div, int root_2x, int pre_div, int seld5, int pclk_manual, int pclk_div);

static int read_reg(uint8_t slv_addr, const uint16_t reg)
{
    int ret = SCCB_Read16(slv_addr, reg);
#ifdef REG_DEBUG_ON
    if (ret < 0) {
        ESP_LOGE(TAG, "READ REG 0x%04x FAILED: %d", reg, ret);
    }
#endif
    return ret;
}

static int check_reg_mask(uint8_t slv_addr, uint16_t reg, uint8_t mask)
{
    return (read_reg(slv_addr, reg) & mask) == mask;
}

static int read_reg16(uint8_t slv_addr, const uint16_t reg)
{
    int ret = 0, ret2 = 0;

    ret = read_reg(slv_addr, reg);
    if (ret >= 0) {
        ret = (ret & 0xFF) << 8;
        ret2 = read_reg(slv_addr, reg + 1);
        if (ret2 < 0) {
            ret = ret2;
        } else {
            ret |= ret2 & 0xFF;
        }
    }

    return ret;
}

static int write_reg(uint8_t slv_addr, const uint16_t reg, uint8_t value)
{
    int ret = 0;
#ifndef REG_DEBUG_ON
    ret = SCCB_Write16(slv_addr, reg, value);
#else
    int old_value = read_reg(slv_addr, reg);
    if (old_value < 0) {
        return old_value;
    }

    if ((uint8_t)old_value != value) {
        ESP_LOGD(TAG, "NEW REG 0x%04x: 0x%02x to 0x%02x", reg, (uint8_t)old_value, value);
        ret = SCCB_Write16(slv_addr, reg, value);
    } else {
        ESP_LOGD(TAG, "OLD REG 0x%04x: 0x%02x", reg, (uint8_t)old_value);
        ret = SCCB_Write16(slv_addr, reg, value); // maybe not?
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
    if (ret < 0) {
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

#define write_reg_bits(slv_addr, reg, mask, enable) set_reg_bits(slv_addr, reg, 0, mask, (enable) ? (mask) : 0)

static int reset(sensor_t *sensor)
{
    vTaskDelay(100 / portTICK_PERIOD_MS);
    int ret = 0;

    // Software Reset: clear all registers and reset them to their default values
    ret = write_reg(sensor->slv_addr, SW_RESET, 0x00);
    if (ret) {
        ESP_LOGE(TAG, "Software Reset FAILED!");
        return ret;
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
    ret = write_regs(sensor->slv_addr, sensor_default_regs);
    if (ret == 0) {
        ESP_LOGD(TAG, "Camera defaults loaded");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    return ret;
}

static int set_pixformat(sensor_t *sensor, pixformat_t pixformat)
{
    int ret = 0;
    sensor->pixformat = pixformat;

    switch (pixformat) {
        case PIXFORMAT_GRAYSCALE:
            break;
        default:
            ESP_LOGE(TAG, "Only support GRAYSCALE");
            return -1;
    }

    return ret;
}

static int set_framesize(sensor_t *sensor, framesize_t framesize)
{
    int ret = 0;

    sensor->status.framesize = framesize;
    ret = write_regs(sensor->slv_addr, sensor_default_regs);

    if (framesize == FRAMESIZE_QQVGA) {
        ESP_LOGI(TAG, "Set FRAMESIZE_QQVGA");
        ret |= write_regs(sensor->slv_addr, sensor_framesize_QQVGA);
        ret |= set_reg_bits(sensor->slv_addr, 0x3024, 0, 0x01, 1);
    } else if (framesize == FRAMESIZE_QVGA) {
        ESP_LOGI(TAG, "Set FRAMESIZE_QVGA");
        ret |= write_regs(sensor->slv_addr, sensor_framesize_QVGA);
        ret |= set_reg_bits(sensor->slv_addr, 0x3024, 0, 0x01, 1);
    } else if (framesize == FRAMESIZE_VGA) {
        ESP_LOGI(TAG, "Set FRAMESIZE_VGA");
        ret |= set_reg_bits(sensor->slv_addr, 0x3024, 0, 0x01, 0);
    } else {
        ESP_LOGI(TAG, "Dont suppost this size, Set FRAMESIZE_VGA");
        ret |= set_reg_bits(sensor->slv_addr, 0x3024, 0, 0x01, 0);
    }

    if (ret == 0) {
        _set_pll(sensor, 0, 0, 0, 0, 0, 0, 0, 0);
        ret |= write_reg(sensor->slv_addr, 0x0104, 0x01);
    }

    return ret;
}

static int set_hmirror(sensor_t *sensor, int enable)
{
    if (set_reg_bits(sensor->slv_addr, 0x0101, 0, 0x01, enable)) {
        return -1;
    }

    ESP_LOGD(TAG, "Set h-mirror to: %d", enable);

    return 0;
}

static int set_vflip(sensor_t *sensor, int enable)
{
    if (set_reg_bits(sensor->slv_addr, 0x0101, 1, 0x01, enable)) {
        return -1;
    }

    ESP_LOGD(TAG, "Set v-flip to: %d", enable);

    return 0;
}

static int set_colorbar(sensor_t *sensor, int enable)
{
    if (set_reg_bits(sensor->slv_addr, 0x0601, 0, 0x01, enable)) {
        return -1;
    }

    ESP_LOGD(TAG, "Set color-bar to: %d", enable);

    return 0;
}

static int set_exposure_ctrl(sensor_t *sensor, int enable)
{
    if (set_reg_bits(sensor->slv_addr, 0x2000, 0, 0x01, enable)) {
        return -1;
    }

    ESP_LOGD(TAG, "Set exposure to: %d", enable);

    return 0;
}

static int set_brightness(sensor_t *sensor, int level)
{
    uint8_t ae_mean;

    switch (level) {
        case 0:
            ae_mean = 60;
            break;
        case 1:
            ae_mean = 80;
            break;
        case 2:
            ae_mean = 100;
            break;
        case 3:
            ae_mean = 127;
            break;
        default:
            ae_mean = 80;
    }

    return write_reg(sensor->slv_addr, AE_TARGET_MEAN, ae_mean);
}

static int get_reg(sensor_t *sensor, int reg, int mask)
{
    int ret = 0, ret2 = 0;

    if (mask > 0xFF) {
        ret = read_reg16(sensor->slv_addr, reg);
        if (ret >= 0 && mask > 0xFFFF) {
            ret2 = read_reg(sensor->slv_addr, reg + 2);
            if (ret2 >= 0) {
                ret = (ret << 8) | ret2;
            } else {
                ret = ret2;
            }
        }
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
    int ret = 0, ret2 = 0;

    if (mask > 0xFF) {
        ret = read_reg16(sensor->slv_addr, reg);
        if (ret >= 0 && mask > 0xFFFF) {
            ret2 = read_reg(sensor->slv_addr, reg + 2);
            if (ret2 >= 0) {
                ret = (ret << 8) | ret2;
            } else {
                ret = ret2;
            }
        }
    } else {
        ret = read_reg(sensor->slv_addr, reg);
    }

    if (ret < 0) {
        return ret;
    }

    value = (ret & ~mask) | (value & mask);
    if (mask > 0xFFFF) {
        ret = write_reg16(sensor->slv_addr, reg, value >> 8);
        if (ret >= 0) {
            ret = write_reg(sensor->slv_addr, reg + 2, value & 0xFF);
        }
    } else if (mask > 0xFF) {
        ret = write_reg16(sensor->slv_addr, reg, value);
    } else {
        ret = write_reg(sensor->slv_addr, reg, value);
    }

    return ret;
}

static int set_xclk(sensor_t *sensor, int timer, int xclk)
{
    int ret = 0;
    sensor->xclk_freq_hz = xclk * 1000000U;
    ret = xclk_timer_conf(timer, sensor->xclk_freq_hz);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set xclk to %d", xclk);
    }
    return ret;
}

static int _set_pll(sensor_t *sensor, int bypass, int multiplier, int sys_div, int root_2x, int pre_div, int seld5, int pclk_manual, int pclk_div)
{
    (void)bypass;
    (void)multiplier;
    (void)sys_div;
    (void)root_2x;
    (void)pre_div;
    (void)seld5;
    (void)pclk_manual;
    (void)pclk_div;
    uint8_t value = 0;
    uint8_t pll_cfg = 0;

    if (sensor->xclk_freq_hz <= 6000000) {
        value = 0x03;
    } else if (sensor->xclk_freq_hz <= 12000000) {
        value = 0x02;
    } else if (sensor->xclk_freq_hz <= 18000000) {
        value = 0x01;
    } else { // max is 48000000
        value = 0x00;
    }

    int ret = read_reg(sensor->slv_addr, PLL1CFG);
    if (ret < 0) {
        return ret;
    }
    if (ret > 0xFF) {
        /*
         * Guard against unexpected wide register values. If read_reg
         * ever returns a 16-bit result, reject values that don't fit
         * in a single byte to avoid truncation.
         */
        return -ERANGE;
    }

    pll_cfg = (uint8_t)ret;
    return write_reg(sensor->slv_addr, PLL1CFG, (pll_cfg & 0xFC) | value);
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

static int init_status(sensor_t *sensor)
{
    (void) write_addr_reg;

    sensor->status.brightness = 0;
    sensor->status.contrast = 0;
    sensor->status.saturation = 0;
    sensor->status.sharpness = 0;
    sensor->status.denoise = 0;
    sensor->status.ae_level = 0;
    sensor->status.awb = 0;
    sensor->status.aec = 0;
    sensor->status.hmirror = check_reg_mask(sensor->slv_addr, 0x101, 0x01);
    sensor->status.vflip = check_reg_mask(sensor->slv_addr, 0x101, 0x02);
    sensor->status.lenc = 0;
    sensor->status.awb_gain = 0;
    sensor->status.agc_gain = 0;
    sensor->status.aec_value = 0;

    return 0;
}

int hm0360_detect(int slv_addr, sensor_id_t *id)
{
    if (HM1055_SCCB_ADDR == slv_addr) {
        uint8_t h = SCCB_Read16(slv_addr, MODEL_ID_H);
        uint8_t l = SCCB_Read16(slv_addr, MODEL_ID_L);
        uint16_t PID = (h << 8) | l;
        if (HM0360_PID == PID) {
            id->PID = PID;
            id->VER = SCCB_Read16(slv_addr, SILICON_REV); 
            return PID;
        } else {
            ESP_LOGD(TAG, "Mismatch PID=0x%x", PID);
        }
    }
    return 0;
}

int hm0360_init(sensor_t *sensor)
{
    sensor->reset = reset;
    sensor->set_pixformat = set_pixformat;
    sensor->set_framesize = set_framesize;
    sensor->set_contrast = set_dummy;
    sensor->set_brightness = set_brightness;
    sensor->set_saturation = set_dummy;
    sensor->set_sharpness = set_dummy;
    sensor->set_gainceiling = set_gainceiling_dummy;
    sensor->set_quality = set_dummy;
    sensor->set_colorbar = set_colorbar;
    sensor->set_gain_ctrl = set_dummy;
    sensor->set_exposure_ctrl = set_exposure_ctrl;
    sensor->set_whitebal = set_dummy;
    sensor->set_hmirror = set_hmirror;
    sensor->set_vflip = set_vflip;
    sensor->init_status = init_status;
    sensor->set_aec2 = set_dummy;
    sensor->set_aec_value = set_dummy;
    sensor->set_special_effect = set_dummy;
    sensor->set_wb_mode = set_dummy;
    sensor->set_ae_level = set_dummy;
    sensor->set_dcw = set_dummy;
    sensor->set_bpc = set_dummy;
    sensor->set_wpc = set_dummy;
    sensor->set_agc_gain = set_dummy;
    sensor->set_raw_gma = set_dummy;
    sensor->set_lenc = set_dummy;
    sensor->set_denoise = set_dummy;

    sensor->get_reg = get_reg;
    sensor->set_reg = set_reg;
    sensor->set_res_raw = NULL;
    sensor->set_pll = _set_pll;
    sensor->set_xclk = set_xclk;
    return 0;
}
