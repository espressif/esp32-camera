/*
 *
 * HM0360 driver.
 *
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
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
    if (ret < 0)
    {
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
    if (ret >= 0)
    {
        ret = (ret & 0xFF) << 8;
        ret2 = read_reg(slv_addr, reg + 1);
        if (ret2 < 0)
        {
            ret = ret2;
        }
        else
        {
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
    if (old_value < 0)
    {
        return old_value;
    }
    if ((uint8_t)old_value != value)
    {
        ESP_LOGD(TAG, "NEW REG 0x%04x: 0x%02x to 0x%02x", reg, (uint8_t)old_value, value);
        ret = SCCB_Write16(slv_addr, reg, value);
    }
    else
    {
        ESP_LOGD(TAG, "OLD REG 0x%04x: 0x%02x", reg, (uint8_t)old_value);
        ret = SCCB_Write16(slv_addr, reg, value); // maybe not?
    }
    if (ret < 0)
    {
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
    if (ret < 0)
    {
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
    while (!ret && regs[i][0] != REGLIST_TAIL)
    {
        if (regs[i][0] == REG_DLY)
        {
            vTaskDelay(regs[i][1] / portTICK_PERIOD_MS);
        }
        else
        {
            ret = write_reg(slv_addr, regs[i][0], regs[i][1]);
        }
        i++;
    }
    return ret;
}

static int write_reg16(uint8_t slv_addr, const uint16_t reg, uint16_t value)
{
    if (write_reg(slv_addr, reg, value >> 8) || write_reg(slv_addr, reg + 1, value))
    {
        return -1;
    }
    return 0;
}

static int write_addr_reg(uint8_t slv_addr, const uint16_t reg, uint16_t x_value, uint16_t y_value)
{
    if (write_reg16(slv_addr, reg, x_value) || write_reg16(slv_addr, reg + 2, y_value))
    {
        return -1;
    }
    return 0;
}

#define write_reg_bits(slv_addr, reg, mask, enable) set_reg_bits(slv_addr, reg, 0, mask, (enable) ? (mask) : 0)

static int set_ae_level(sensor_t *sensor, int level);

static int reset(sensor_t *sensor)
{
    vTaskDelay(100 / portTICK_PERIOD_MS);
    int ret = 0;
    // Software Reset: clear all registers and reset them to their default values
    ret = write_reg(sensor->slv_addr, SW_RESET, 0x00);
    if (ret)
    {
        ESP_LOGE(TAG, "Software Reset FAILED!");
        return ret;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ret = write_regs(sensor->slv_addr, sensor_default_regs);
    if (ret == 0)
    {
        ESP_LOGD(TAG, "Camera defaults loaded");
        vTaskDelay(100 / portTICK_PERIOD_MS);
        set_ae_level(sensor, 0);
    }
    return ret;
}

static int set_pixformat(sensor_t *sensor, pixformat_t pixformat)
{
    int ret = 0;
    return ret;
}

static int set_framesize(sensor_t *sensor, framesize_t framesize)
{
    int ret = 0;

    return ret;
}

static int set_hmirror(sensor_t *sensor, int enable)
{
    int ret = 0;
    return ret;
}

static int set_vflip(sensor_t *sensor, int enable)
{
    int ret = 0;
    return ret;
}

static int set_quality(sensor_t *sensor, int qs)
{
    return 0;
}

static int set_colorbar(sensor_t *sensor, int enable)
{
    return 0;
}

static int set_gain_ctrl(sensor_t *sensor, int enable)
{
    return 0;
}

static int set_exposure_ctrl(sensor_t *sensor, int enable)
{
    int ret = 0;

    return ret;
}

// real gain
static int set_agc_gain(sensor_t *sensor, int gain)
{
    int ret = 0;
    return ret;
}
static int set_aec_value(sensor_t *sensor, int value)
{
    int ret = 0;
    return ret;
}

static int set_ae_level(sensor_t *sensor, int level)
{
    int ret = 0;
    return ret;
}

static int set_brightness(sensor_t *sensor, int level)
{
    int ret = 0;
    return ret;
}

static int get_reg(sensor_t *sensor, int reg, int mask)
{
    int ret = 0, ret2 = 0;
    if (mask > 0xFF)
    {
        ret = read_reg16(sensor->slv_addr, reg);
        if (ret >= 0 && mask > 0xFFFF)
        {
            ret2 = read_reg(sensor->slv_addr, reg + 2);
            if (ret2 >= 0)
            {
                ret = (ret << 8) | ret2;
            }
            else
            {
                ret = ret2;
            }
        }
    }
    else
    {
        ret = read_reg(sensor->slv_addr, reg);
    }
    if (ret > 0)
    {
        ret &= mask;
    }
    return ret;
}

static int set_reg(sensor_t *sensor, int reg, int mask, int value)
{
    int ret = 0, ret2 = 0;
    if (mask > 0xFF)
    {
        ret = read_reg16(sensor->slv_addr, reg);
        if (ret >= 0 && mask > 0xFFFF)
        {
            ret2 = read_reg(sensor->slv_addr, reg + 2);
            if (ret2 >= 0)
            {
                ret = (ret << 8) | ret2;
            }
            else
            {
                ret = ret2;
            }
        }
    }
    else
    {
        ret = read_reg(sensor->slv_addr, reg);
    }
    if (ret < 0)
    {
        return ret;
    }
    value = (ret & ~mask) | (value & mask);
    if (mask > 0xFFFF)
    {
        ret = write_reg16(sensor->slv_addr, reg, value >> 8);
        if (ret >= 0)
        {
            ret = write_reg(sensor->slv_addr, reg + 2, value & 0xFF);
        }
    }
    else if (mask > 0xFF)
    {
        ret = write_reg16(sensor->slv_addr, reg, value);
    }
    else
    {
        ret = write_reg(sensor->slv_addr, reg, value);
    }
    return ret;
}

static int set_res_raw(sensor_t *sensor, int startX, int startY, int endX, int endY, int offsetX, int offsetY, int totalX, int totalY, int outputX, int outputY, bool scale, bool binning)
{
    return 0;
}
static int set_xclk(sensor_t *sensor, int timer, int xclk)
{
    int ret = 0;
    sensor->xclk_freq_hz = xclk * 1000000U;
    ret = xclk_timer_conf(timer, sensor->xclk_freq_hz);
    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set xclk to %d", xclk);
    }
    return ret;
}

static int init_status(sensor_t *sensor)
{
    sensor->status.brightness = 0;
    sensor->status.contrast = 0;
    sensor->status.saturation = 0;
    // sensor->status.awb = check_reg_mask(sensor->slv_addr, AEWBCFG, 0x02);
    sensor->status.agc = true;
    // sensor->status.aec = check_reg_mask(sensor->slv_addr, AEWBCFG, 0x04);
    // sensor->status.hmirror = check_reg_mask(sensor->slv_addr, RDCFG, 0x02);
    // sensor->status.vflip = check_reg_mask(sensor->slv_addr, RDCFG, 0x01);
    // sensor->status.lenc = check_reg_mask(sensor->slv_addr, ISPCTRL3, 0x40);
    // sensor->status.awb_gain = read_reg(sensor->slv_addr, DGAIN);
    // sensor->status.agc_gain = read_reg(sensor->slv_addr, AGAIN);
    // sensor->status.aec_value = read_reg(sensor->slv_addr, AETARGM);
    return 0;
}

int hm0360_detect(int slv_addr, sensor_id_t *id)
{
    if (HM1055_SCCB_ADDR == slv_addr)
    {
        uint8_t h = SCCB_Read16(slv_addr, MODEL_ID_H);
        uint8_t l = SCCB_Read16(slv_addr, MODEL_ID_L);
        uint16_t PID = (h << 8) | l;
        if (HM0360_PID == PID)
        {
            id->PID = PID;
            id->VER = SCCB_Read16(slv_addr, SILICON_REV); 
            return PID;
        }
        else
        {
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
    sensor->set_contrast = NULL;
    sensor->set_brightness = set_brightness;
    sensor->set_saturation = NULL;
    sensor->set_sharpness = NULL;
    sensor->set_gainceiling = NULL;
    sensor->set_quality = set_quality;
    sensor->set_colorbar = set_colorbar;
    sensor->set_gain_ctrl = set_gain_ctrl;
    sensor->set_exposure_ctrl = set_exposure_ctrl;
    sensor->set_whitebal = NULL;
    sensor->set_hmirror = set_hmirror;
    sensor->set_vflip = set_vflip;
    sensor->init_status = init_status;
    sensor->set_aec2 = NULL;
    sensor->set_aec_value = set_aec_value;
    sensor->set_special_effect = NULL;
    sensor->set_wb_mode = NULL;
    sensor->set_ae_level = set_ae_level;
    sensor->set_dcw = NULL;
    sensor->set_bpc = NULL;
    sensor->set_wpc = NULL;
    sensor->set_agc_gain = set_agc_gain;
    sensor->set_raw_gma = NULL;
    sensor->set_lenc = NULL;
    sensor->set_denoise = NULL;

    sensor->get_reg = get_reg;
    sensor->set_reg = set_reg;
    sensor->set_res_raw = set_res_raw;
    sensor->set_pll = NULL;
    sensor->set_xclk = set_xclk;
    return 0;
}
