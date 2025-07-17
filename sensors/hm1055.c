/*
 *
 * HM1055 driver.
 *
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "sccb.h"
#include "xclk.h"
#include "hm1055.h"
#include "hm1055_regs.h"
#include "hm1055_settings.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
static const char *TAG = "HM1055";
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
    ret = write_reg(sensor->slv_addr, SFTRST, 0x55);
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

    switch (pixformat)
    {
    case PIXFORMAT_RAW:
        ret = write_reg(sensor->slv_addr, PORTCTRL, 0x20);
        break;
    case PIXFORMAT_YUV422:
        ret = write_reg(sensor->slv_addr, PORTCTRL, 0x30);
        break;
    case PIXFORMAT_RGB565:
    case PIXFORMAT_RGB888:
        ret = write_reg(sensor->slv_addr, PORTCTRL, 0x40);
        break;
    case PIXFORMAT_RGB555:
        ret = write_reg(sensor->slv_addr, PORTCTRL, 0x50);
        break;
    case PIXFORMAT_RGB444:
        ret = write_reg(sensor->slv_addr, PORTCTRL, 0x60);
        break;
    default:
        break;
    }

    if (ret == 0)
    {
        sensor->pixformat = pixformat;
        ESP_LOGD(TAG, "Set pixformat: %d", pixformat);
    }

    return ret;
}

static int set_framesize(sensor_t *sensor, framesize_t framesize)
{
    int ret = 0;

    sensor->status.framesize = framesize;
    ESP_LOGD(TAG, "Set framesize: %d", framesize);
    ret = write_regs(sensor->slv_addr, sensor_default_regs);
    if (framesize == FRAMESIZE_QQVGA)
    {
        ESP_LOGD(TAG, "Set FRAMESIZE_QQVGA");
        ret = write_regs(sensor->slv_addr, sensor_framesize_QQVGA);
    }
    else if (framesize == FRAMESIZE_QCIF)
    {
        ESP_LOGD(TAG, "Set FRAMESIZE_QCIF");
        ret = write_regs(sensor->slv_addr, sensor_framesize_QCIF);
    }
    else if (framesize == FRAMESIZE_240X240)
    {
        ESP_LOGD(TAG, "Set FRAMESIZE_240X240");
        ret = write_regs(sensor->slv_addr, sensor_framesize_240X240);
    }
    else if (framesize == FRAMESIZE_QVGA)
    {
        ESP_LOGD(TAG, "Set FRAMESIZE_QVGA");
        ret = write_regs(sensor->slv_addr, sensor_framesize_QVGA);
    }
    else if (framesize == FRAMESIZE_CIF)
    {
        ESP_LOGD(TAG, "Set FRAMESIZE_CIF");
        ret = write_regs(sensor->slv_addr, sensor_framesize_CIF);
    }
    else if (framesize == FRAMESIZE_VGA)
    {
        ESP_LOGD(TAG, "Set FRAMESIZE_VGA");
        ret = write_regs(sensor->slv_addr, sensor_framesize_VGA);
    }
    else if (framesize == FRAMESIZE_SVGA)
    {
        ESP_LOGD(TAG, "Set FRAMESIZE_SVGA");
        ret = write_regs(sensor->slv_addr, sensor_framesize_SVGA);
    }
    else if (framesize == FRAMESIZE_HD)
    {
        ESP_LOGD(TAG, "Set FRAMESIZE_HD");
        ret = write_regs(sensor->slv_addr, sensor_framesize_HD);
        ret = _set_pll(sensor, 0, 288, 1, 0, 0, 0, 1, 16);
    }
    else
    {
        ESP_LOGD(TAG, "Dont suppost this size, Set FRAMESIZE_VGA");
        ret = write_regs(sensor->slv_addr, sensor_framesize_VGA);
    }

    if (ret == 0)
    {
        ret = write_reg(sensor->slv_addr, CMU, 0x01) || write_reg(sensor->slv_addr, TGRDCFG, 0x01);
    }

    return ret;
}

static int set_hmirror(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = write_reg_bits(sensor->slv_addr, RDCFG, 0x02, enable);
    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set hmirror to: %d", enable);
        sensor->status.hmirror = enable;
    }
    return ret;
}

static int set_vflip(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = write_reg_bits(sensor->slv_addr, RDCFG, 0x01, enable);
    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set vflip to: %d", enable);
        sensor->status.vflip = enable;
    }
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

    ret = write_reg_bits(sensor->slv_addr, AEWBCFG, 0x01, enable);

    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set aec to: %d", enable);
        sensor->status.aec = enable;
    }

    return ret;
}

static int set_whitebal(sensor_t *sensor, int enable)
{
    int ret = 0;

    ret = write_reg_bits(sensor->slv_addr, AEWBCFG, 0x02, enable);

    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set awb to: %d", enable);
        sensor->status.awb = enable;
    }
    return ret;
}

// Gamma enable
static int set_raw_gma_dsp(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = write_reg_bits(sensor->slv_addr, ISPCTRL1, 0x04, enable);
    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set raw_gma to: %d", enable);
        sensor->status.raw_gma = enable;
    }
    return 0;
}

static int set_lenc_dsp(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = write_reg_bits(sensor->slv_addr, ISPCTRL3, 0x40, enable);
    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set lenc to: %d", enable);
        sensor->status.lenc = enable;
    }
    return -1;
}
// real gain
static int set_agc_gain(sensor_t *sensor, int gain)
{
    int ret = 0;
    if (gain < 0 || gain > 7)
    {
        return -1;
    }

    ret = write_reg(sensor->slv_addr, AGAIN, gain);
    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set gain to: %d", gain);
        sensor->status.agc_gain = gain;
    }
    return 0;
}
static int set_aec_value(sensor_t *sensor, int value)
{
    int ret = 0;
    ret = write_reg(sensor->slv_addr, AETARGM, value);
    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set aec_value to: %d", value);
        sensor->status.aec_value = value;
    }

    return 0;
}

static int set_ae_level(sensor_t *sensor, int level)
{
    int ret = 0;
    if (level < -5 || level > 5)
    {
        return -1;
    }
    uint8_t target_level = ((level + 5) * 10) + 5;
    uint8_t upper = target_level * 27 / 25;
    uint8_t lower = target_level * 23 / 25;

    ret = write_reg(sensor->slv_addr, AETARGU, upper) || write_reg(sensor->slv_addr, AETARGL, lower);
    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set ae_level to: %d", level);
        sensor->status.ae_level = level;
    }
    return 0;
}

static int set_brightness(sensor_t *sensor, int level)
{
    int ret = 0;
    uint8_t ispctrl5 = read_reg(sensor->slv_addr, ISPCTRL5);
    uint8_t brightness = 0;

    switch (level)
    {
    case 3:
        brightness = 0xFF;
        break;

    case 2:
        brightness = 0xBA;
        break;

    case 1:
        brightness = 0x96;
        break;

    case 0:
        brightness = 0x72;
        break;

    case -1:
        brightness = 0x48;
        break;

    case -2:
        brightness = 0x24;
        break;

    case -3:
        brightness = 0x00;
        break;

    default: // 0
        break;
    }

    ispctrl5 |= 0x40; // enable brightness
    ret = write_reg(sensor->slv_addr, ISPCTRL5, ispctrl5);
    if (ret == 0)
    {
        ret = write_reg(sensor->slv_addr, BRIGHT, brightness);
    }
    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set brightness to: %d", level);
        sensor->status.brightness = level;
    }
    return ret;
}

static int set_contrast(sensor_t *sensor, int level)
{
    int ret = 0;
    uint8_t ispctrl5 = read_reg(sensor->slv_addr, ISPCTRL5);

    ispctrl5 |= 0x80; // enable contrast
    ret = write_reg(sensor->slv_addr, ISPCTRL5, ispctrl5);
    ret = write_reg(sensor->slv_addr, ACONTQ, (level * 0x20) & 0xFF);
    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set contrast to: %d", level);
        sensor->status.contrast = level;
    }
    return ret;
}

static int set_saturation(sensor_t *sensor, int level)
{
    int ret = 0;
    ret = write_reg(sensor->slv_addr, SAT, (level * 0x20) + 0x4A);
    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set saturation to: %d", level);
        sensor->status.saturation = level;
    }
    return ret;
}

static int get_sharpness(sensor_t *sensor)
{
    int ret = 0;
    int level = 0;
    ret = read_reg(sensor->slv_addr, EDGE);

    level = (ret - 0x60) / 0x20;
    ESP_LOGD(TAG, "Get sharpness: %d", level);

    return level;
}
static int set_sharpness(sensor_t *sensor, int level)
{
    int ret = 0;
    ret = write_reg(sensor->slv_addr, EDGE, (level * 0x20) + 0x60);
    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set sharpness to: %d", level);
        sensor->status.sharpness = level;
    }
    return ret;
}

static int get_denoise(sensor_t *sensor)
{
    int ret = 0;
    int level = 0;
    ret = read_reg(sensor->slv_addr, YDN);

    level = (ret - 0x07) / 2;
    ESP_LOGD(TAG, "Get denoise: %d", level);

    return level;
}
static int set_denoise(sensor_t *sensor, int level)
{
    int ret = 0;
    uint8_t ispctrl5 = read_reg(sensor->slv_addr, ISPCTRL5);

    ispctrl5 |= 0x20; // enable denoise
    ret = write_reg(sensor->slv_addr, ISPCTRL5, ispctrl5);
    ret = write_reg(sensor->slv_addr, YDN, (level * 2) + 0x07);

    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set denoise to: %d", level);
        sensor->status.denoise = level;
    }

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

static int _set_pll(sensor_t *sensor, int bypass, int multiplier, int sys_div, int root_2x, int pre_div, int seld5, int pclk_manual, int pclk_div)
{
    int ret = 0;
    uint8_t ckcfg1 = 0;
    uint8_t ckcfg2 = 0;
    uint8_t ckcfg3 = 0;
    uint8_t pll2 = 0;

    if (sensor->xclk_freq_hz <= 6000000)
    {
        ckcfg2 = 0x00;
    }
    else if (sensor->xclk_freq_hz <= 12000000)
    {
        ckcfg2 = 0x20;
    }
    else if (sensor->xclk_freq_hz <= 18000000)
    {
        ckcfg2 = 0x40;
    }
    else if (sensor->xclk_freq_hz <= 24000000)
    {
        ckcfg2 = 0x60;
    }
    else if (sensor->xclk_freq_hz <= 30000000)
    {
        ckcfg2 = 0x80;
    }
    else if (sensor->xclk_freq_hz <= 36000000)
    {
        ckcfg2 = 0xA0;
    }
    else if (sensor->xclk_freq_hz <= 42000000)
    {
        ckcfg2 = 0xC0;
    }
    else
    { // max is 48000000
        ckcfg2 = 0xE0;
    }

    if (bypass == 0)
    {
        switch (multiplier)
        {
        case 204:
            ckcfg2 |= 10;
            break;
        case 216:
            ckcfg2 |= 11;
            break;
        case 228:
            ckcfg2 |= 0x12;
            break;
        case 240:
            ckcfg2 |= 0x13;
            break;
        case 288:
            ckcfg2 |= 0x17;
            break;
        case 300:
            ckcfg2 |= 0x18;
            break;
        case 312:
            ckcfg2 |= 0x19;
            break;
        case 324:
            ckcfg2 |= 0x1A;
            break;
        case 336:
            ckcfg2 |= 0x1B;
            break;
        case 348:
            ckcfg2 |= 0x1C;
            break;
        case 360:
            ckcfg2 |= 0x1D;
            break;
        default:
            ckcfg2 |= 0x17;
            break;
        }
    }

    if (pclk_manual > 0)
    {
        if (pclk_div > 128)
        {
            pclk_div = 128;
        }
        if (pclk_div < 1)
        {
            pclk_div = 1;
        }
        ckcfg1 |= (pclk_div - 1);
    }

    if (root_2x > 0)
    {
        ckcfg3 = 0x00;
    }
    else
    {
        ckcfg3 = 0x01;
    }

    ESP_LOGD(TAG, "ckcfg1 = 0x%02x, ckcfg2 = 0x%02x, ckcfg3 = 0x%02x, pll2 = 0x%02x", ckcfg1, ckcfg2, ckcfg3, pll2);
    ret = write_reg(sensor->slv_addr, CKCFG1, ckcfg1);
    ret = write_reg(sensor->slv_addr, CKCFG2, ckcfg2);
    ret = write_reg(sensor->slv_addr, CKCFG3, ckcfg3);
    ret = write_reg(sensor->slv_addr, PLL2, pll2);

    return ret;
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
    (void) write_addr_reg;

    sensor->status.brightness = 0;
    sensor->status.contrast = 0;
    sensor->status.saturation = 0;
    sensor->status.sharpness = get_sharpness(sensor);
    sensor->status.denoise = get_denoise(sensor);
    sensor->status.ae_level = 0;
    sensor->status.awb = check_reg_mask(sensor->slv_addr, AEWBCFG, 0x02);
    sensor->status.agc = true;
    sensor->status.aec = check_reg_mask(sensor->slv_addr, AEWBCFG, 0x04);
    sensor->status.hmirror = check_reg_mask(sensor->slv_addr, RDCFG, 0x02);
    sensor->status.vflip = check_reg_mask(sensor->slv_addr, RDCFG, 0x01);
    sensor->status.lenc = check_reg_mask(sensor->slv_addr, ISPCTRL3, 0x40);
    sensor->status.awb_gain = read_reg(sensor->slv_addr, DGAIN);
    sensor->status.agc_gain = read_reg(sensor->slv_addr, AGAIN);
    sensor->status.aec_value = read_reg(sensor->slv_addr, AETARGM);
    return 0;
}

int hm1055_detect(int slv_addr, sensor_id_t *id)
{
    if (HM1055_SCCB_ADDR == slv_addr)
    {
        uint8_t h = SCCB_Read16(slv_addr, IDH);
        uint8_t l = SCCB_Read16(slv_addr, IDL);
        uint16_t PID = (h << 8) | l;
        if (HM1055_PID == PID)
        {
            id->PID = PID;
            return PID;
        }
        else
        {
            ESP_LOGD(TAG, "Mismatch PID=0x%x", PID);
        }
    }
    return 0;
}

int hm1055_init(sensor_t *sensor)
{
    sensor->reset = reset;
    sensor->set_pixformat = set_pixformat;
    sensor->set_framesize = set_framesize;
    sensor->set_contrast = set_contrast;
    sensor->set_brightness = set_brightness;
    sensor->set_saturation = set_saturation;
    sensor->set_sharpness = set_sharpness;
    sensor->set_gainceiling = NULL;
    sensor->set_quality = set_quality;
    sensor->set_colorbar = set_colorbar;
    sensor->set_gain_ctrl = set_gain_ctrl;
    sensor->set_exposure_ctrl = set_exposure_ctrl;
    sensor->set_whitebal = set_whitebal;
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
    sensor->set_raw_gma = set_raw_gma_dsp;
    sensor->set_lenc = set_lenc_dsp;
    sensor->set_denoise = set_denoise;

    sensor->get_reg = get_reg;
    sensor->set_reg = set_reg;
    sensor->set_res_raw = set_res_raw;
    sensor->set_pll = _set_pll;
    sensor->set_xclk = set_xclk;
    return 0;
}
