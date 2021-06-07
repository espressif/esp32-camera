
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

static int reset(sensor_t *sensor)
{
    int ret = 0;
    // Software Reset: clear all registers and reset them to their default values
    ret = write_reg(sensor->slv_addr, RESET_RELATED, 0xf0);
    if (ret) {
        ESP_LOGE(TAG, "Software Reset FAILED!");
        return ret;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ret = write_regs(sensor->slv_addr, GC0308_DEFAULT_CONFIG);
    if (ret == 0) {
        ESP_LOGD(TAG, "Camera defaults loaded");
        vTaskDelay(100 / portTICK_PERIOD_MS);
        // write_reg(sensor->slv_addr, 0xfe, 0x00);
        // set_reg_bits(sensor->slv_addr, 0x86, 0, 0x01, 1);  //vsync polarity
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

static int set_contrast(sensor_t *sensor, uint8_t contrast)
{
    if (contrast != 0) {
        SCCB_Write(sensor->slv_addr, 0xfe, 0x00);
        SCCB_Write(sensor->slv_addr, 0xb3, contrast);
    }
    return 0;
}

static int set_global_gain(sensor_t *sensor, uint8_t gain_level)
{
    if (gain_level != 0) {
        SCCB_Write(sensor->slv_addr, 0xfe, 0x00);
        SCCB_Write(sensor->slv_addr, 0x50, gain_level);
    }
    return 0;
}

static void set_AEC(sensor_t *sensor, bool aec, uint8_t *aec_value)
{
    for (int i = 0; i < sizeof(GC0308_HB_VB_STEPS) / 2; i++) {
        SCCB_Write(sensor->slv_addr, GC0308_HB_VB_STEPS[i][0], GC0308_HB_VB_STEPS[i][1]);
    }
    if (aec == true) { // Auto exposure control
        for (int i = 0; i < sizeof(aec_value); i++) {
            uint8_t index = aec_value[i] + 1;
            SCCB_Write(sensor->slv_addr, GC0308_AEC_EXP_LEVEL[i][0], GC0308_AEC_EXP_LEVEL[index][0]);
            SCCB_Write(sensor->slv_addr, GC0308_AEC_EXP_LEVEL[i][1], GC0308_AEC_EXP_LEVEL[index][1]);
            SCCB_Write(sensor->slv_addr, 0xec, 0x00 + i * 16);
        };
    } else { // Fixed exposure
        uint8_t index = *aec_value + 1;
        SCCB_Write(sensor->slv_addr, GC0308_AEC_EXP_LEVEL[0][0], GC0308_AEC_EXP_LEVEL[index][0]);
        SCCB_Write(sensor->slv_addr, GC0308_AEC_EXP_LEVEL[0][1], GC0308_AEC_EXP_LEVEL[index][1]);
        SCCB_Write(sensor->slv_addr, 0xec, 0x00);
    }
}

static void set_AEC_target_Y(sensor_t *sensor, uint8_t AEC_target_Y)
{
    SCCB_Write(sensor->slv_addr, 0xfe, 0x00);
    SCCB_Write(sensor->slv_addr, 0xd3, AEC_target_Y);
}

static int set_framesize(sensor_t *sensor, framesize_t framesize)
{
    SCCB_Write(sensor->slv_addr, 0xfe, 0x00);
    return 0;
}

static int set_hmirror(sensor_t *sensor, int enable)
{
    int ret = 0;
    sensor->status.hmirror = enable;
    ret = write_reg(sensor->slv_addr, 0xfe, 0x00);
    ret |= set_reg_bits(sensor->slv_addr, 0x14, 0, 0x01, enable != 0);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set h-mirror to: %d", enable);
    }
    return ret;
}

static int set_vflip(sensor_t *sensor, int enable)
{
    int ret = 0;
    sensor->status.vflip = enable;
    ret = write_reg(sensor->slv_addr, 0xfe, 0x00);
    ret |= set_reg_bits(sensor->slv_addr, 0x14, 1, 0x01, enable != 0);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set v-flip to: %d", enable);
    }
    return ret;
}

static int set_colorbar(sensor_t *sensor, int enable)
{
    int ret = 0;
    // ret = write_reg(sensor->slv_addr, 0xfe, 0x00);
    // ret |= set_reg_bits(sensor->slv_addr, P0_DEBUG_MODE3, 3, 0x01, enable);
    if (ret == 0) {
        sensor->status.colorbar = enable;
        ESP_LOGD(TAG, "Set colorbar to: %d", enable);
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
    write_reg(sensor->slv_addr, 0xfe, 0x00);
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
    sensor->status.hmirror = check_reg_mask(sensor->slv_addr, 0x14, 0x01);
    sensor->status.vflip = check_reg_mask(sensor->slv_addr, 0x14, 0x02);
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

int gc0308_detect(int slv_addr)
{
    if (GC0308_SCCB_ADDR == slv_addr) {
        uint8_t PID = SCCB_Read(slv_addr, 0x00);
        if (GC0308_PID == PID) {
            return PID;
        }
    }
    return 0;
}

int gc0308_init(sensor_t *sensor)
{
    sensor->init_status = init_status;
    sensor->reset = reset;
    sensor->set_pixformat = set_pixformat;
    sensor->set_framesize = set_framesize;
    sensor->set_contrast = set_contrast;
    sensor->set_brightness = set_dummy;
    sensor->set_saturation = set_dummy;
    sensor->set_sharpness = set_dummy;
    sensor->set_denoise = set_dummy;
    sensor->set_gainceiling = set_gainceiling_dummy;
    sensor->set_quality = set_dummy;
    sensor->set_colorbar = set_colorbar;
    sensor->set_whitebal = set_dummy;
    sensor->set_gain_ctrl = set_global_gain;
    sensor->set_exposure_ctrl = set_dummy;
    sensor->set_hmirror = set_hmirror;
    sensor->set_vflip = set_vflip;

    sensor->set_aec2 = set_dummy;
    sensor->set_awb_gain = set_dummy;
    sensor->set_agc_gain = set_dummy;
    sensor->set_aec_value = set_dummy;

    sensor->set_special_effect = set_dummy;
    sensor->set_wb_mode = set_dummy;
    sensor->set_ae_level = set_dummy;

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
