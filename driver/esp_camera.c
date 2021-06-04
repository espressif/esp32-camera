// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
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
#include <stdlib.h>
#include <string.h>
#include "time.h"
#include "sys/time.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "sensor.h"
#include "sccb.h"
#include "cam_hal.h"
#include "esp_camera.h"
// #include "camera_common.h"
#include "xclk.h"
#if CONFIG_OV2640_SUPPORT
#include "ov2640.h"
#endif
#if CONFIG_OV7725_SUPPORT
#include "ov7725.h"
#endif
#if CONFIG_OV3660_SUPPORT
#include "ov3660.h"
#endif
#if CONFIG_OV5640_SUPPORT
#include "ov5640.h"
#endif
#if CONFIG_NT99141_SUPPORT
#include "nt99141.h"
#endif
#if CONFIG_OV7670_SUPPORT
#include "ov7670.h"
#endif
#if CONFIG_GC2145_SUPPORT
#include "gc2145.h"
#endif


#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#define TAG ""
#else
#include "esp_log.h"
static const char *TAG = "camera";
#endif

typedef struct {
    sensor_t sensor;
    camera_fb_t fb;
} camera_state_t;

static const char *CAMERA_SENSOR_NVS_KEY = "sensor";
static const char *CAMERA_PIXFORMAT_NVS_KEY = "pixformat";
static camera_state_t *s_state = NULL;

#if CONFIG_IDF_TARGET_ESP32S3 // LCD_CAM module of ESP32-S3 will generate xclk
#define CAMERA_ENABLE_OUT_CLOCK(v)
#define CAMERA_DISABLE_OUT_CLOCK()
#else
#define CAMERA_ENABLE_OUT_CLOCK(v) camera_enable_out_clock((v))
#define CAMERA_DISABLE_OUT_CLOCK() camera_disable_out_clock()
#endif

static esp_err_t camera_probe(const camera_config_t *config, camera_model_t *out_camera_model)
{
    *out_camera_model = CAMERA_NONE;
    if (s_state != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_state = (camera_state_t *) calloc(sizeof(camera_state_t), 1);
    if (!s_state) {
        return ESP_ERR_NO_MEM;
    }

    if (config->pin_xclk >= 0) {
        ESP_LOGD(TAG, "Enabling XCLK output");
        CAMERA_ENABLE_OUT_CLOCK(config);
    }

    if (config->pin_sscb_sda != -1) {
        ESP_LOGD(TAG, "Initializing SSCB");
        SCCB_Init(config->pin_sscb_sda, config->pin_sscb_scl);
    }

    if (config->pin_pwdn >= 0) {
        ESP_LOGD(TAG, "Resetting camera by power down line");
        gpio_config_t conf = { 0 };
        conf.pin_bit_mask = 1LL << config->pin_pwdn;
        conf.mode = GPIO_MODE_OUTPUT;
        gpio_config(&conf);

        // carefull, logic is inverted compared to reset pin
        gpio_set_level(config->pin_pwdn, 1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        gpio_set_level(config->pin_pwdn, 0);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    if (config->pin_reset >= 0) {
        ESP_LOGD(TAG, "Resetting camera");
        gpio_config_t conf = { 0 };
        conf.pin_bit_mask = 1LL << config->pin_reset;
        conf.mode = GPIO_MODE_OUTPUT;
        gpio_config(&conf);

        gpio_set_level(config->pin_reset, 0);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        gpio_set_level(config->pin_reset, 1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }


    ESP_LOGD(TAG, "Searching for camera address");
    vTaskDelay(10 / portTICK_PERIOD_MS);

    uint8_t slv_addr = SCCB_Probe();

    if (slv_addr == 0) {
        CAMERA_DISABLE_OUT_CLOCK();
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Detected camera at address=0x%02x", slv_addr);
    s_state->sensor.slv_addr = slv_addr;
    s_state->sensor.xclk_freq_hz = config->xclk_freq_hz;

    /**
     * Read sensor ID
     */
    sensor_id_t *id = &s_state->sensor.id;

    if (slv_addr == OV2640_SCCB_ADDR || slv_addr == OV7725_SCCB_ADDR) {
        SCCB_Write(slv_addr, 0xFF, 0x01);//bank sensor
        id->PID = SCCB_Read(slv_addr, REG_PID);
        id->VER = SCCB_Read(slv_addr, REG_VER);
        id->MIDL = SCCB_Read(slv_addr, REG_MIDL);
        id->MIDH = SCCB_Read(slv_addr, REG_MIDH);
    } else if (slv_addr == OV5640_SCCB_ADDR || slv_addr == OV3660_SCCB_ADDR || slv_addr == GC2145_SCCB_ADDR) {
        id->PID = SCCB_Read(slv_addr, 0xf0); // read for gc2145
        if (GC2145_PID == id->PID) {
            /* sensor is GC2145 */
        } else {
            id->PID = SCCB_Read16(slv_addr, REG16_CHIDH);
            id->VER = SCCB_Read16(slv_addr, REG16_CHIDL);
        }
    } else if (slv_addr == NT99141_SCCB_ADDR) {
        SCCB_Write16(slv_addr, 0x3008, 0x01);//bank sensor
        id->PID = SCCB_Read16(slv_addr, 0x3000);
        id->VER = SCCB_Read16(slv_addr, 0x3001);
        if (config->xclk_freq_hz > 10000000) {
            ESP_LOGE(TAG, "NT99141: only XCLK under 10MHz is supported, and XCLK is now set to 10M");
            s_state->sensor.xclk_freq_hz = 10000000;
        }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Camera PID=0x%02x VER=0x%02x MIDL=0x%02x MIDH=0x%02x",
             id->PID, id->VER, id->MIDH, id->MIDL);

    /**
     * Initialize sensor according to sensor ID
     */
    switch (id->PID) {
#if CONFIG_OV2640_SUPPORT
    case OV2640_PID:
        *out_camera_model = CAMERA_OV2640;
        ESP_LOGI(TAG, "Detected OV2640 camera");
        ov2640_init(&s_state->sensor);
        break;
#endif
#if CONFIG_OV7725_SUPPORT
    case OV7725_PID:
        *out_camera_model = CAMERA_OV7725;
        ESP_LOGI(TAG, "Detected OV7725 camera");
        ov7725_init(&s_state->sensor);
        break;
#endif
#if CONFIG_OV3660_SUPPORT
    case OV3660_PID:
        *out_camera_model = CAMERA_OV3660;
        ESP_LOGI(TAG, "Detected OV3660 camera");
        ov3660_init(&s_state->sensor);
        break;
#endif
#if CONFIG_OV5640_SUPPORT
    case OV5640_PID:
        *out_camera_model = CAMERA_OV5640;
        ESP_LOGI(TAG, "Detected OV5640 camera");
        ov5640_init(&s_state->sensor);
        break;
#endif
#if CONFIG_OV7670_SUPPORT
    case OV7670_PID:
        *out_camera_model = CAMERA_OV7670;
        ESP_LOGI(TAG, "Detected OV7670 camera");
        ov7670_init(&s_state->sensor);
        break;
#endif
#if CONFIG_NT99141_SUPPORT
    case NT99141_PID:
        *out_camera_model = CAMERA_NT99141;
        ESP_LOGI(TAG, "Detected NT99141 camera");
        NT99141_init(&s_state->sensor);
        break;
#endif
#if CONFIG_GC2145_SUPPORT
    case GC2145_PID:
        *out_camera_model = CAMERA_GC2145;
        ESP_LOGI(TAG, "Detected GC2145 camera");
        gc2145_init(&s_state->sensor);
        break;
#endif
    default:
        id->PID = 0;
        CAMERA_DISABLE_OUT_CLOCK();
        ESP_LOGE(TAG, "Detected camera not supported.");
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_LOGD(TAG, "Doing SW reset of sensor");
    s_state->sensor.reset(&s_state->sensor);

    return ESP_OK;
}


esp_err_t esp_camera_init(const camera_config_t *config)
{
    esp_err_t err;
    err = cam_init(config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return err;
    }

    camera_model_t camera_model = CAMERA_NONE;
    err = camera_probe(config, &camera_model);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera probe failed with error 0x%x(%s)", err, esp_err_to_name(err));
        goto fail;
    }

    framesize_t frame_size = (framesize_t) config->frame_size;
    pixformat_t pix_format = (pixformat_t) config->pixel_format;

    if (PIXFORMAT_JPEG == pix_format && (!camera_sensor[camera_model].support_jpeg)) {
        ESP_LOGE(TAG, "JPEG format is not supported on this sensor");
        err = ESP_ERR_NOT_SUPPORTED;
        goto fail;
    }

    if (frame_size > camera_sensor[camera_model].max_size) {
        ESP_LOGW(TAG, "The frame size exceeds the maximum for this sensor, it will be forced to the maximum possible value");
        frame_size = camera_sensor[camera_model].max_size;
    }

    err = cam_config(config, frame_size, s_state->sensor.id.PID);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera config failed with error 0x%x", err);
        goto fail;
    }

    s_state->sensor.status.framesize = frame_size;
    s_state->sensor.pixformat = pix_format;
    // ESP_LOGD(TAG, "Setting frame size to %dx%d", s_state->width, s_state->height);
    if (s_state->sensor.set_framesize(&s_state->sensor, frame_size) != 0) {
        ESP_LOGE(TAG, "Failed to set frame size");
        err = ESP_ERR_CAMERA_FAILED_TO_SET_FRAME_SIZE;
        goto fail;
    }
    s_state->sensor.set_pixformat(&s_state->sensor, pix_format);

    if (s_state->sensor.id.PID == OV2640_PID) {
        s_state->sensor.set_gainceiling(&s_state->sensor, GAINCEILING_2X);
        s_state->sensor.set_bpc(&s_state->sensor, false);
        s_state->sensor.set_wpc(&s_state->sensor, true);
        s_state->sensor.set_lenc(&s_state->sensor, true);
    }

    if (pix_format == PIXFORMAT_JPEG) {
        s_state->sensor.set_quality(&s_state->sensor, config->jpeg_quality);
    }
    s_state->sensor.init_status(&s_state->sensor);

    cam_start();

    return ESP_OK;

fail:
    CAMERA_DISABLE_OUT_CLOCK();
    esp_camera_deinit();
    return err;
}

esp_err_t esp_camera_deinit()
{
    esp_err_t ret = cam_deinit();
    if (s_state) {
        SCCB_Deinit();

        free(s_state);
        s_state = NULL;
    }

    return ret;
}

#define FB_GET_TIMEOUT (4000 / portTICK_PERIOD_MS)

camera_fb_t *esp_camera_fb_get()
{
    if (s_state == NULL) {
        return NULL;
    }
    camera_fb_t *fb = cam_take(FB_GET_TIMEOUT);
    //set the frame properties
    if (fb) {
        fb->width = resolution[s_state->sensor.status.framesize].width;
        fb->height = resolution[s_state->sensor.status.framesize].height;
        fb->format = s_state->sensor.pixformat;
    }
    return fb;
}

void esp_camera_fb_return(camera_fb_t *fb)
{
    if (s_state == NULL) {
        return;
    }
    cam_give(fb);
}

sensor_t *esp_camera_sensor_get()
{
    if (s_state == NULL) {
        return NULL;
    }
    return &s_state->sensor;
}

esp_err_t esp_camera_save_to_nvs(const char *key)
{
#if ESP_IDF_VERSION_MAJOR > 3
    nvs_handle_t handle;
#else
    nvs_handle handle;
#endif
    esp_err_t ret = nvs_open(key, NVS_READWRITE, &handle);

    if (ret == ESP_OK) {
        sensor_t *s = esp_camera_sensor_get();
        if (s != NULL) {
            ret = nvs_set_blob(handle, CAMERA_SENSOR_NVS_KEY, &s->status, sizeof(camera_status_t));
            if (ret == ESP_OK) {
                uint8_t pf = s->pixformat;
                ret = nvs_set_u8(handle, CAMERA_PIXFORMAT_NVS_KEY, pf);
            }
            return ret;
        } else {
            return ESP_ERR_CAMERA_NOT_DETECTED;
        }
        nvs_close(handle);
        return ret;
    } else {
        return ret;
    }
}

esp_err_t esp_camera_load_from_nvs(const char *key)
{
#if ESP_IDF_VERSION_MAJOR > 3
    nvs_handle_t handle;
#else
    nvs_handle handle;
#endif
    uint8_t pf;

    esp_err_t ret = nvs_open(key, NVS_READWRITE, &handle);

    if (ret == ESP_OK) {
        sensor_t *s = esp_camera_sensor_get();
        camera_status_t st;
        if (s != NULL) {
            size_t size = sizeof(camera_status_t);
            ret = nvs_get_blob(handle, CAMERA_SENSOR_NVS_KEY, &st, &size);
            if (ret == ESP_OK) {
                s->set_ae_level(s, st.ae_level);
                s->set_aec2(s, st.aec2);
                s->set_aec_value(s, st.aec_value);
                s->set_agc_gain(s, st.agc_gain);
                s->set_awb_gain(s, st.awb_gain);
                s->set_bpc(s, st.bpc);
                s->set_brightness(s, st.brightness);
                s->set_colorbar(s, st.colorbar);
                s->set_contrast(s, st.contrast);
                s->set_dcw(s, st.dcw);
                s->set_denoise(s, st.denoise);
                s->set_exposure_ctrl(s, st.aec);
                s->set_framesize(s, st.framesize);
                s->set_gain_ctrl(s, st.agc);
                s->set_gainceiling(s, st.gainceiling);
                s->set_hmirror(s, st.hmirror);
                s->set_lenc(s, st.lenc);
                s->set_quality(s, st.quality);
                s->set_raw_gma(s, st.raw_gma);
                s->set_saturation(s, st.saturation);
                s->set_sharpness(s, st.sharpness);
                s->set_special_effect(s, st.special_effect);
                s->set_vflip(s, st.vflip);
                s->set_wb_mode(s, st.wb_mode);
                s->set_whitebal(s, st.awb);
                s->set_wpc(s, st.wpc);
            }
            ret = nvs_get_u8(handle, CAMERA_PIXFORMAT_NVS_KEY, &pf);
            if (ret == ESP_OK) {
                s->set_pixformat(s, pf);
            }
        } else {
            return ESP_ERR_CAMERA_NOT_DETECTED;
        }
        nvs_close(handle);
        return ret;
    } else {
        ESP_LOGW(TAG, "Error (%d) opening nvs key \"%s\"", ret, key);
        return ret;
    }
}
