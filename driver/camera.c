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
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/lcd_cam.h"
// #include "soc/soc.h"
// #include "soc/gpio_sig_map.h"
// #include "soc/i2s_reg.h"
// #include "soc/i2s_struct.h"
// #include "soc/io_mux_reg.h"
// #include "driver/rtc_io.h"
// #include "driver/periph_ctrl.h"
// #include "esp_intr_alloc.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "sensor.h"
#include "sccb.h"
#include "esp_camera.h"
#include "camera_common.h"
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

typedef enum {
    CAMERA_NONE = 0,
    CAMERA_UNKNOWN = 1,
    CAMERA_OV7725 = 7725,
    CAMERA_OV2640 = 2640,
    CAMERA_OV3660 = 3660,
    CAMERA_OV5640 = 5640,
    CAMERA_OV7670 = 7670,
    CAMERA_NT99141 = 9141,
} camera_model_t;

#define REG_PID        0x0A
#define REG_VER        0x0B
#define REG_MIDH       0x1C
#define REG_MIDL       0x1D

#define REG16_CHIDH     0x300A
#define REG16_CHIDL     0x300B

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#define TAG ""
#else
#include "esp_log.h"
static const char* TAG = "camera";
#endif
static const char* CAMERA_SENSOR_NVS_KEY = "sensor";
static const char* CAMERA_PIXFORMAT_NVS_KEY = "pixformat";

typedef struct {
    camera_config_t config;
    sensor_t sensor;

    size_t fb_size;
    size_t data_size;

    size_t width;
    size_t height;
    size_t in_bytes_per_pixel;
    size_t fb_bytes_per_pixel;

    lcd_cam_handle_t lcd_cam;
    lcd_cam_config_t lcd_cam_config;
} camera_state_t;

camera_state_t* s_state = NULL;

// static int skip_frame()
// {
//     if (s_state == NULL) {
//         return -1;
//     }
//     int64_t st_t = esp_timer_get_time();
//     while (gpio_get_level(s_state->config.pin_vsync) == 0) {
//         if((esp_timer_get_time() - st_t) > 1000000LL){
//             goto timeout;
//         }
//     }
//     while (gpio_get_level(s_state->config.pin_vsync) != 0) {
//         if((esp_timer_get_time() - st_t) > 1000000LL){
//             goto timeout;
//         }
//     }
//     while (gpio_get_level(s_state->config.pin_vsync) == 0) {
//         if((esp_timer_get_time() - st_t) > 1000000LL){
//             goto timeout;
//         }
//     }
//     return 0;

// timeout:
//     ESP_LOGE(TAG, "Timeout waiting for VSYNC");
//     return -1;
// }

/*
 * Public Methods
 * */

esp_err_t camera_probe(const camera_config_t* config, camera_model_t* out_camera_model)
{
    if (s_state == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (config->pin_sscb_sda != -1) {
      ESP_LOGD(TAG, "Initializing SSCB");
      SCCB_Init(config->pin_sscb_sda, config->pin_sscb_scl);
    }
	
    if(config->pin_pwdn >= 0) {
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

    if(config->pin_reset >= 0) {
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
        *out_camera_model = CAMERA_NONE;
        return ESP_ERR_CAMERA_NOT_DETECTED;
    }
    
    //slv_addr = 0x30;
    ESP_LOGD(TAG, "Detected camera at address=0x%02x", slv_addr);
    sensor_id_t* id = &s_state->sensor.id;

#if CONFIG_OV2640_SUPPORT
    if (slv_addr == 0x30) {
        ESP_LOGD(TAG, "Resetting OV2640");
        //camera might be OV2640. try to reset it
        SCCB_Write(0x30, 0xFF, 0x01);//bank sensor
        SCCB_Write(0x30, 0x12, 0x80);//reset
        vTaskDelay(10 / portTICK_PERIOD_MS);
        slv_addr = SCCB_Probe();
    }
#endif
#if CONFIG_NT99141_SUPPORT
   if (slv_addr == 0x2a)
    {
        ESP_LOGD(TAG, "Resetting NT99141");
        SCCB_Write16(0x2a, 0x3008, 0x01);//bank sensor
    }
#endif 

    s_state->sensor.slv_addr = slv_addr;
    s_state->sensor.xclk_freq_hz = config->xclk_freq_hz;

#if (CONFIG_OV3660_SUPPORT || CONFIG_OV5640_SUPPORT || CONFIG_NT99141_SUPPORT)
    if(s_state->sensor.slv_addr == 0x3c){
        id->PID = SCCB_Read16(s_state->sensor.slv_addr, REG16_CHIDH);
        id->VER = SCCB_Read16(s_state->sensor.slv_addr, REG16_CHIDL);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        ESP_LOGD(TAG, "Camera PID=0x%02x VER=0x%02x", id->PID, id->VER);
    } else if(s_state->sensor.slv_addr == 0x2a){
        id->PID = SCCB_Read16(s_state->sensor.slv_addr, 0x3000);
        id->VER = SCCB_Read16(s_state->sensor.slv_addr, 0x3001);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        ESP_LOGD(TAG, "Camera PID=0x%02x VER=0x%02x", id->PID, id->VER);
        if(config->xclk_freq_hz > 10000000)
        {
            ESP_LOGE(TAG, "NT99141: only XCLK under 10MHz is supported, and XCLK is now set to 10M");
            s_state->sensor.xclk_freq_hz = 10000000;
        }    
    } else {
#endif
        id->PID = SCCB_Read(s_state->sensor.slv_addr, REG_PID);
        id->VER = SCCB_Read(s_state->sensor.slv_addr, REG_VER);
        id->MIDL = SCCB_Read(s_state->sensor.slv_addr, REG_MIDL);
        id->MIDH = SCCB_Read(s_state->sensor.slv_addr, REG_MIDH);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        ESP_LOGD(TAG, "Camera PID=0x%02x VER=0x%02x MIDL=0x%02x MIDH=0x%02x",
                 id->PID, id->VER, id->MIDH, id->MIDL);

#if (CONFIG_OV3660_SUPPORT || CONFIG_OV5640_SUPPORT || CONFIG_NT99141_SUPPORT)
    }
#endif


    switch (id->PID) {
#if CONFIG_OV2640_SUPPORT
    case OV2640_PID:
        *out_camera_model = CAMERA_OV2640;
        ov2640_init(&s_state->sensor);
        break;
#endif
#if CONFIG_OV7725_SUPPORT
    case OV7725_PID:
        *out_camera_model = CAMERA_OV7725;
        ov7725_init(&s_state->sensor);
        break;
#endif
#if CONFIG_OV3660_SUPPORT
    case OV3660_PID:
        *out_camera_model = CAMERA_OV3660;
        ov3660_init(&s_state->sensor);
        break;
#endif
#if CONFIG_OV5640_SUPPORT
    case OV5640_PID:
        *out_camera_model = CAMERA_OV5640;
        ov5640_init(&s_state->sensor);
        break;
#endif
#if CONFIG_OV7670_SUPPORT
    case OV7670_PID:
        *out_camera_model = CAMERA_OV7670;
        ov7670_init(&s_state->sensor);
        break;
#endif
#if CONFIG_NT99141_SUPPORT
        case NT99141_PID:
        *out_camera_model = CAMERA_NT99141;
        NT99141_init(&s_state->sensor);
        break;
#endif
    default:
        id->PID = 0;
        *out_camera_model = CAMERA_UNKNOWN;
        ESP_LOGE(TAG, "Detected camera not supported.");
        return ESP_ERR_CAMERA_NOT_SUPPORTED;
    }

    ESP_LOGD(TAG, "Doing SW reset of sensor");
    s_state->sensor.reset(&s_state->sensor);

    return ESP_OK;
}

esp_err_t camera_init(const camera_config_t* config)
{
    if (!s_state) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_state->sensor.id.PID == 0) {
        return ESP_ERR_CAMERA_NOT_SUPPORTED;
    }
    memcpy(&s_state->config, config, sizeof(*config));
    esp_err_t err = ESP_OK;
    framesize_t frame_size = (framesize_t) config->frame_size;
    pixformat_t pix_format = (pixformat_t) config->pixel_format;

    switch (s_state->sensor.id.PID) {
#if CONFIG_OV2640_SUPPORT
        case OV2640_PID:
            if (frame_size > FRAMESIZE_UXGA) {
                frame_size = FRAMESIZE_UXGA;
            }
            break;
#endif
#if CONFIG_OV7725_SUPPORT
        case OV7725_PID:
            if (frame_size > FRAMESIZE_VGA) {
                frame_size = FRAMESIZE_VGA;
            }
            break;
#endif
#if CONFIG_OV3660_SUPPORT
        case OV3660_PID:
            if (frame_size > FRAMESIZE_QXGA) {
                frame_size = FRAMESIZE_QXGA;
            }
            break;
#endif
#if CONFIG_OV5640_SUPPORT
        case OV5640_PID:
            if (frame_size > FRAMESIZE_QSXGA) {
                frame_size = FRAMESIZE_QSXGA;
            }
            break;
#endif
#if CONFIG_OV7670_SUPPORT
        case OV7670_PID:
            if (frame_size > FRAMESIZE_VGA) {
                frame_size = FRAMESIZE_VGA;
            }
            break;
#endif
#if CONFIG_NT99141_SUPPORT
        case NT99141_PID:
            if (frame_size > FRAMESIZE_HD) {
                frame_size = FRAMESIZE_HD;
            }
            break;
#endif
        default:
            return ESP_ERR_CAMERA_NOT_SUPPORTED;
    }

    s_state->width = resolution[frame_size].width;
    s_state->height = resolution[frame_size].height;

    if (pix_format == PIXFORMAT_GRAYSCALE) {
        s_state->fb_size = s_state->width * s_state->height;
        if (s_state->sensor.id.PID == OV3660_PID || s_state->sensor.id.PID == OV5640_PID || s_state->sensor.id.PID == NT99141_PID) {
            s_state->in_bytes_per_pixel = 1;       // camera sends Y8
        } else {
            s_state->in_bytes_per_pixel = 2;       // camera sends YU/YV
        }
        s_state->fb_bytes_per_pixel = 1;       // frame buffer stores Y8
    } else if (pix_format == PIXFORMAT_YUV422 || pix_format == PIXFORMAT_RGB565) {
        s_state->fb_size = s_state->width * s_state->height * 2;
        s_state->in_bytes_per_pixel = 2;       // camera sends YU/YV
        s_state->fb_bytes_per_pixel = 2;       // frame buffer stores YU/YV/RGB565
    } else if (pix_format == PIXFORMAT_RGB888) {
        s_state->fb_size = s_state->width * s_state->height * 3;
        s_state->in_bytes_per_pixel = 2;       // camera sends RGB565
        s_state->fb_bytes_per_pixel = 3;       // frame buffer stores RGB888
    } else if (pix_format == PIXFORMAT_JPEG) {
        if (s_state->sensor.id.PID != OV2640_PID && s_state->sensor.id.PID != OV3660_PID && s_state->sensor.id.PID != OV5640_PID  && s_state->sensor.id.PID != NT99141_PID) {
            ESP_LOGE(TAG, "JPEG format is only supported for ov2640, ov3660 and ov5640");
            err = ESP_ERR_NOT_SUPPORTED;
            goto fail;
        }
        int qp = config->jpeg_quality;
        int compression_ratio_bound = 1;
        if (qp > 10) {
            compression_ratio_bound = 16;
        } else if (qp > 5) {
            compression_ratio_bound = 10;
        } else {
            compression_ratio_bound = 4;
        }
        (*s_state->sensor.set_quality)(&s_state->sensor, qp);
        s_state->in_bytes_per_pixel = 2;
        s_state->fb_bytes_per_pixel = 2;
        s_state->fb_size = (s_state->width * s_state->height * s_state->fb_bytes_per_pixel) / compression_ratio_bound;
    } else {
        ESP_LOGE(TAG, "Requested format is not supported");
        err = ESP_ERR_NOT_SUPPORTED;
        goto fail;
    }

    ESP_LOGD(TAG, "in_bpp: %d, fb_bpp: %d, fb_size: %d, width: %d height: %d",
             s_state->in_bytes_per_pixel, s_state->fb_bytes_per_pixel,
             s_state->fb_size,
             s_state->width, s_state->height);

    s_state->sensor.status.framesize = frame_size;
    s_state->sensor.pixformat = pix_format;
    ESP_LOGD(TAG, "Setting frame size to %dx%d", s_state->width, s_state->height);
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

    //todo: for some reason the first set of the quality does not work.
    if (pix_format == PIXFORMAT_JPEG) {
        (*s_state->sensor.set_quality)(&s_state->sensor, config->jpeg_quality);
    }
    s_state->sensor.init_status(&s_state->sensor);

    //lcd_cam_init

    return ESP_OK;

fail:
    esp_camera_deinit();
    return err;
}

static esp_err_t camera_init_state(){
    if(s_state == NULL){
        s_state = (camera_state_t*) calloc(sizeof(*s_state), 1);
        if (!s_state) {
            return ESP_ERR_NO_MEM;
        }
        memset(s_state, 0, sizeof(camera_state_t));
    }
    return ESP_OK;
}

esp_err_t esp_camera_initwith_lcd(const camera_config_t* config, lcd_config_t* lcd, lcd_cam_handle_t ** lcd_cam_handle)
{
    esp_err_t err = camera_init_state();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init state failed with error 0x%x", err);
        return err;
    }
    memcpy(&s_state->lcd_cam_config.lcd, lcd, sizeof(lcd_config_t));

    err = esp_camera_init(config);
    if (err != ESP_OK) {
        return err;
    }
    *lcd_cam_handle = &s_state->lcd_cam;
    return ESP_OK;
}

esp_err_t esp_camera_init(const camera_config_t* config)
{
    camera_model_t camera_model = CAMERA_NONE;
    esp_err_t err = camera_init_state();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init state failed with error 0x%x", err);
        return err;
    }
    
    //setup lcd_cam
    cam_config_t cam = {
        .en = true,
        .width = 8,
        .fre = config->xclk_freq_hz,
        .pin = {
            .xclk  = config->pin_xclk,
            .pclk  = config->pin_pclk,
            .vsync = config->pin_vsync,
            .href = config->pin_href,
            .data = {config->pin_d0, config->pin_d1, config->pin_d2, config->pin_d3, config->pin_d4, config->pin_d5, config->pin_d6, config->pin_d7},
        },
        .invert = {
            .xclk  = false,
            .pclk  = false,
            .vsync = true,
            .href = false,
            .data = {false, false, false, false, false, false, false, false},
        },
        .mode.jpeg = (config->pixel_format == PIXFORMAT_JPEG),
        .recv_size = resolution[config->frame_size].width * resolution[config->frame_size].height * 2,
        .max_dma_buffer_size = 16 * 1024,
        .frame_cnt = 2, 
        .frame_caps = MALLOC_CAP_SPIRAM,
        .task_stack = 1024,
        .task_pri = configMAX_PRIORITIES,
        .swap_data = false
    };
    if(config->pixel_format == PIXFORMAT_JPEG) {
        cam.recv_size /= 8;
    }
    memcpy(&s_state->lcd_cam_config.cam, &cam, sizeof(cam_config_t));

    err = lcd_cam_init(&s_state->lcd_cam, &s_state->lcd_cam_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LCD-Camera init failed with error 0x%x", err);
        return err;
    }

    err = camera_probe(config, &camera_model);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera probe failed with error 0x%x", err);
        goto fail;
    }
    if (camera_model == CAMERA_OV7725) {
        ESP_LOGI(TAG, "Detected OV7725 camera");
        if(config->pixel_format == PIXFORMAT_JPEG) {
            ESP_LOGE(TAG, "Camera does not support JPEG");
            err = ESP_ERR_CAMERA_NOT_SUPPORTED;
            goto fail;
        }
    } else if (camera_model == CAMERA_OV2640) {
        ESP_LOGI(TAG, "Detected OV2640 camera");
    } else if (camera_model == CAMERA_OV3660) {
        ESP_LOGI(TAG, "Detected OV3660 camera");
    } else if (camera_model == CAMERA_OV5640) {
        ESP_LOGI(TAG, "Detected OV5640 camera");
    } else if (camera_model == CAMERA_OV7670) {
        ESP_LOGI(TAG, "Detected OV7670 camera");
    } else if (camera_model == CAMERA_NT99141) {
        ESP_LOGI(TAG, "Detected NT99141 camera");
    } else {
        ESP_LOGI(TAG, "Camera not supported");
        err = ESP_ERR_CAMERA_NOT_SUPPORTED;
        goto fail;
    }
    err = camera_init(config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return err;
    }

    s_state->lcd_cam.cam.start();
    return ESP_OK;

fail:
    free(s_state);
    s_state = NULL;
    return err;
}

esp_err_t esp_camera_deinit()
{
    if (s_state == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    lcd_cam_deinit(&s_state->lcd_cam);
    free(s_state);
    s_state = NULL;
    return ESP_OK;
}

camera_fb_t* esp_camera_fb_get()
{
    if (s_state == NULL) {
        return NULL;
    }
    camera_fb_t* fb = (camera_fb_t*)malloc(sizeof(camera_fb_t));
    if(!fb){
        return NULL;
    }
    fb->format = s_state->sensor.pixformat;
    fb->width = resolution[s_state->sensor.status.framesize].width;
    fb->height = resolution[s_state->sensor.status.framesize].height;
    fb->len = s_state->lcd_cam.cam.take(&fb->buf);
    uint64_t us = (uint64_t)esp_timer_get_time();
    fb->timestamp.tv_sec = us / 1000000UL;
    fb->timestamp.tv_usec = us % 1000000UL;
    return fb;
}

void esp_camera_fb_return(camera_fb_t * fb)
{
    if(fb == NULL) {
        return;
    }
    if(s_state != NULL) {
        s_state->lcd_cam.cam.give(fb->buf);
    }
    free(fb);
}

sensor_t * esp_camera_sensor_get()
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
    esp_err_t ret = nvs_open(key,NVS_READWRITE,&handle);
    
    if (ret == ESP_OK) {
        sensor_t *s = esp_camera_sensor_get();
        if (s != NULL) {
            ret = nvs_set_blob(handle,CAMERA_SENSOR_NVS_KEY,&s->status,sizeof(camera_status_t));
            if (ret == ESP_OK) {
                uint8_t pf = s->pixformat;
                ret = nvs_set_u8(handle,CAMERA_PIXFORMAT_NVS_KEY,pf);
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

  esp_err_t ret = nvs_open(key,NVS_READWRITE,&handle);
  
  if (ret == ESP_OK) {
      sensor_t *s = esp_camera_sensor_get();
      camera_status_t st;
      if (s != NULL) {
        size_t size = sizeof(camera_status_t);
        ret = nvs_get_blob(handle,CAMERA_SENSOR_NVS_KEY,&st,&size);
        if (ret == ESP_OK) {
            s->set_ae_level(s,st.ae_level);
            s->set_aec2(s,st.aec2);
            s->set_aec_value(s,st.aec_value);
            s->set_agc_gain(s,st.agc_gain);
            s->set_awb_gain(s,st.awb_gain);
            s->set_bpc(s,st.bpc);
            s->set_brightness(s,st.brightness);
            s->set_colorbar(s,st.colorbar);
            s->set_contrast(s,st.contrast);
            s->set_dcw(s,st.dcw);
            s->set_denoise(s,st.denoise);
            s->set_exposure_ctrl(s,st.aec);
            s->set_framesize(s,st.framesize);
            s->set_gain_ctrl(s,st.agc);          
            s->set_gainceiling(s,st.gainceiling);
            s->set_hmirror(s,st.hmirror);
            s->set_lenc(s,st.lenc);
            s->set_quality(s,st.quality);
            s->set_raw_gma(s,st.raw_gma);
            s->set_saturation(s,st.saturation);
            s->set_sharpness(s,st.sharpness);
            s->set_special_effect(s,st.special_effect);
            s->set_vflip(s,st.vflip);
            s->set_wb_mode(s,st.wb_mode);
            s->set_whitebal(s,st.awb);
            s->set_wpc(s,st.wpc);
        }  
        ret = nvs_get_u8(handle,CAMERA_PIXFORMAT_NVS_KEY,&pf);
        if (ret == ESP_OK) {
          s->set_pixformat(s,pf);
        }
      } else {
          return ESP_ERR_CAMERA_NOT_DETECTED;
      }
      nvs_close(handle);
      return ret;
  } else {
      ESP_LOGW(TAG,"Error (%d) opening nvs key \"%s\"",ret,key);
      return ret;
  }
}
