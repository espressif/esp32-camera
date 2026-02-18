/**
 * This example takes a picture every 5s and print its size on serial monitor.
 */

// =============================== SETUP ======================================

// 1. Board setup (Uncomment):
// #define BOARD_WROVER_KIT
// #define BOARD_ESP32CAM_AITHINKER
// #define BOARD_ESP32S3_WROOM
// #define BOARD_ESP32S3_XIAO
// #define BOARD_ESP32S3_GOOUUU
// #define BOARD_ESP32S3_XIAO

/**
 * 2. Kconfig setup
 *
 * If you have a Kconfig file, copy the content from
 *  https://github.com/espressif/esp32-camera/blob/master/Kconfig into it.
 * In case you haven't, copy and paste this Kconfig file inside the src directory.
 * This Kconfig file has definitions that allows more control over the camera and
 * how it will be initialized.
 */

/**
 * 3. Enable PSRAM on sdkconfig:
 *
 * CONFIG_ESP32_SPIRAM_SUPPORT=y
 *
 * More info on
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/kconfig.html#config-esp32-spiram-support
 */

// ================================ CODE ======================================

#include "sdkconfig.h"

#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

#include "esp_camera.h"

#if defined(CONFIG_CAMERA_AF_SUPPORT) && CONFIG_CAMERA_AF_SUPPORT
#include "esp_camera_af.h"
#endif

#define BOARD_WROVER_KIT 1

#include "camera_pinout.h"

static const char *TAG = "example:take_picture";

#if ESP_CAMERA_SUPPORTED
static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_RGB565, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    .jpeg_quality = 12, //0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 1,       //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

static esp_err_t init_camera(void)
{
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}

#if defined(CONFIG_CAMERA_AF_SUPPORT) && CONFIG_CAMERA_AF_SUPPORT
static void maybe_init_autofocus(void)
{
    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        ESP_LOGW(TAG, "AF: no sensor handle");
        return;
    }

    if (!esp_camera_af_is_supported(s)) {
        ESP_LOGI(TAG, "AF: not supported by this sensor");
        return;
    }

    esp_camera_af_config_t af_cfg = {
        .mode = ESP_CAMERA_AF_MODE_AUTO,
        .timeout_ms = CONFIG_CAMERA_AF_DEFAULT_TIMEOUT_MS,
    };

    esp_err_t ret = esp_camera_af_init(s, &af_cfg);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "AF init failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "AF initialized (AUTO mode)");
}
#endif
#endif

void app_main(void)
{
#if ESP_CAMERA_SUPPORTED
    if(ESP_OK != init_camera()) {
        return;
    }

#if defined(CONFIG_CAMERA_AF_SUPPORT) && CONFIG_CAMERA_AF_SUPPORT
    // Initialize autofocus if configured and supported by the sensor.
    // In menuconfig: Component config → Camera configuration → Enable autofocus support
    maybe_init_autofocus();
#endif

    while (1)
    {
        ESP_LOGI(TAG, "Taking picture...");
        camera_fb_t *pic = esp_camera_fb_get();

        // use pic->buf to access the image
        ESP_LOGI(TAG, "Picture taken! Its size was: %zu bytes", pic->len);
        esp_camera_fb_return(pic);

        vTaskDelay(5000 / portTICK_RATE_MS);
    }
#else
    ESP_LOGE(TAG, "Camera support is not available for this chip");
    return;
#endif
}
