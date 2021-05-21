#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "xclk.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
static const char* TAG = "camera_xclk";
#endif

esp_err_t xclk_timer_conf(int ledc_timer, int xclk_freq_hz)
{
    ledc_timer_config_t timer_conf;
    timer_conf.duty_resolution = LEDC_TIMER_1_BIT;
    timer_conf.freq_hz = xclk_freq_hz;
#if CONFIG_IDF_TARGET_ESP32
    timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
#else
    timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
#endif
#if ESP_IDF_VERSION_MAJOR >= 4
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
#endif
    timer_conf.timer_num = (ledc_timer_t)ledc_timer;
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed for freq %d, rc=%x", xclk_freq_hz, err);
    }
    return err;
}

esp_err_t camera_enable_out_clock(camera_config_t* config)
{
    periph_module_enable(PERIPH_LEDC_MODULE);

    esp_err_t err = xclk_timer_conf(config->ledc_timer, config->xclk_freq_hz);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed, rc=%x", err);
        return err;
    }

    ledc_channel_config_t ch_conf;
    ch_conf.gpio_num = config->pin_xclk;
#if CONFIG_IDF_TARGET_ESP32
    ch_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
#else
    ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
#endif
    ch_conf.channel = config->ledc_channel;
    ch_conf.intr_type = LEDC_INTR_DISABLE;
    ch_conf.timer_sel = config->ledc_timer;
    ch_conf.duty = 1;
    ch_conf.hpoint = 0;
    err = ledc_channel_config(&ch_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed, rc=%x", err);
        return err;
    }
    return ESP_OK;
}

void camera_disable_out_clock()
{
    periph_module_disable(PERIPH_LEDC_MODULE);
}
