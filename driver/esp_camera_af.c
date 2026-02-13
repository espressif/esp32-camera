#include "sdkconfig.h"

#include "esp_camera_af.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#ifndef CONFIG_CAMERA_AF_DEFAULT_TIMEOUT_MS
#define CONFIG_CAMERA_AF_DEFAULT_TIMEOUT_MS 2000
#endif

#if defined(CONFIG_CAMERA_AF_SUPPORT) && CONFIG_CAMERA_AF_SUPPORT

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#define TAG ""
#else
#include "esp_log.h"
#if CONFIG_LOG_DEFAULT_LEVEL > 0
static const char *TAG = "camera_af";
#endif
#endif

static uint32_t s_default_timeout_ms = CONFIG_CAMERA_AF_DEFAULT_TIMEOUT_MS;

bool esp_camera_af_is_supported(const sensor_t *sensor)
{
    if (!sensor || !sensor->af_is_supported) {
        return false;
    }
    return sensor->af_is_supported((sensor_t *)sensor) != 0;
}

esp_err_t esp_camera_af_init(sensor_t *sensor, const esp_camera_af_config_t *config)
{
    if (!sensor || !config) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->timeout_ms) {
        s_default_timeout_ms = config->timeout_ms;
    }

    if (!esp_camera_af_is_supported(sensor)) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (!sensor->af_init) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    int ret = sensor->af_init(sensor, s_default_timeout_ms);
    if (ret < 0) {
        ESP_LOGE(TAG, "AF init failed");
        return ESP_FAIL;
    }

    return esp_camera_af_set_mode(sensor, config->mode);
}

esp_err_t esp_camera_af_set_mode(sensor_t *sensor, esp_camera_af_mode_t mode)
{
    if (!sensor) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!esp_camera_af_is_supported(sensor)) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (!sensor->af_set_mode) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    int ret = sensor->af_set_mode(sensor, (int)mode);
    return (ret == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t esp_camera_af_trigger(sensor_t *sensor)
{
    if (!sensor) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!esp_camera_af_is_supported(sensor)) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (!sensor->af_trigger) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    int ret = sensor->af_trigger(sensor);
    return (ret == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t esp_camera_af_get_status(sensor_t *sensor, esp_camera_af_status_t *out_status)
{
    if (!sensor || !out_status) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!esp_camera_af_is_supported(sensor)) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (!sensor->af_get_status) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    memset(out_status, 0, sizeof(*out_status));
    int ret = sensor->af_get_status(sensor, &out_status->raw, &out_status->focused, &out_status->busy);
    return (ret == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t esp_camera_af_wait(sensor_t *sensor, uint32_t timeout_ms, esp_camera_af_status_t *out_status)
{
    if (!sensor || !out_status) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!esp_camera_af_is_supported(sensor)) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (!timeout_ms) {
        timeout_ms = s_default_timeout_ms;
    }

    const uint64_t start = esp_timer_get_time();
    while (true) {
        esp_err_t ret = esp_camera_af_get_status(sensor, out_status);
        if (ret != ESP_OK) {
            return ret;
        }

        if (!out_status->busy) {
            return ESP_OK;
        }

        if (timeout_ms && ((esp_timer_get_time() - start) / 1000ULL) > timeout_ms) {
            return ESP_ERR_TIMEOUT;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

esp_err_t esp_camera_af_set_manual_position(sensor_t *sensor, uint16_t position)
{
    if (!sensor) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!esp_camera_af_is_supported(sensor)) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (!sensor->af_set_manual_position) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    int ret = sensor->af_set_manual_position(sensor, position);
    return (ret == 0) ? ESP_OK : ESP_FAIL;
}

#else // CONFIG_CAMERA_AF_SUPPORT

bool esp_camera_af_is_supported(const sensor_t *sensor)
{
    (void)sensor;
    return false;
}

esp_err_t esp_camera_af_init(sensor_t *sensor, const esp_camera_af_config_t *config)
{
    (void)sensor;
    (void)config;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t esp_camera_af_set_mode(sensor_t *sensor, esp_camera_af_mode_t mode)
{
    (void)sensor;
    (void)mode;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t esp_camera_af_trigger(sensor_t *sensor)
{
    (void)sensor;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t esp_camera_af_get_status(sensor_t *sensor, esp_camera_af_status_t *out_status)
{
    (void)sensor;
    (void)out_status;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t esp_camera_af_wait(sensor_t *sensor, uint32_t timeout_ms, esp_camera_af_status_t *out_status)
{
    (void)sensor;
    (void)timeout_ms;
    (void)out_status;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t esp_camera_af_set_manual_position(sensor_t *sensor, uint16_t position)
{
    (void)sensor;
    (void)position;
    return ESP_ERR_NOT_SUPPORTED;
}

#endif // CONFIG_CAMERA_AF_SUPPORT
