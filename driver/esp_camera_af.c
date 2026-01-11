#include "sdkconfig.h"

#include "esp_camera_af.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#ifndef CONFIG_CAMERA_AF_DEFAULT_TIMEOUT_MS
#define CONFIG_CAMERA_AF_DEFAULT_TIMEOUT_MS 2000
#endif

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#define TAG ""
#else
#include "esp_log.h"
static const char *TAG = "camera_af";
#endif

#if defined(CONFIG_CAMERA_AF_SUPPORT) && CONFIG_CAMERA_AF_SUPPORT

#if defined(CONFIG_OV5640_SUPPORT) && CONFIG_OV5640_SUPPORT
#include "ov5640_af_firmware.h"
#endif

#define OV5640_CHIPID_HIGH 0x300a
#define OV5640_CHIPID_LOW  0x300b

#define OV5640_CMD_MAIN      0x3022
#define OV5640_CMD_ACK       0x3023
#define OV5640_CMD_PARA0     0x3024
#define OV5640_CMD_PARA1     0x3025
#define OV5640_CMD_PARA2     0x3026
#define OV5640_CMD_PARA3     0x3027
#define OV5640_CMD_PARA4     0x3028
#define OV5640_CMD_FW_STATUS 0x3029

#define OV5640_AF_TRIG_SINGLE 0x03
#define OV5640_AF_CONTINUOUS  0x04

#define OV5640_FW_STATUS_IDLE    0x70
#define OV5640_FW_STATUS_FOCUSED 0x10

static uint32_t s_default_timeout_ms = CONFIG_CAMERA_AF_DEFAULT_TIMEOUT_MS;

static bool ov5640_is_sensor(const sensor_t *sensor)
{
    if (!sensor) {
        return false;
    }

    if (sensor->id.PID == OV5640_PID) {
        return true;
    }

    if (sensor->get_reg) {
        int vid = sensor->get_reg((sensor_t *)sensor, OV5640_CHIPID_HIGH, 0xff);
        int pid = sensor->get_reg((sensor_t *)sensor, OV5640_CHIPID_LOW, 0xff);
        return (vid == 0x56) && (pid == 0x40);
    }

    return false;
}

static esp_err_t ov5640_reg_write(sensor_t *sensor, int reg, int value)
{
    if (sensor->set_reg(sensor, reg, 0xff, value) < 0) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t ov5640_reg_read(sensor_t *sensor, int reg, int *out_value)
{
    int v = sensor->get_reg(sensor, reg, 0xff);
    if (v < 0) {
        return ESP_FAIL;
    }
    *out_value = v;
    return ESP_OK;
}

static esp_err_t ov5640_af_wait_ack_clear(sensor_t *sensor, uint32_t timeout_ms)
{
    const uint64_t start = esp_timer_get_time();
    while (true) {
        int ack = 0;
        if (ov5640_reg_read(sensor, OV5640_CMD_ACK, &ack) != ESP_OK) {
            return ESP_FAIL;
        }
        if (ack == 0x00) {
            return ESP_OK;
        }
        if (timeout_ms && ((esp_timer_get_time() - start) / 1000ULL) > timeout_ms) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static esp_err_t ov5640_af_wait_fw_idle(sensor_t *sensor, uint32_t timeout_ms)
{
    const uint64_t start = esp_timer_get_time();
    while (true) {
        int st = 0;
        if (ov5640_reg_read(sensor, OV5640_CMD_FW_STATUS, &st) != ESP_OK) {
            return ESP_FAIL;
        }
        if ((uint8_t)st == OV5640_FW_STATUS_IDLE) {
            return ESP_OK;
        }
        if (timeout_ms && ((esp_timer_get_time() - start) / 1000ULL) > timeout_ms) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static esp_err_t ov5640_af_firmware_load(sensor_t *sensor, uint32_t timeout_ms)
{
#if !(defined(CONFIG_OV5640_SUPPORT) && CONFIG_OV5640_SUPPORT)
    (void)sensor;
    (void)timeout_ms;
    return ESP_ERR_NOT_SUPPORTED;
#else
    esp_err_t ret = ov5640_reg_write(sensor, 0x3000, 0x20); // reset MCU
    if (ret != ESP_OK) {
        return ret;
    }

    uint16_t addr = 0x8000;
    for (size_t i = 0; i < sizeof(ov5640_af_firmware); i++) {
        ret = ov5640_reg_write(sensor, addr, ov5640_af_firmware[i]);
        if (ret != ESP_OK) {
            return ret;
        }
        addr++;
    }

    (void)ov5640_reg_write(sensor, OV5640_CMD_MAIN, 0x00);
    (void)ov5640_reg_write(sensor, OV5640_CMD_ACK, 0x00);
    (void)ov5640_reg_write(sensor, OV5640_CMD_PARA0, 0x00);
    (void)ov5640_reg_write(sensor, OV5640_CMD_PARA1, 0x00);
    (void)ov5640_reg_write(sensor, OV5640_CMD_PARA2, 0x00);
    (void)ov5640_reg_write(sensor, OV5640_CMD_PARA3, 0x00);
    (void)ov5640_reg_write(sensor, OV5640_CMD_PARA4, 0x00);

    (void)ov5640_reg_write(sensor, OV5640_CMD_FW_STATUS, 0x7f);
    (void)ov5640_reg_write(sensor, 0x3000, 0x00); // start MCU

    return ov5640_af_wait_fw_idle(sensor, timeout_ms);
#endif
}

static esp_err_t ov5640_af_start(sensor_t *sensor, bool continuous, uint32_t timeout_ms)
{
    esp_err_t ret = ov5640_reg_write(sensor, OV5640_CMD_MAIN, 0x01);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = ov5640_reg_write(sensor, OV5640_CMD_MAIN, 0x08);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = ov5640_af_wait_ack_clear(sensor, timeout_ms);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = ov5640_reg_write(sensor, OV5640_CMD_ACK, 0x01);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = ov5640_reg_write(sensor, OV5640_CMD_MAIN, continuous ? OV5640_AF_CONTINUOUS : OV5640_AF_TRIG_SINGLE);
    if (ret != ESP_OK) {
        return ret;
    }

    return ov5640_af_wait_ack_clear(sensor, timeout_ms);
}

bool esp_camera_af_is_supported(const sensor_t *sensor)
{
#if defined(CONFIG_OV5640_SUPPORT) && CONFIG_OV5640_SUPPORT
    return ov5640_is_sensor(sensor);
#else
    (void)sensor;
    return false;
#endif
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

    esp_err_t ret = ov5640_af_firmware_load(sensor, s_default_timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AF firmware init failed: %s", esp_err_to_name(ret));
        return ret;
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

    switch (mode) {
    case ESP_CAMERA_AF_MODE_AUTO:
        return ov5640_af_start(sensor, true, s_default_timeout_ms);
    case ESP_CAMERA_AF_MODE_MANUAL:
        return ov5640_reg_write(sensor, OV5640_CMD_MAIN, 0x00);
    default:
        return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t esp_camera_af_trigger(sensor_t *sensor)
{
    if (!sensor) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!esp_camera_af_is_supported(sensor)) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    return ov5640_af_start(sensor, false, s_default_timeout_ms);
}

esp_err_t esp_camera_af_get_status(sensor_t *sensor, esp_camera_af_status_t *out_status)
{
    if (!sensor || !out_status) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!esp_camera_af_is_supported(sensor)) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    int st = 0;
    esp_err_t ret = ov5640_reg_read(sensor, OV5640_CMD_FW_STATUS, &st);
    if (ret != ESP_OK) {
        return ret;
    }

    memset(out_status, 0, sizeof(*out_status));
    out_status->raw = (uint8_t)st;
    out_status->focused = ((uint8_t)st == OV5640_FW_STATUS_FOCUSED);
    out_status->busy = !(((uint8_t)st == OV5640_FW_STATUS_IDLE) || out_status->focused);
    return ESP_OK;
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
    (void)position;
    if (!sensor) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!esp_camera_af_is_supported(sensor)) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    // Manual lens position is not exposed safely in this driver yet.
    return ESP_ERR_NOT_SUPPORTED;
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
