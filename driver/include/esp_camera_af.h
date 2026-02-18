#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Auto-focus modes.
 *
 * - AUTO: sensor runs its internal AF routine (continuous, if supported).
 * - MANUAL: AF is not running automatically; user triggers focus explicitly.
 */
typedef enum {
    ESP_CAMERA_AF_MODE_AUTO = 0,
    ESP_CAMERA_AF_MODE_MANUAL = 1,
} esp_camera_af_mode_t;

/**
 * @brief Auto-focus configuration.
 *
 * Notes:
 * - Some fields are sensor-dependent (e.g., manual position control).
 * - For sensors without AF hardware/firmware support, APIs return ESP_ERR_NOT_SUPPORTED.
 */
typedef struct {
    esp_camera_af_mode_t mode;

    /**
     * @brief Step size for manual stepping algorithms.
     *
     * Used by software search helpers and sensors that expose manual lens control.
     */
    uint16_t step_size;

    /**
     * @brief Inclusive focus range limits for manual stepping algorithms.
     */
    uint16_t range_min;
    uint16_t range_max;

    /**
     * @brief Default timeout used by AF operations, in milliseconds.
     */
    uint32_t timeout_ms;
} esp_camera_af_config_t;

/**
 * @brief AF status values (sensor-specific raw values).
 */
typedef struct {
    uint8_t raw;
    bool focused;
    bool busy;
} esp_camera_af_status_t;

/**
 * @brief Return whether the attached sensor supports AF through this module.
 */
bool esp_camera_af_is_supported(const sensor_t *sensor);

/**
 * @brief Initialize AF support for the attached sensor.
 */
esp_err_t esp_camera_af_init(sensor_t *sensor, const esp_camera_af_config_t *config);

/**
 * @brief Change AF mode.
 */
esp_err_t esp_camera_af_set_mode(sensor_t *sensor, esp_camera_af_mode_t mode);

/**
 * @brief Trigger a single autofocus cycle (if supported).
 */
esp_err_t esp_camera_af_trigger(sensor_t *sensor);

/**
 * @brief Read current AF status.
 */
esp_err_t esp_camera_af_get_status(sensor_t *sensor, esp_camera_af_status_t *out_status);

/**
 * @brief Wait for AF to finish or report focused.
 *
 * @param timeout_ms If 0, uses config timeout passed to esp_camera_af_init().
 */
esp_err_t esp_camera_af_wait(sensor_t *sensor, uint32_t timeout_ms, esp_camera_af_status_t *out_status);

/**
 * @brief Set manual lens position.
 *
 * Some sensors (including OV5640 when using internal AF firmware only) may not expose a safe
 * manual lens position register via the public driver; in those cases this returns ESP_ERR_NOT_SUPPORTED.
 */
esp_err_t esp_camera_af_set_manual_position(sensor_t *sensor, uint16_t position);

#ifdef __cplusplus
}
#endif
