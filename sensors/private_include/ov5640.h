
#ifndef __OV5640_H__
#define __OV5640_H__

#include "sensor.h"

/**
 * @brief Detect sensor pid
 *
 * @param slv_addr SCCB address
 * @param id Detection result
 * @return
 *     0:       Can't detect this sensor
 *     Nonzero: This sensor has been detected
 */
int esp32_camera_ov5640_detect(int slv_addr, sensor_id_t *id);

/**
 * @brief initialize sensor function pointers
 *
 * @param sensor pointer of sensor
 * @return
 *      Always 0
 */
int esp32_camera_ov5640_init(sensor_t *sensor);

// Autofocus function implementations (in ov5640_af.c)
#if defined(CONFIG_CAMERA_AF_SUPPORT) && CONFIG_CAMERA_AF_SUPPORT
int ov5640_af_is_supported(sensor_t *sensor);
int ov5640_af_init(sensor_t *sensor, uint32_t timeout_ms);
int ov5640_af_set_mode(sensor_t *sensor, int mode);
int ov5640_af_trigger(sensor_t *sensor);
int ov5640_af_get_status(sensor_t *sensor, uint8_t *out_raw, bool *out_focused, bool *out_busy);
int ov5640_af_set_manual_position(sensor_t *sensor, uint16_t position);
#endif

#endif // __OV5640_H__
