
#ifndef __BF20A6_H__
#define __BF20A6_H__

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
int esp32_camera_bf20a6_detect(int slv_addr, sensor_id_t *id);

/**
 * @brief initialize sensor function pointers
 *
 * @param sensor pointer of sensor
 * @return
 *      Always 0
 */
int esp32_camera_bf20a6_init(sensor_t *sensor);

#endif // __BF20A6_H__
