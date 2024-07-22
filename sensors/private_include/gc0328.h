#pragma once

#include "sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Detect sensor pid
 *
 * @param slv_addr SCCB address
 * @param id Detection result
 * @return
 *     0:       Can't detect this sensor
 *     Nonzero: This sensor has been detected
 */
int gc0328_detect(int slv_addr, sensor_id_t *id);

/**
 * @brief initialize sensor function pointers
 *
 * @param sensor pointer of sensor
 * @return
 *      Always 0
 */
int gc0328_init(sensor_t *sensor);

/**
 * @brief antiflicker setting
 * 
 * @param sensor pointer of sensor
 * @param mode 0: 50Hz, 1: 60Hz
 * 
 */
void gc0328_antiflicker(sensor_t* sensor, int mode);


/**
 * @brief  read exposure value of camera
 * @param sensor pointer of sensor
 * 
 * @return exposure value (0~1023)
 */
int gc0328_get_aec_value(sensor_t *sensor);


#ifdef __cplusplus
}
#endif
