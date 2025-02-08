/*
 * HM0360 driver.
 */
#ifndef __HM0360_H__
#define __HM0360_H__
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
int hm0360_detect(int slv_addr, sensor_id_t *id);

/**
 * @brief initialize sensor function pointers
 *
 * @param sensor pointer of sensor
 * @return
 *      Always 0
 */
int hm0360_init(sensor_t *sensor);

#endif // __HM1055_H__
