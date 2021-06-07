/*
 *
 * GC032A driver.
 *
 */
#ifndef __GC032A_H__
#define __GC032A_H__

#include "sensor.h"

/**
 * @brief detect sensor pid
 *
 * @param slv_addr sccb address
 * @return
 *     0:       can't detect this sensor
 *     Nonzero: sensor pid
 */
int gc032a_detect(int slv_addr);

/**
 * @brief initialize sensor function pointers
 *
 * @param sensor pointer of sensor
 * @return
 *      Always 0
 */
int gc032a_init(sensor_t *sensor);

#endif // __GC032A_H__
