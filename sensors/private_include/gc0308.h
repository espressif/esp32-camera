#pragma once

#include "sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief detect sensor pid
 *
 * @param slv_addr sccb address
 * @return
 *     0:       can't detect this sensor
 *     Nonzero: sensor pid
 */
int gc0308_detect(int slv_addr);

/**
 * @brief initialize sensor function pointers
 *
 * @param sensor pointer of sensor
 * @return
 *      Always 0
 */
int gc0308_init(sensor_t *sensor);

#ifdef __cplusplus
}
#endif
