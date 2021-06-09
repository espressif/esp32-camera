
#ifndef __GC2145_H__
#define __GC2145_H__

#include "sensor.h"

int gc2145_detect(int slv_addr);

int gc2145_init(sensor_t *sensor);

#endif // __GC2145_H__
