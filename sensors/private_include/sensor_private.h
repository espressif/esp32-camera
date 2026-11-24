#ifndef _SENSOR_PRIVATE_H_
#define _SENSOR_PRIVATE_H_

#include "esp_attr.h"
#include "sdkconfig.h"

// Attribute for the sensor settings tables in *_settings.h.
// By default the tables are in flash, to save internal RAM.
// This is safe because they are only read when configuring the sensor, not from interrupts.
// Enable CONFIG_CAMERA_SENSOR_SETTINGS_IN_DRAM to put them in internal RAM.
#if CONFIG_CAMERA_SENSOR_SETTINGS_IN_DRAM
#define SENSOR_SETTINGS_ATTR DRAM_ATTR
#else
#define SENSOR_SETTINGS_ATTR
#endif

#endif // _SENSOR_PRIVATE_H_
