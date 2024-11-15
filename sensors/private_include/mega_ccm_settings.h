#ifndef _GC032A_SETTINGS_H_
#define _GC032A_SETTINGS_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_attr.h"
#include "mega_ccm_regs.h"


#define REG_DLY 0xffff
#define REGLIST_TAIL 0x0000

static const uint16_t mega_ccm_default_regs[][2] = {
    {0x0120,      0x01 }, // JPEG 
    {0x0121,      0x01 }, // 320X240
    {REGLIST_TAIL, 0x00},
};

#endif
