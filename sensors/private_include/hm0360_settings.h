
#include <stdint.h>

#define REG_DLY 0xffff
#define REGLIST_TAIL 0x0000

static const DRAM_ATTR uint16_t sensor_default_regs[][2] = {
    {0x0103, 0x00},
    {REG_DLY, 100},
    {0x3035, 0x01},
    {0x3126, 0x03},
    {0x3128, 0x57},
    {0x3142, 0x9F},
    {0x311C, 0x10},
    {0x3115, 0x42},
    {0x3116, 0x10},
    {0x3117, 0x0A},
    {0x310B, 0x10},
    {0x3031, 0x01},
    {0x312E, 0x00},
    {0x30D7, 0x00},
    {0x30DC, 0x00},
    {0x30E1, 0x00},
    {0x30E6, 0x00},
    {0x30EB, 0x00},
    {0x30F0, 0x00},
    {0x30F5, 0x00},
    {0x30FA, 0x00},
    {0x30FF, 0x00},
    {0x3104, 0x00},
    {0x30D8, 0xA7},
    {0x30DD, 0x27},
    {0x30E2, 0x27},
    {0x30E7, 0x27},
    {0x30EC, 0x27},
    {0x30F1, 0xA7},
    {0x30F6, 0x27},
    {0x30FB, 0x27},
    {0x3100, 0x27},
    {0x3105, 0x27},
    {0x30D9, 0x00}, // 220215
    {0x30DE, 0x05},
    {0x30E3, 0x05},
    {0x30E8, 0x05},
    {0x30ED, 0x05},
    {0x30F2, 0x00}, // 220215
    {0x30F7, 0x05},
    {0x30FC, 0x05},
    {0x3101, 0x05},
    {0x3106, 0x05},
    {0x30C4, 0x10},
    {0x30C5, 0x01},
    {0x30C6, 0x2F}, // 220215
    {0x30CB, 0xFF},
    {0x30CC, 0xFF},
    {0x30CD, 0x7F},
    {0x30CE, 0x7F},
    {0x30D3, 0x01},
    {0x30D4, 0xFF},
    {0x311F, 0x0E},
    {0x3120, 0x0D},
    {0x3121, 0x0F},
    {0x3122, 0x00},
    {0x3147, 0x18},
    {0x314B, 0x01},
    {0x3149, 0x28},
    {0x3150, 0x50},
    {0x3152, 0x00},
    {0x3156, 0x2C},
    {0x3163, 0x1F},
    {0x3165, 0x7F},
    {0x312B, 0x41},
    {0x3113, 0xA0},
    {0x3114, 0x67},
    {0x317D, 0x02},
    {0x3160, 0x1F},
    {0x318C, 0x00},
    {0x315C, 0xE0},
    {0x311E, 0x0F},
    {0x30D5, 0x00},
    {0x30DA, 0x01},
    {0x30DF, 0x07},
    {0x30E4, 0x47},
    {0x30E9, 0x87},
    {0x30FD, 0x47},
    {0x3102, 0x87},
    {0x311D, 0x06},
    {0x3141, 0x2A},
    {0x315A, 0x0A},
    {0x30D6, 0x40},
    {0x30DB, 0x40},
    {0x30E0, 0x40},
    {0x30E5, 0x30},
    {0x30EA, 0x30},
    {0x30EF, 0x40},
    {0x30F4, 0x40},
    {0x30F9, 0x40},
    {0x30FE, 0x30},
    {0x3103, 0x30},
    {0x3164, 0x7F},
    {0x282A, 0x0F},
    {0x282E, 0x2F},
    {0x282B, 0x08},
    {0x312A, 0x11},
    {0x1000, 0x43},
    {0x1001, 0x80},
    {0x0350, 0xE0},
    {0x101D, 0xCF},
    {0x1021, 0x5D},

    // setA VGA
    {0x3500, 0x74},
    {0x3501, 0x0A},
    {0x3502, 0x77},
    {0x3503, 0x04},
    {0x3504, 0x14},
    {0x3505, 0x03},
    {0x3506, 0x00},
    {0x3507, 0x00},
    {0x3508, 0x00},
    {0x3509, 0x00},
    {0x350A, 0xFF},
    {0x350B, 0x00},
    {0x350D, 0x01},
    {0x350C, 0x00},
    {0x350F, 0x00},
    {0x3510, 0x01},
    {0x3513, 0x00},
    {0x351C, 0x00},
    {0x3514, 0x00},
    {0x3515, 0x01},
    {0x3516, 0x00},
    {0x3517, 0x02},
    {0x3518, 0x00},
    {0x3519, 0x7F},
    {0x351A, 0x00},
    {0x351B, 0x5F},
    {0x351D, 0x04},
    {0x351E, 0x10},
    {0x352A, 0x01},
    {0x352B, 0x04},
    {0x352C, 0x01},
    {0x352D, 0x38},
    {0x354B, 0x21},
    {0x354C, 0x01},
    {0x354D, 0xE0},
    {0x354E, 0xF0},
    {0x354F, 0x10},
    {0x3550, 0x10},
    {0x3551, 0x10},
    {0x3552, 0x20},
    {0x3553, 0x10},
    {0x3554, 0x01},
    {0x3555, 0x06},
    {0x3556, 0x0C},
    {0x3557, 0x12},
    {0x3558, 0x1C},
    {0x3559, 0x30},
    {0x3549, 0x04},
    {0x354A, 0x35},

    // setB QVGA
    {0x355A, 0x74},
    {0x355B, 0x0A},
    {0x355C, 0x77},
    {0x355D, 0x04},
    {0x355E, 0x14},
    {0x355F, 0x03},
    {0x3560, 0x00},
    {0x3561, 0x01},
    {0x3562, 0x01},
    {0x3563, 0x00},
    {0x3564, 0xFF},
    {0x3565, 0x00},
    {0x3567, 0x01},
    {0x3566, 0x00},
    {0x3569, 0x00},
    {0x356A, 0x01},
    {0x356D, 0x00},
    {0x3576, 0x00},
    {0x356E, 0x00},
    {0x356F, 0x01},
    {0x3570, 0x00},
    {0x3571, 0x02},
    {0x3572, 0x00},
    {0x3573, 0x3F},
    {0x3574, 0x00},
    {0x3575, 0x2F},
    {0x3577, 0x04},
    {0x3578, 0x10},
    {0x3584, 0x01},
    {0x3585, 0x04},
    {0x3586, 0x01},
    {0x3587, 0x38},
    {0x3588, 0x02},
    {0x3589, 0x12},
    {0x358A, 0x04},
    {0x358B, 0x24},
    {0x358C, 0x06},
    {0x358D, 0x36},
    {0x35A5, 0x21},
    {0x35A6, 0x01},
    {0x35A7, 0xE0},
    {0x35A8, 0xF0},
    {0x35A9, 0x10},
    {0x35AA, 0x10},
    {0x35AB, 0x10},
    {0x35AC, 0x20},
    {0x35AD, 0x10},
    {0x35AE, 0x01},
    {0x35AF, 0x06},
    {0x35B0, 0x0C},
    {0x35B1, 0x12},
    {0x35B2, 0x1C},
    {0x35B3, 0x30},
    {0x35A3, 0x02},
    {0x35A4, 0x03},

    // AE_tuning
    {0x3512, 0x3F},
    {0x351F, 0x04},
    {0x3520, 0x03},
    {0x3521, 0x00},
    {0x3523, 0x60},
    {0x3524, 0x08},
    {0x3525, 0x19},
    {0x3526, 0x08},
    {0x3527, 0x10},
    {0x356C, 0x3F},
    {0x3579, 0x04},
    {0x357A, 0x03},
    {0x357B, 0x00},
    {0x357D, 0x60},
    {0x357E, 0x08},
    {0x357F, 0x19},
    {0x3580, 0x08},
    {0x3581, 0x10},
    {0x2048, 0x00},
    {0x2049, 0x10},
    {0x204A, 0x40},
    {0x204E, 0x00},
    {0x204F, 0x38},
    {0x2050, 0xE0},
    {0x204B, 0x00},
    {0x204C, 0x08},
    {0x204D, 0x20},
    {0x2051, 0x00},
    {0x2052, 0x1C},
    {0x2053, 0x70},
    {0x2054, 0x00},
    {0x2055, 0x1A},
    {0x2056, 0xC0},
    {0x2057, 0x00},
    {0x2058, 0x06},
    {0x2059, 0xB0},

    // MD tuning
    {0x2080, 0x41},
    {0x2083, 0x01},
    {0x208D, 0x02},
    {0x208E, 0x08},
    {0x208F, 0x0D},
    {0x2090, 0x14},
    {0x2091, 0x1D},
    {0x2092, 0x30},
    {0x2093, 0x08},
    {0x2094, 0x0A},
    {0x2095, 0x0F},
    {0x2096, 0x14},
    {0x2097, 0x18},
    {0x2098, 0x20},
    {0x2099, 0x10},
    {0x209A, 0x00},
    {0x209B, 0x01},
    {0x209C, 0x01},
    {0x209D, 0x11},
    {0x209E, 0x06},

    // Tone mapping
    {0x1030, 0x04},
    {0x1031, 0x08},
    {0x1032, 0x10},
    {0x1033, 0x18},
    {0x1034, 0x20},
    {0x1035, 0x28},
    {0x1036, 0x30},
    {0x1037, 0x38},
    {0x1038, 0x40},
    {0x1039, 0x50},
    {0x103A, 0x60},
    {0x103B, 0x70},
    {0x103C, 0x80},
    {0x103D, 0xA0},
    {0x103E, 0xC0},
    {0x103F, 0xE0},

    // RESERVED
    {0x35B4, 0x74},
    {0x35B5, 0x0A},
    {0x35B6, 0x77},
    {0x35B7, 0x00},
    {0x35B8, 0x94},
    {0x35B9, 0x03},
    {0x35BA, 0x00},
    {0x35BB, 0x03},
    {0x35BD, 0x00},
    {0x35BE, 0xFF},
    {0x35BF, 0x00},
    {0x35C1, 0x01},
    {0x35C0, 0x01},
    {0x35C3, 0x00},
    {0x35C4, 0x00},
    {0x35C6, 0x3F},
    {0x35C7, 0x00},
    {0x35D0, 0x00},
    {0x35D3, 0x04},
    {0x35D7, 0x18},
    {0x35D8, 0x01},
    {0x35D9, 0x20},
    {0x35DA, 0x08},
    {0x35DB, 0x14},
    {0x35DC, 0x70},
    {0x35C8, 0x00},
    {0x35C9, 0x01},
    {0x35CA, 0x00},
    {0x35CB, 0x02},
    {0x35CC, 0x00},
    {0x35CD, 0x0F},
    {0x35CE, 0x00},
    {0x35CF, 0x0B},
    {0x35DE, 0x00},
    {0x35DF, 0x01},
    {0x35FD, 0x00},
    {0x35FE, 0x5E},

    // RESERVED
    {0x3024, 0x00},
    {0x3025, 0x12},
    {0x3026, 0x03},
    {0x3027, 0x81},
    {0x3028, 0x01},
    {0x3029, 0x00},
    {0x302A, 0x30},

    // Interrupt
    {0x2061, 0x00},
    {0x2062, 0x00},
    {0x2063, 0xC8},

    // Parallel I/F
    {0x1014, 0x00},
    {0x102F, 0x08},
    {0x309E, 0x05},
    {0x309F, 0x02},
    {0x30A0, 0x02},
    {0x30A1, 0x00},
    {0x30A2, 0x08},
    {0x30A3, 0x00},
    {0x30A4, 0x20},
    {0x30A5, 0x04},
    {0x30A6, 0x02},
    {0x30A7, 0x02},
    {0x30A8, 0x01},
    {0x30B0, 0x03},
    {0x3112, 0x04},
    {0x311A, 0x30}, //31：bypass internal ldo 30:internal LDO

    // MIPI
    {0x2800, 0x00},

    // Enable BPC}
    {0x0370, 0x00},
    {0x0371, 0x00},
    {0x0372, 0x01},
    {0x100A, 0x05},
    {0x2590, 0x01},
    {0x0104, 0x01},
    {0x0100, 0x01},

    {REGLIST_TAIL, 0x00}, // tail
};

static const DRAM_ATTR uint16_t sensor_framesize_VGA[][2] = {
    {0x0006, 0x10},
    {0x000D, 0x00},
    {0x000E, 0x00},
    {0x0122, 0xEB},
    {0x0125, 0xFF},
    {0x0126, 0x70},
    {0x05E0, 0XC1},
    {0x05E1, 0x00},
    {0x05E2, 0xC1},
    {0x05E3, 0x00},
    {0x05E4, 0x03},
    {0x05E5, 0x00},
    {0x05E6, 0x82},
    {0x05E7, 0x02},
    {0x05E8, 0x04},
    {0x05E9, 0x00},
    {0x05EA, 0xE3},
    {0x05EB, 0x01},
    {REGLIST_TAIL, 0x00},
};

static const DRAM_ATTR uint16_t sensor_framesize_QVGA[][2] = {
    {0x355A, 0x74}, // vt_sys_d
    {0x355B, 0x0A}, // op_sys_d
    {0x355C, 0x77}, // mclk_div_d
    {0x355D, 0x01},
    {0x355E, 0x1C},
    {0x355F, 0x03},
    {0x3560, 0x00},
    {0x3561, 0x01},
    {0x3562, 0x01},
    {0x3563, 0x00},
    {0x3564, 0xFF},
    {0x3565, 0x00},
    {0x3567, 0x01},
    {0x3566, 0x00},
    {0x3569, 0x00},
    {0x356A, 0x01},
    {0x356C, 0x7F},
    {0x356D, 0x00},
    {0x3576, 0x00},
    {0x3579, 0x03}, // Max Again
    {0x356E, 0x00}, // AE cnt org H HB
    {0x356F, 0x01}, // AE cnt org H LB
    {0x3570, 0x00}, // AE cnt org V HB
    {0x3571, 0x02}, // AE cnt org V LB
    {0x3572, 0x00}, // AE cnt st H HB
    {0x3573, 0x3F}, // AE cnt st H LB
    {0x3574, 0x00}, // AE cnt st V HB
    {0x3575, 0x2F}, // AE cnt st V LB
    {0x3577, 0x04},
    {0x3578, 0x24},
    {0x3584, 0x01},
    {0x3585, 0x04},
    {0x3586, 0x01},
    {0x3587, 0x38},
    {0x3588, 0x02}, // FR stage 1 H
    {0x3589, 0x12}, // FR stage 1 L
    {0x358A, 0x04}, // FR stage 2 H
    {0x358B, 0x24}, // FR stage 2 L
    {0x358C, 0x06}, // FR stage 3 H
    {0x358D, 0x36}, // FR stage 3 L
    {0x35A5, 0x21}, // [7:1]MD_light_coef  [0] MD enable
    {0x35A6, 0x01}, // MD_block_num_th
    {0x35A7, 0xE0}, // [7:4]md_roi_end_v  [3:0]md_roi_start_v
    {0x35A8, 0xF0}, // [7:4]md_roi_end_h  [3:0]md_roi_start_h
    {0x35A9, 0x10}, // [6:0] md_th_str_HCG
    {0x35AA, 0x10}, // [5:0] md_th_str_LCG
    {0x35AB, 0x10}, // [5:0] md_th_str_HDR
    {0x35AC, 0x20}, // md_flick_skip_th_adj_N
    {0x35AD, 0x10}, // md_flick_skip_th_adj_P
    {0x35AE, 0x01},
    {0x35AF, 0x06},
    {0x35B0, 0x0C},
    {0x35B1, 0x12},
    {0x35BC, 0x1C},
    {0x35B3, 0x30},
    {0x35A3, 0x02}, // full trigger H
    {0x35A4, 0x03}, // full trigger L

    {REGLIST_TAIL, 0x00},
};

static const DRAM_ATTR uint16_t sensor_framesize_240X240[][2] = {
    {0x0006, 0x00},
    {0x000D, 0x01},
    {0x000E, 0x11},
    {0x0122, 0xFB},
    {0x0125, 0xFF},
    {0x0126, 0x70},
    {0x05E0, 0XBE},
    {0x05E1, 0x00},
    {0x05E2, 0xBE},
    {0x05E3, 0x00},
    {0x05E4, 0x62},
    {0x05E5, 0x00},
    {0x05E6, 0x51},
    {0x05E7, 0x01},
    {0x05E8, 0x04},
    {0x05E9, 0x00},
    {0x05EA, 0xF3},
    {0x05EB, 0x00},
    {REGLIST_TAIL, 0x00},
};

static const DRAM_ATTR uint16_t sensor_framesize_QQVGA[][2] = {
    {0x355A, 0x74}, // vt_sys_d
    {0x355B, 0x0A}, // op_sys_d
    {0x355C, 0x77}, // mclk_div_d
    {0x355D, 0x00},
    {0x355E, 0x8E},
    {0x355F, 0x03},
    {0x3560, 0x00},
    {0x3561, 0x02},
    {0x3562, 0x02},
    {0x3563, 0x00},
    {0x3564, 0xFF},
    {0x3565, 0x00},
    {0x3567, 0x01},
    {0x3566, 0x00},
    {0x3569, 0x00},
    {0x356A, 0x01},
    {0x356C, 0x7F},
    {0x356D, 0x00},
    {0x3576, 0x00},
    {0x3579, 0x03}, // Max Again
    {0x356E, 0x00}, // AE cnt org H HB
    {0x356F, 0x01}, // AE cnt org H LB
    {0x3570, 0x00}, // AE cnt org V HB
    {0x3571, 0x02}, // AE cnt org V LB
    {0x3572, 0x00}, // AE cnt st H HB
    {0x3573, 0x1F}, // AE cnt st H LB
    {0x3574, 0x00}, // AE cnt st V HB
    {0x3575, 0x17}, // AE cnt st V LB
    {0x3577, 0x04},
    {0x3578, 0x24},
    {0x3584, 0x01},
    {0x3585, 0x04},
    {0x3586, 0x01},
    {0x3587, 0x38},
    {0x3588, 0x02}, // FR stage 1 H
    {0x3589, 0x12}, // FR stage 1 L
    {0x358A, 0x04}, // FR stage 2 H
    {0x358B, 0x24}, // FR stage 2 L
    {0x358C, 0x06}, // FR stage 3 H
    {0x358D, 0x36}, // FR stage 3 L
    {0x35A5, 0x21}, // [7:1]MD_light_coef  [0] MD enable
    {0x35A6, 0x01}, // MD_block_num_th
    {0x35A7, 0xD0}, // [7:4]md_roi_end_v  [3:0]md_roi_start_v
    {0x35A8, 0xF0}, // [7:4]md_roi_end_h  [3:0]md_roi_start_h
    {0x35A9, 0x10}, // [6:0] md_th_str_HCG
    {0x35AA, 0x10}, // [5:0] md_th_str_LCG
    {0x35AB, 0x10}, // [5:0] md_th_str_HDR
    {0x35AC, 0x20}, // md_flick_skip_th_adj_N
    {0x35AD, 0x10}, // md_flick_skip_th_adj_P
    {0x35AE, 0x01},
    {0x35AF, 0x06},
    {0x35B0, 0x0C},
    {0x35B1, 0x12},
    {0x35B2, 0x1C},
    {0x35B3, 0x30},
    {0x35A3, 0x00}, // full trigger H
    {0x35A4, 0xEA}, // full trigger L

    {REGLIST_TAIL, 0x00},
};