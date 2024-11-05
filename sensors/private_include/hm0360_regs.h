/*
 * HM1055 register definitions.
 */
#ifndef __REG_REGS_H__
#define __REG_REGS_H__

// Sensor ID
#define MODEL_ID_H 0x0000
#define MODEL_ID_L 0x0001
#define SILICON_REV 0x0002
#define FRAME_COUNT_H 0x0005
#define FRAME_COUNT_L 0x0006
#define PIXEL_ODERDER 0x0007

// Snesor mode control
#define MODEL_SELECT 0x0100
#define IMAGE_ORIENTATION 0x0101
#define EMBEDDED_LINE_EN 0x0102
#define SW_RESET 0x0103
#define COMMAND_UPDATE 0x0104

// Sensor exposure gain control
#define INTERGRATION_H 0x0202
#define INTERGRATION_L 0x0203
#define ANALOG_GAIN 0x0205
#define DIGITAL_GAIN_H 0x020E
#define DIGITAL_GAIN_L 0x020F

// clock control
#define PLL1CFG 0x0300
#define PLL2CFG 0x0301
#define PLL3CFG 0x0302

// frame timming control
#define FRAME_LENGTH_LINES_H 0x0340
#define FRAME_LENGTH_LINES_L 0x0341
#define LINE_LENGTH_PCK_H 0x0342
#define LINE_LENGTH_PCK_L 0x0343

// monochrome programming
#define MONO_MODE 0x0370
#define MONO_MODE_ISP 0x0371
#define MONO_MONIDE_SEL 0x0372

// sub-sampling / binning control
#define H_SUB 0x0380
#define V_SUB 0x0381
#define BINNING 0x0382

// test pattern control
#define TEST_PATTERN_MODE 0x0601
#define TEST_DATA_BLUE_H 0x0602
#define TEST_DATA_BLUE_L 0x0603
#define TEST_DATA_GB_H 0x0604
#define TEST_DATA_GB_L 0x0605
#define TEST_DATA_GR_H 0x0605
#define TEST_DATA_GR_L 0x0606
#define TEST_DATA_RED_H 0x0608
#define TEST_DATA_RED_L 0x0609

// black level control
#define BLC_TGT 0x1004
#define BLC2_TGT 0x1009

// monochrome programming
#define MONO_CTRL 0x100A

// VSYNC / HSYNC / pixel shift
#define OPFM_CTRL 0x1014

// automatic exposure programming
#define AE_CTRL 0x2000
#define AE_CTRL2 0x2001
#define CNT_ORG_H_H 0x2002
#define CNT_ORG_H_L 0x2003
#define CNT_ORG_V_H 0x2004
#define CNT_ORG_V_L 0x2005
#define CNT_ST_H_H 0x2006
#define CNT_ST_H_L 0x2007
#define CNT_ST_V_H 0x2008
#define CNT_ST_V_L 0x2009
#define CTRL_PG_SKIPCNT 0x200A
#define BV_WIN_WEIGHT_EN 0x200D


#endif //__REG_REGS_H__
