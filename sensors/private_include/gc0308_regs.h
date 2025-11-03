/*
 * GC0308 register definitions.
 * Based on GC0308 datasheet specifications
 */
#ifndef __GC0308_REG_REGS_H__
#define __GC0308_REG_REGS_H__

// System Control Registers
#define RESET_RELATED   0xfe    // Bit[7]: Software reset
                                // Bit[6:5]: NA
                                // Bit[4]: CISCTL_restart_n
                                // Bit[3:1]: NA
                                // Bit[0]: page select
                                //  0:page0
                                //  1:page1

// Page 0 Registers

// Windowing Registers
#define ROW_START_H     0x05    // Row start address high byte
#define ROW_START_L     0x06    // Row start address low byte
#define COL_START_H     0x07    // Column start address high byte
#define COL_START_L     0x08    // Column start address low byte
#define WIN_HEIGHT_H    0x09    // Window height high byte
#define WIN_HEIGHT_L    0x0a    // Window height low byte
#define WIN_WIDTH_H     0x0b    // Window width high byte
#define WIN_WIDTH_L     0x0c    // Window width low byte

// Analog Circuit Control
#define ANALOG_MODE1    0x17    // Analog mode control 1
#define ANALOG_MODE2    0x1a    // Analog mode control 2
#define PCLK_DIV        0x28    // PCLK divider control (frequency division)

// Sensor Control
#define CISCTL_MODE1    0x14    // Bit[1]: V_flip, Bit[0]: H_mirror

// Output Format
#define OUTPUT_FMT      0x24    // Output format control
                                // Bit[3:0]: Output format selection
                                //  0010: YCbYCr (YUV422)
                                //  0110: RGB565
                                //  others: see datasheet

// AWB/AEC/AGC Control
#define AEC_MODE1       0x22    // Auto control modes
                                // Bit[0]: AEC enable (1=on, 0=off)
                                // Bit[1]: AWB enable (1=on, 0=off)
                                // Bit[2]: AGC enable (1=on, 0=off)

// Special Effects and Output Control
#define SPECIAL_EFFECT  0x23    // Special effect control
                                // Bit[1:0]: Effect mode
                                //  00: Normal
                                //  01: Negative
                                //  10: Grayscale/Color tint mode
#define OUT_CTRL        0x2e    // Output control
                                // Bit[0]: Color bar enable

// Color Effect Registers (for tint effects when in grayscale mode)
#define FIXED_CB        0x20    // Fixed Cb value for special effects
#define FIXED_CR        0x21    // Fixed Cr value for special effects
#define EFFECT_MODE     0x2d    // Effect mode additional control

// Global Gain
#define GLOBAL_GAIN     0x50    // Global gain control (hardware: 0-63, 6-bit)
                                // API uses 0-30 range, mapped to hardware values

// Contrast
#define CONTRAST        0xb3    // Contrast control (0-255)

// Manual White Balance Gain (when AWB disabled)
#define AWB_R_GAIN      0x77    // Red channel gain for manual WB
#define AWB_G_GAIN      0x78    // Green channel gain for manual WB
#define AWB_B_GAIN      0x79    // Blue channel gain for manual WB

// Edge Enhancement (Sharpness)
#define EDGE_DEC_SA1    0x8b    // Edge decimation smooth area 1
#define EDGE_DEC_SA2    0x8c    // Edge decimation smooth area 2
#define EDGE_DEC_SA3    0x8d    // Edge decimation smooth area 3
#define EDGE_DEC_SA4    0x8e    // Edge decimation smooth area 4
#define EDGE_DEC_SA5    0x8f    // Edge decimation smooth area 5
#define EDGE_DEC_SA6    0x90    // Edge decimation smooth area 6
#define EDGE_DEC_SA7    0x91    // Edge decimation smooth area 7
#define EDGE_DEC_SA8    0x92    // Edge decimation smooth area 8

// Color Matrix
#define CMX1            0x93    // Color matrix coefficient 1
#define CMX2            0x94    // Color matrix coefficient 2
#define CMX3            0x95    // Color matrix coefficient 3
#define CMX4            0x96    // Color matrix coefficient 4
#define CMX5            0x97    // Color matrix coefficient 5
#define CMX6            0x98    // Color matrix coefficient 6

// Saturation Control
#define SATURATION_CB   0xd0    // Cb saturation control
#define SATURATION_CB1  0xd1    // Cb saturation control 1
#define SATURATION_CR1  0xd2    // Cr saturation control 1

// Lens Shading Correction (Brightness)
#define LSC_RED_B2      0xd3    // LSC target Y base (affects brightness)

// Subsample Window (Page 0 for coordinate reference)
#define SUB_COL_N       0xf7    // Subsample column start (units of 4 pixels)
#define SUB_ROW_N       0xf8    // Subsample row start (units of 4 pixels)
#define SUB_COL_N1      0xf9    // Subsample column end (units of 4 pixels)
#define SUB_ROW_N1      0xfa    // Subsample row end (units of 4 pixels)

// Page 1 Registers (access after setting 0xfe = 0x01)

// AEC Control (Page 1)
#define AEC_TARGET_Y    0x13    // AEC target Y value (brightness target for AE)

// Subsample Control (Page 1)
#define SUBSAMPLE_EN    0x53    // Bit[7]: Subsample enable
#define SUBSAMPLE_MODE  0x54    // Subsample mode control
#define SUBSAMPLE_EN2   0x55    // Bit[0]: Subsample enable 2
#define SUBSAMPLE_Y0    0x56    // Y subsample config 0
#define SUBSAMPLE_Y1    0x57    // Y subsample config 1
#define SUBSAMPLE_UV0   0x58    // UV subsample config 0
#define SUBSAMPLE_UV1   0x59    // UV subsample config 1

/**
 * @brief Register bit masks
 */

// AEC_MODE1 (0x22) bit masks
#define AEC_ENABLE      0x01    // Bit 0: AEC enable
#define AWB_ENABLE      0x02    // Bit 1: AWB enable
#define AGC_ENABLE      0x04    // Bit 2: AGC enable

// CISCTL_MODE1 (0x14) bit masks
#define HMIRROR_MASK    0x01    // Bit 0: Horizontal mirror
#define VFLIP_MASK      0x02    // Bit 1: Vertical flip

// SPECIAL_EFFECT (0x23) bit masks
#define EFFECT_NORMAL   0x00    // Normal mode
#define EFFECT_NEGATIVE 0x01    // Negative/inverse
#define EFFECT_GRAYSCALE 0x02   // B&W or color tint mode

// OUT_CTRL (0x2e) bit masks
#define COLORBAR_ENABLE 0x01    // Bit 0: Color bar test pattern

#endif // __GC0308_REG_REGS_H__
