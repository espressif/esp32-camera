/*
 * OV3640 register definitions (same map as OV3660 family).
 */
#ifndef __OV3640_REG_REGS_H__
#define __OV3640_REG_REGS_H__

/* system control registers */
#define SYSTEM_CTROL0   0x3008  // Bit[7]: Software reset 
                                // Bit[6]: Software power down 
                                // Bit[5]: Reserved 
                                // Bit[4]: SRB clock SYNC enable 
                                // Bit[3]: Isolation suspend select 
                                // Bit[2:0]: Not used

#define PWCOM1          0x30A9  // Power Common Control 1, bit[3]: bypass regulator

/* JPEG/MCU register bank (bank select via 0x00FF) */
#define OV3640_BANK_SEL         0x00FF
#define OV3640_BANK_DEFAULT     0x00
#define OV3640_BANK_JPEG_MCU    0x01
#define OV3640_MCU_EIS_CTRL     0x00D7  /* disable EIS */
#define OV3640_MCU_SLEEP        0x00D2  /* sleep embedded MCU */
#define OV3640_MCU_IQ_CTRL      0x00D1  /* disable MCU auto IQ */
#define OV3640_MCU_FIFO_CTRL    0x001E  /* enable on-chip FIFO for JPEG */

/* ISP / SDE (Application Notes) */
#define OV3640_SDE_CTRL         0x3302  // bit7: enable SDE
#define OV3640_DPC_CTRL         0x3301  /* defect pixel correction */
#define OV3640_SIMPLE_AWB       0x3308  /* 0xa5 = Simple AWB */
#define OV3640_AWB_CTRL         0x332b  /* bit3: 0=auto, 1=manual */
#define OV3640_SHARPNESS        0x332d
#define OV3640_SHARPNESS_AUTO   0x332f
#define OV3640_AEC_ALGO         0x3047  // 0x00=average, 0x80=histogram
#define OV3640_AEC_18           0x3018
#define OV3640_AEC_19           0x3019
#define OV3640_AEC_1A           0x301a
#define OV3640_AWB_R_GAIN       0x33a7
#define OV3640_AWB_G_GAIN       0x33a8
#define OV3640_AWB_B_GAIN       0x33a9
#define OV3640_SDE_ENABLE       0x3355
#define OV3640_SDE_OFFSET       0x3354
#define OV3640_BRIGHTNESS       0x335e
#define OV3640_CONTRAST_Y1      0x335c
#define OV3640_CONTRAST_Y2      0x335d
#define OV3640_SATURATION_C     0x3358
#define OV3640_SATURATION_M     0x3359
#define OV3640_EFFECT_C1        0x335a
#define OV3640_EFFECT_C2        0x335b

/* output format control registers */
#define FORMAT_CTRL     0x501F // Format select
                                // Bit[2:0]:
                                //  000: YUV422
                                //  001: RGB
                                //  010: Dither
                                //  011: RAW after DPC
                                //  101: RAW after CIP

/* format control registers */
#define FORMAT_CTRL00   0x4300

/* frame control registers */
#define FRAME_CTRL01    0x4201  // Control Passed Frame Number When both ON and OFF number set to 0x00,frame control is in bypass mode
                                // Bit[7:4]: Not used
                                // Bit[3:0]: Frame ON number
#define FRAME_CTRL02    0x4202  // Control Masked Frame Number When both ON and OFF number set to 0x00,frame control is in bypass mode
                                // Bit[7:4]: Not used
                                // BIT[3:0]: Frame OFF number

/* ISP top control registers */
#define PRE_ISP_TEST_SETTING_1  0x503D  // Bit[7]: Test enable
                                        //         0: Test disable
                                        //         1: Color bar enable
                                        // Bit[6]: Rolling
                                        // Bit[5]: Transparent
                                        // Bit[4]: Square black and white
                                        // Bit[3:2]: Color bar style
                                        //         00: Standard 8 color bar
                                        //         01: Gradual change at vertical mode 1
                                        //         10: Gradual change at horizontal
                                        //         11: Gradual change at vertical mode 2
                                        // Bit[1:0]: Test select
                                        //         00: Color bar
                                        //         01: Random data
                                        //         10: Square data
                                        //         11: Black image

//exposure = {0x3500[3:0], 0x3501[7:0], 0x3502[7:0]} / 16 × tROW

/* AEC/AGC control functions */
#define AEC_PK_MANUAL   0x3503  // AEC Manual Mode Control
                                // Bit[7:6]: Reserved
                                // Bit[5]: Gain delay option
                                //         Valid when 0x3503[4]=1’b0
                                //         0: Delay one frame latch
                                //         1: One frame latch
                                // Bit[4:2]: Reserved
                                // Bit[1]: AGC manual
                                //         0: Auto enable
                                //         1: Manual enable
                                // Bit[0]: AEC manual
                                //         0: Auto enable
                                //         1: Manual enable

//gain = {0x350A[1:0], 0x350B[7:0]} / 16

/* mirror and flip (Application Notes section 11.8) */
#define OV3640_TIMING_CTRL      0x307c  /* 0x10=normal 0x11=flip 0x12=mirror 0x13=both */
#define OV3640_ARRAY_CTRL       0x3090
#define OV3640_ARRAY_VSTART     0x3023

/* binning/scale still uses 3820/3821; mirror/flip uses 307c on OV3640 */
#define TIMING_TC_REG20 0x3820  // Timing Control Register
                                // Bit[2:1]: Vertical flip enable
                                //         00: Normal
                                //         11: Vertical flip
                                // Bit[0]: Vertical binning enable
#define TIMING_TC_REG21 0x3821  // Timing Control Register
                                // Bit[5]: Compression Enable
                                // Bit[2:1]: Horizontal mirror enable
                                //         00: Normal
                                //         11: Horizontal mirror
                                // Bit[0]: Horizontal binning enable

#define CLOCK_POL_CONTROL 0x4740// Bit[5]: PCLK polarity 0: active low
                                //          1: active high
                                // Bit[3]: Gate PCLK under VSYNC
                                // Bit[2]: Gate PCLK under HREF
                                // Bit[1]: HREF polarity
                                //          0: active low
                                //          1: active high
                                // Bit[0] VSYNC polarity
                                //          0: active low
                                //          1: active high
#define DRIVE_CAPABILITY 0x302c // Bit[7:6]:
                                //          00: 1x
                                //          01: 2x
                                //          10: 3x
                                //          11: 4x


#define X_ADDR_ST_H     0x3800 //Bit[3:0]: X address start[11:8]
#define X_ADDR_ST_L     0x3801 //Bit[7:0]: X address start[7:0]
#define Y_ADDR_ST_H     0x3802 //Bit[2:0]: Y address start[10:8]
#define Y_ADDR_ST_L     0x3803 //Bit[7:0]: Y address start[7:0]
#define X_ADDR_END_H    0x3804 //Bit[3:0]: X address end[11:8]
#define X_ADDR_END_L    0x3805 //Bit[7:0]:
#define Y_ADDR_END_H    0x3806 //Bit[2:0]: Y address end[10:8]
#define Y_ADDR_END_L    0x3807 //Bit[7:0]:
// Size after scaling
#define X_OUTPUT_SIZE_H 0x3808 //Bit[3:0]: DVP output horizontal width[11:8]
#define X_OUTPUT_SIZE_L 0x3809 //Bit[7:0]:
#define Y_OUTPUT_SIZE_H 0x380a //Bit[2:0]: DVP output vertical height[10:8]
#define Y_OUTPUT_SIZE_L 0x380b //Bit[7:0]:
#define X_TOTAL_SIZE_H  0x380c //Bit[3:0]: Total horizontal size[11:8]
#define X_TOTAL_SIZE_L  0x380d //Bit[7:0]:
#define Y_TOTAL_SIZE_H  0x380e //Bit[7:0]: Total vertical size[15:8]
#define Y_TOTAL_SIZE_L  0x380f //Bit[7:0]:
#define X_OFFSET_H      0x3810 //Bit[3:0]: ISP horizontal offset[11:8]
#define X_OFFSET_L      0x3811 //Bit[7:0]:
#define Y_OFFSET_H      0x3812 //Bit[2:0]: ISP vertical offset[10:8]
#define Y_OFFSET_L      0x3813 //Bit[7:0]:
#define X_INCREMENT     0x3814 //Bit[7:4]: Horizontal odd subsample increment
                               //Bit[3:0]: Horizontal even subsample increment
#define Y_INCREMENT     0x3815 //Bit[7:4]: Vertical odd subsample increment
                               //Bit[3:0]: Vertical even subsample increment
// Size before scaling
//#define X_INPUT_SIZE    (X_ADDR_END - X_ADDR_ST + 1 - (2 * X_OFFSET))
//#define Y_INPUT_SIZE    (Y_ADDR_END - Y_ADDR_ST + 1 - (2 * Y_OFFSET))

#define ISP_CONTROL_01   0x5001 // Bit[5]: Scale enable
                                //          0: Disable
                                //          1: Enable

#define SCALE_CTRL_1     0x5601 // Bit[6:4]: HDIV RW
                                //          DCW scale times
                                //          000: DCW 1 time
                                //          001: DCW 2 times
                                //          010: DCW 4 times
                                //          100: DCW 8 times
                                //          101: DCW 16 times
                                //          Others: DCW 16 times
                                // Bit[2:0]: VDIV RW
                                //          DCW scale times
                                //          000: DCW 1 time
                                //          001: DCW 2 times
                                //          010: DCW 4 times
                                //          100: DCW 8 times
                                //          101: DCW 16 times
                                //          Others: DCW 16 times

#define SCALE_CTRL_2     0x5602 // X_SCALE High Bits
#define SCALE_CTRL_3     0x5603 // X_SCALE Low Bits
#define SCALE_CTRL_4     0x5604 // Y_SCALE High Bits
#define SCALE_CTRL_5     0x5605 // Y_SCALE Low Bits
#define SCALE_CTRL_6     0x5606 // Bit[3:0]: V Offset

#define PCLK_RATIO       0x3824 // Bit[4:0]: PCLK ratio manual
#define VFIFO_CTRL0C     0x460C // Bit[1]: PCLK manual enable
                                //          0: Auto
                                //          1: Manual by PCLK_RATIO

#define VFIFO_X_SIZE_H   0x4602
#define VFIFO_X_SIZE_L   0x4603
#define VFIFO_Y_SIZE_H   0x4604
#define VFIFO_Y_SIZE_L   0x4605

#define SC_PLLS_CTRL0    0x303a // Bit[7]: PLLS bypass
#define SC_PLLS_CTRL1    0x303b // Bit[4:0]: PLLS multiplier
#define SC_PLLS_CTRL2    0x303c // Bit[6:4]: PLLS charge pump control
                                // Bit[3:0]: PLLS system divider
#define SC_PLLS_CTRL3    0x303d // Bit[5:4]: PLLS pre-divider
                                //          00: 1
                                //          01: 1.5
                                //          10: 2
                                //          11: 3
                                // Bit[2]: PLLS root-divider - 1
                                // Bit[1:0]: PLLS seld5
                                //          00: 1
                                //          01: 1
                                //          10: 2
                                //          11: 2.5

#define COMPRESSION_CTRL00 0x4400 //
#define COMPRESSION_CTRL01 0x4401 //
#define COMPRESSION_CTRL02 0x4402 //
#define COMPRESSION_CTRL03 0x4403 //
#define COMPRESSION_CTRL04 0x4404 //
#define COMPRESSION_CTRL05 0x4405 //
#define COMPRESSION_CTRL06 0x4406 //
#define COMPRESSION_CTRL07 0x4407 // Bit[5:0]: QS
#define COMPRESSION_ISI_CTRL 0x4408 //
#define COMPRESSION_CTRL09 0x4409 //
#define COMPRESSION_CTRL0a 0x440a //
#define COMPRESSION_CTRL0b 0x440b //
#define COMPRESSION_CTRL0c 0x440c //
#define COMPRESSION_CTRL0d 0x440d //
#define COMPRESSION_CTRL0E 0x440e //

/**
 * @brief register value
 */
#define TEST_COLOR_BAR  0xC0    /* Enable Color Bar roling Test */

#define AEC_PK_MANUAL_AGC_MANUALEN  0x02    /* Enable AGC Manual enable */
#define AEC_PK_MANUAL_AEC_MANUALEN  0x01    /* Enable AEC Manual enable */

#define TIMING_TC_REG20_VFLIP   0x06 /* Vertical flip enable */
#define TIMING_TC_REG21_HMIRROR 0x06 /* Horizontal mirror enable */

#endif // __OV3640_REG_REGS_H__
