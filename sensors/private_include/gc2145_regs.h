/*
 * GC2145 register definitions.
 */
#ifndef __GC2145_REG_REGS_H__
#define __GC2145_REG_REGS_H__

#define RESET_RELATED   0xfe    // Bit[7]: Software reset 
                                // Bit[6]: cm reset 
                                // Bit[5]: mipi reset 
                                // Bit[4]: CISCTL_restart_n
                                // Bit[3]: NA
                                // Bit[2:0]: page select
                                //  000:page0
                                //  001:page1
                                //  010:page2
                                //  011:page3

// page0:
#define OUTPUT_FORMAT     0x84 // Format select
                                // Bit[7]:YUV420 row switch
                                // Bit[6]:YUV420 col switch
                                // Bit[7]:YUV420_legacy
                                // Bit[4:0]:output data mode
                                //  5’h00 Cb Y Cr Y
                                //  5’h01 Cr Y Cb Y
                                //  5’h02 Y Cb Y Cr
                                //  5’h03 Y Cr Y Cb
                                //  5’h04 LSC bypass, C/Y
                                //  5’h05 LSC bypass, Y/C
                                //  5’h06 RGB 565
                                //  5’h0f bypass 10bits
                                //  5’h17 switch odd/even column /row to controls output Bayer pattern
                                //    00 RGBG
                                //    01 RGGB
                                //    10 BGGR
                                //    11 GBRG
                                //  5'h18 DNDD out mode
                                //  5'h19 LSC out mode
                                //  5;h1b EEINTP out mode


/**
 * @brief register value
 */


#endif // __GC2145_REG_REGS_H__
