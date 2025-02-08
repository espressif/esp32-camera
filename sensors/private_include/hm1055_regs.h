/*
 * HM1055 register definitions.
 */
#ifndef __REG_REGS_H__
#define __REG_REGS_H__

// Imager Configuration
#define CMU 0x0000
#define IDH 0x0001
#define IDL 0x0002
#define VID 0x0003
#define PWDCTRL 0x0004
#define TGRDCFG 0x0005
#define RDCFG 0x0006
#define VREAD 0x000D
#define HREAD 0x000E
#define IMG_CFG 0x000F

// Imager Operation
#define BLNKRH 0x0010
#define BLNKRL 0x0011
#define BLNKCCLK 0x0012
#define BLNKC 0x0013
#define INTGH 0x0015
#define INTGL 0x0016
#define AGAIN 0x0018
#define DGAIN 0x001D

// IO and Clock Configuration Setting
#define OPRTCFG 0x0020
#define SFTRST 0x0022 // any value to reset
#define IOCTRLH 0x0023
#define IOCTRLL 0x0024
#define CKCFG1 0x0025
#define CKCFG2 0x0026
#define PORTCTRL 0x0027
#define TESTIMG 0x0028
#define CCIR656 0x0029
#define PLL1 0x002A
#define CKCFG3 0x002B
#define PLL2 0x002C

// Black Level Target
#define BLCTGT 0x0040
#define BLCTGT2 0x0053

// Vertical Arbitrary Window Configuration
#define VAWINSTARTH 0x0078
#define VAWINSTARTL 0x0079
#define VAWINENDH 0x007A
#define VAWINENDL 0x007B

// Image Signal Processing Control
#define CMU_AE 0x0100
#define CMU_AWB 0x0101
#define ISPID 0x0105
#define ISPCTRL1 0x0120
#define ISPCTRL2 0x0121
#define ISPCTRL3 0x0122
#define ISPCTRL4 0x0124
#define ISPCTRL5 0x0125
#define ISPCTRL6 0x0126

// RAW Noise Filter Control
#define RAWNF 0x01E4
#define ARAWNF 0x01E5
#define ARAWNFODEL 0x01E6

// Automatic Exposure Control Registers
#define AEWBCFG 0x0380
#define AETARGU 0x0381
#define AETARGL 0x0382
#define AETARGM 0x0383

// Saturation and Hue Control
#define SAT 0x0480
#define ASAT 0x0481
#define ASATODEL 0x0482
#define HUESIN 0x0486
#define HUECOS 0x0487
#define SCENE 0x0488

// Contrast and Brightness Control
#define CONTM 0x04B0
#define ACONTM 0x04B1
#define CONTQ 0x04B3
#define ACONTQ 0x04B4
#define CONTN 0x04B6
#define CONTP 0x04B9
#define CONTGAIN 0x04BC
#define YMEAN 0x04BD
#define BRIGHT 0x04C0

// Y Denoise Control
#define YDN 0x0580
#define AYDN 0x0581
#define AYDNODEL 0x0582

// Sharpness Control
#define EDGE 0x05A1

// Fade to Black Control
#define F2BMEAN 0x05D0
#define F2BRANGE 0x05D1

// Digital Window Control
#define YUVSCXL 0x05E0
#define YUVSCXH 0x05E1
#define YUVSCYL 0x05E2
#define YUVSCYH 0x05E3
#define WINXSTL 0x05E4
#define WINXSTH 0x05E5
#define WINXEDL 0x05E6
#define WINXEDH 0x05E7
#define WINYSTL 0x05E8
#define WINYSTH 0x05E9
#define WINYEDL 0x05EA
#define WINYEDH 0x05EB

#endif //__REG_REGS_H__
