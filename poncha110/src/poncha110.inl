// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for ams PONCHA110 cameras.
 * Copyright (C) 2022, ams-OSRAM
 *
 * Based on Sony IMX219 camera driver
 * Copyright (C) 2019, Raspberry Pi (Trading) Ltd
 */

#ifndef __PONCHA110_INL__
#define __PONCHA110_INL__

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <asm/unaligned.h>

/*
 * Introduce new v4l2 control/
 */
#include <linux/v4l2-controls.h>
#define AMS_CAMERA_CID_BASE (V4L2_CTRL_CLASS_CAMERA | 0x2000)
#define AMS_CAMERA_CID_MIRA_REG_W (AMS_CAMERA_CID_BASE + 0)
#define AMS_CAMERA_CID_MIRA_REG_R (AMS_CAMERA_CID_BASE + 1)

/* Most significant Byte is flag, and most significant bit is unused. */
#define AMS_CAMERA_CID_PONCHA110_REG_FLAG_FOR_READ 0b00000001
#define AMS_CAMERA_CID_PONCHA110_REG_FLAG_USE_BANK 0b00000010
#define AMS_CAMERA_CID_PONCHA110_REG_FLAG_BANK 0b00000100
#define AMS_CAMERA_CID_PONCHA110_REG_FLAG_CONTEXT 0b00001000
/* Use bit 5 to indicate special command, bit 1,2,3,4 for command. */
#define AMS_CAMERA_CID_PONCHA110_REG_FLAG_CMD_SEL 0b00010000
/* Special command for sleep. The other 3 Bytes (addr+val) is sleep values in us. */
#define AMS_CAMERA_CID_PONCHA110_REG_FLAG_SLEEP_US 0b00010000
/* Special command to enable power on (/off) when stream on (/off). */
#define AMS_CAMERA_CID_PONCHA110_REG_FLAG_RESET_ON 0b00010010
/* Special command to disable power on (/off) when stream on (/off). */
#define AMS_CAMERA_CID_PONCHA110_REG_FLAG_RESET_OFF 0b00010100
/* Special command to enable base register sequence upload, overwrite skip-reg-upload in dtoverlay */
#define AMS_CAMERA_CID_PONCHA110_REG_FLAG_REG_UP_ON 0b00010110
/* Special command to disable base register sequence upload, overwrite skip-reg-upload in dtoverlay */
#define AMS_CAMERA_CID_PONCHA110_REG_FLAG_REG_UP_OFF 0b00011000
/* Special command to manually power on */
#define AMS_CAMERA_CID_PONCHA110_REG_FLAG_POWER_ON 0b00011010
/* Special command to manually power off */
#define AMS_CAMERA_CID_PONCHA110_REG_FLAG_POWER_OFF 0b00011100
/* Special command to turn illumination trigger on */
#define AMS_CAMERA_CID_PONCHA110_REG_FLAG_ILLUM_TRIG_ON 0b00011110
/* Special command to turn illumination trigger off */
#define AMS_CAMERA_CID_PONCHA110_REG_FLAG_ILLUM_TRIG_OFF 0b00010001
/* Special command to set ILLUM_WIDTH. The other 3 Bytes (addr+val) is width value. */
#define AMS_CAMERA_CID_PONCHA110_REG_FLAG_ILLUM_WIDTH 0b00010011
/* Special command to set ILLUM_DELAY. The other 3 Bytes (addr+val) is width value. */
#define AMS_CAMERA_CID_PONCHA110_REG_FLAG_ILLUM_DELAY 0b00010101
/* Special command to enable ILLUM_WIDTH automatically tracking exposure time */
#define AMS_CAMERA_CID_PONCHA110_REG_FLAG_ILLUM_EXP_T_ON 0b00010111
/* Special command to disable ILLUM_WIDTH automatically tracking exposure time */
#define AMS_CAMERA_CID_PONCHA110_REG_FLAG_ILLUM_EXP_T_OFF 0b00011001
/* Special command to enable force_stream_ctrl */
#define AMS_CAMERA_CID_PONCHA110_REG_FLAG_STREAM_CTRL_ON 0b00011011
/* Special command to disable force_stream_ctrl */
#define AMS_CAMERA_CID_PONCHA110_REG_FLAG_STREAM_CTRL_OFF 0b00011101
/*
 * Bit 6&7 of flag are combined to specify I2C dev (default is Mira).
 * If bit 6&7 is 0b01, the reg_addr and reg_val are for a TBD I2C address.
 * The TBD I2C address is default to PONCHA110LED_I2C_ADDR.
 * To change the TBD I2C address, set bit 6&7 to 0b10,
 * then the reg_val will become TBD I2C address.
 * The TBD I2C address is stored in poncha110->tbd_client_i2c_addr.
 */
#define AMS_CAMERA_CID_PONCHA110_REG_FLAG_I2C_SEL 0b01100000
#define AMS_CAMERA_CID_PONCHA110_REG_FLAG_I2C_MIRA 0b00000000
#define AMS_CAMERA_CID_PONCHA110_REG_FLAG_I2C_TBD 0b00100000
#define AMS_CAMERA_CID_PONCHA110_REG_FLAG_I2C_SET_TBD 0b01000000

/* Pre-allocated i2c_client */
#define PONCHA110PMIC_I2C_ADDR 0x2D
#define PONCHA110UC_I2C_ADDR 0x0A
#define PONCHA110LED_I2C_ADDR 0x53

#define PONCHA110_NATIVE_WIDTH 1080
#define PONCHA110_NATIVE_HEIGHT 1082

#define PONCHA110_PIXEL_ARRAY_LEFT 0U
#define PONCHA110_PIXEL_ARRAY_TOP 0U
#define PONCHA110_PIXEL_ARRAY_WIDTH PONCHA110_NATIVE_WIDTH
#define PONCHA110_PIXEL_ARRAY_HEIGHT PONCHA110_NATIVE_HEIGHT

/* Set analog gain min and max to 0 to avoid user changing it. */
/*poncha gains:
0 0 CCC FFFFF
c = coarse
F = fine
000 = 1
001 = 2
010 = 3
011 = 4
110 = 8 (not supprted)
*/
#define PONCHA110_ANALOG_GAIN_REG 0x01fb
#define PONCHA110_ANALOG_GAIN_TRIM 19

#define PONCHA110_ANALOG_GAIN_MAX 2
#define PONCHA110_ANALOG_GAIN_MIN 0
#define PONCHA110_ANALOG_GAIN_STEP 1
#define PONCHA110_ANALOG_GAIN_DEFAULT PONCHA110_ANALOG_GAIN_MIN

#define PONCHA110_CONTEXT_REG 0x0000

// Exposure time is indicated in us
#define PONCHA110_EXP_TIME_L_REG 0x000E
#define PONCHA110_EXP_TIME_S_REG 0x0012

// Target frame time is indicated in us


#define PONCHA110_SUPPORTED_XCLK_FREQ 24000000

// Some timings
#define PONCHA110_DATA_RATE 500 // Mbit/s
#define PONCHA110_ROW_LENGTH_1_3 2910 // 1-4 gain
#define PONCHA110_ROW_LENGTH_1 1650 // 1-4 gain


// pixel_rate = link_freq * 2 * nr_of_lanes / bits_per_sample
// 0.9Gb/s * 2 * 1 / 12 = 157286400
// 1.5 Gbit/s * 2 * 1 / 12 = 250 000 000


/* Trick the libcamera with achievable fps via hblank */

/* Formular in libcamera to derive TARGET_FPS:
 * TARGET_FPS=1/((1/PONCHA110_PIXEL_RATE)*(WIDTH+HBLANK)*(HEIGHT+PONCHA110_MIN_VBLANK))
 * Example with HBLANK=0 and PONCHA110_MIN_VBLANK=12
 * TARGET_FPS=1/((1/157286400)*400*(400+12))=954
 *
 * Inverse the above formula to derive HBLANK from TARGET_FPS:
 * HBLANK=1/((1/PONCHA110_PIXEL_RATE)*TARGET_FPS*(HEIGHT+PONCHA110_MIN_VBLANK))-WIDTH
 * Example with TARGET_FPS of 100 fps
 * HBLANK=1/((1/157286400)*100*(400+12))-400=3418
 */
// #define PONCHA110_HBLANK_100FPS 3418
// #define PONCHA110_HBLANK_360FPS 1290
// #define PONCHA110_HBLANK_200FPS 2650


// For test pattern with fixed data
#define PONCHA110_TRAINING_WORD_REG 0x0060
// For test pattern with 2D gradiant
#define PONCHA110_DELTA_TEST_IMG_REG 0x0056
// For setting test pattern type
#define PONCHA110_TEST_PATTERN_REG 0x0062
#define PONCHA110_TEST_PATTERN_DISABLE 0x00
#define PONCHA110_TEST_PATTERN_FIXED_DATA 0x01
#define PONCHA110_TEST_PATTERN_2D_GRADIENT 0x02

/* Embedded metadata stream structure */
#define PONCHA110_EMBEDDED_LINE_WIDTH 0
#define PONCHA110_NUM_EMBEDDED_LINES 0

/* From Jetson driver */




#define PONCHA110_OTP_COMMAND 0x0066
#define PONCHA110_OTP_ADDR 0x0067
#define PONCHA110_OTP_START 0x0064
#define PONCHA110_OTP_BUSY 0x0065
#define PONCHA110_OTP_DOUT 0x006C
#define PONCHA110_OTP_CAL_VALUE_DEFAULT 2250
#define PONCHA110_OTP_CAL_VALUE_MIN 2000
#define PONCHA110_OTP_CAL_VALUE_MAX 2400




/*relevant params for poncha */
#define PONCHA110_PIXEL_RATE (100000000) //=sequencer clock. row time = row_len /pixel rate
/* Should match device tree link freq */
#define PONCHA110_DEFAULT_LINK_FREQ 456000000

/*relevant registers for Poncha*/
#define PONCHA110_TARGET_FRAME_TIME_REG 0x000A
#define PONCHA110_ROW_LENGTH_REG 0x0010 //multiples of t_seq
#define PONCHA110_EXPOSURE_REG 0x000E

// #define PONCHA110_MIN_VBLANK 353 // for 10b or 8b, 360fps
#define PONCHA110_MIN_VBLANK 20  
#define PONCHA110_MAX_VBLANK 30000	   
#define PONCHA110_DEFAULT_VBLANK_30 148  

#define PONCHA110_EXPOSURE_MIN 1
#define PONCHA110_DEFAULT_EXPOSURE 0x5FF

#define PONCHA110_EXPOSURE_MAX 0xFFFF //long enough..


#define PONCHA110_HBLANK_1 (PONCHA110_ROW_LENGTH_1 - PONCHA110_PIXEL_ARRAY_WIDTH)
#define PONCHA110_HBLANK_1_3 (PONCHA110_ROW_LENGTH_1_3 - PONCHA110_PIXEL_ARRAY_WIDTH)


// Power on function timing
#define PONCHA110_XCLR_MIN_DELAY_US 120000
#define PONCHA110_XCLR_DELAY_RANGE_US 3000


enum pad_types
{
	IMAGE_PAD,
	METADATA_PAD,
	NUM_PADS
};

struct poncha110_reg
{
	u16 address;
	u8 val;
};

struct poncha110_fine_gain_lut
{
	u8 gdig_amp;
	u8 rg_adcgain;
	u8 rg_mult;
};

struct poncha110_reg_list
{
	unsigned int num_of_regs;
	const struct poncha110_reg *regs;
};

struct poncha110_v4l2_reg
{
	u32 val;
};

/* Mode : resolution and related config&values */
struct poncha110_mode
{
	/* Frame width */
	unsigned int width;
	/* Frame height */
	unsigned int height;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* Default register values */
	struct poncha110_reg_list reg_list_pre_soft_reset;

	u32 min_vblank;
	u32 max_vblank;
	u32 hblank;
	u32 row_length;
	/* Format code */
	u32 code;
	u32 gain_min;
	u32 gain_max;
	/* bit_depth needed for analog gain selection */
	u8 bit_depth;
};

// 
//30 fps
static const struct poncha110_reg full_10b_2lane_gain1_3_reg_pre_soft_reset[] = {
	//POLAR_PIX
	{ 0x0106, 0x02 },
	{ 0x0107, 0x83 },
	//POLAR_ANACOL
	{ 0x0104, 0x00 },
	{ 0x0105, 0x09 },
	//GRAN_PIX
	{ 0x0103, 0x01 },
	//GRAN_ANACOL
	{ 0x0102, 0x01 },
	//POS_DPP_TRIGGER
	{ 0x00e7, 0x00 },
	{ 0x00e8, 0x01 },
	//POS_ANACOL_TRIGGER
	{ 0x00e9, 0x00 },
	{ 0x00ea, 0x01 },
	//POS_ANACOL_YBIN_TRIGGER
	{ 0x00eb, 0x00 },
	{ 0x00ec, 0x01 },
	//POS_PIX_TRIGGER
	{ 0x00ed, 0x00 },
	{ 0x00ee, 0x01 },
	//POS_PIX_YBIN_TRIGGER
	{ 0x00ef, 0x00 },
	{ 0x00f0, 0x01 },
	//POS_YADDR_TRIGGER
	{ 0x00f1, 0x00 },
	{ 0x00f2, 0x01 },
	//POS_YADDR_YBIN_TRIGGER
	{ 0x00f3, 0x00 },
	{ 0x00f4, 0x01 },
	//NR_ADC_CLKS_RST
	{ 0x00dd, 0x03 },
	{ 0x00de, 0x10 },
	//NR_ADC_CLKS_SIG
	{ 0x00df, 0x05 },
	{ 0x00e0, 0x10 },
	//DPATH_BITMODE
	{ 0x004d, 0x01 },
	//OFFSET_CLIPPING
	{ 0x004a, 0x06 },
	{ 0x004b, 0x1c },
	//ROW_LENGTH
	{ 0x0010, 0x0b },
	{ 0x0011, 0x5e },
	//ADCANA_BYPASS_CDS
	{ 0x01f1, 0x01 },
	//RAMPGEN_RSTN
	{ 0x01f2, 0x01 },
	//COLLOAD_PULSE_ENABLE
	{ 0x0018, 0x00 },
	//IDAC_SET_1
	{ 0x0200, 0x01 },
	//IDAC_SET_2
	{ 0x0201, 0x00 },
	//IDAC_SET_4
	{ 0x0203, 0x00 },
	//ADCANA_SEL_BW
	{ 0x01f0, 0x03 },
	//IDAC_SET_3
	{ 0x0202, 0x03 },
	//IDAC_SET_5
	{ 0x0204, 0x01 },
	//ANA_PWR_MGR_CTRL
	{ 0x021a, 0x00 },
	{ 0x021b, 0x00 },
	{ 0x021c, 0x01 },
	//ENABLE_IREF
	{ 0x01f8, 0x01 },
	//ENABLE_IDAC
	{ 0x01fd, 0x01 },
	{ 0x01fe, 0xff },
	{ 0x01ff, 0xff },
	//ENABLE_VDD38
	{ 0x01e3, 0x01 },
	//ENABLE_VDD28
	{ 0x01e7, 0x01 },
	//ENABLE_VSS16N
	{ 0x01eb, 0x01 },
	//PCP_DOMAIN_NEW_CMD_TOGGLE
	{ 0x00cc, 0x01 },
	//NCP_DOMAIN_NEW_CMD_TOGGLE
	{ 0x00cf, 0x01 },
	//ENABLE_VSS1N
	{ 0x01e9, 0x01 },
	//ENABLE_VDAC
	{ 0x0213, 0x0f },
	//ENABLE_IDAC_RAMP_IREF
	{ 0x01f9, 0x01 },
	//EN_CLK_F2I
	{ 0x00d9, 0x01 },
	//ENABLE_F2I
	{ 0x01f5, 0x01 },
	//ENABLE_ADCANA_KBC
	{ 0x01ef, 0x00 },
	//ENABLE_RAMPGEN
	{ 0x01f3, 0x01 },
	//ENABLE_COLLOAD_CIS
	{ 0x0218, 0x01 },
	//ENABLE_COLLOAD_LOGIC
	{ 0x0219, 0x01 },
	//CMD
	{ 0x0006, 0x20 },
	//PROG_000
	{ 0x011a, 0x02 },
	{ 0x011b, 0x07 },
	//PROG_001
	{ 0x011c, 0x00 },
	{ 0x011d, 0xf0 },
	//PROG_002
	{ 0x011e, 0x00 },
	{ 0x011f, 0x04 },
	//PROG_003
	{ 0x0120, 0x02 },
	{ 0x0121, 0x04 },
	//PROG_004
	{ 0x0122, 0x01 },
	{ 0x0123, 0x14 },
	//PROG_005
	{ 0x0124, 0x00 },
	{ 0x0125, 0xf5 },
	//PROG_006
	{ 0x0126, 0x00 },
	{ 0x0127, 0xd9 },
	//PROG_007
	{ 0x0128, 0x02 },
	{ 0x0129, 0x04 },
	//PROG_008
	{ 0x012a, 0x02 },
	{ 0x012b, 0x64 },
	//PROG_009
	{ 0x012c, 0x00 },
	{ 0x012d, 0xf7 },
	//PROG_010
	{ 0x012e, 0x00 },
	{ 0x012f, 0xf3 },
	//PROG_011
	{ 0x0130, 0x00 },
	{ 0x0131, 0xf4 },
	//PROG_012
	{ 0x0132, 0x00 },
	{ 0x0133, 0xd0 },
	//PROG_013
	{ 0x0134, 0x00 },
	{ 0x0135, 0xcf },
	//PROG_014
	{ 0x0136, 0x02 },
	{ 0x0137, 0x04 },
	//PROG_015
	{ 0x0138, 0x01 },
	{ 0x0139, 0x42 },
	//PROG_016
	{ 0x013a, 0x00 },
	{ 0x013b, 0xf1 },
	//PROG_017
	{ 0x013c, 0x00 },
	{ 0x013d, 0x01 },
	//PROG_018
	{ 0x013e, 0x02 },
	{ 0x013f, 0x02 },
	//PROG_019
	{ 0x0140, 0x00 },
	{ 0x0141, 0xd9 },
	//PROG_020
	{ 0x0142, 0x02 },
	{ 0x0143, 0x04 },
	//PROG_021
	{ 0x0144, 0x02 },
	{ 0x0145, 0x64 },
	//PROG_022
	{ 0x0146, 0x00 },
	{ 0x0147, 0xf9 },
	//PROG_023
	{ 0x0148, 0x00 },
	{ 0x0149, 0xfa },
	//PROG_024
	{ 0x014a, 0x00 },
	{ 0x014b, 0xf7 },
	//PROG_025
	{ 0x014c, 0x00 },
	{ 0x014d, 0xd0 },
	//PROG_026
	{ 0x014e, 0x02 },
	{ 0x014f, 0x04 },
	//PROG_027
	{ 0x0150, 0x02 },
	{ 0x0151, 0x04 },
	//PROG_028
	{ 0x0152, 0x01 },
	{ 0x0153, 0x17 },
	//PROG_029
	{ 0x0154, 0x00 },
	{ 0x0155, 0x00 },
	//PROG_030
	{ 0x0156, 0x02 },
	{ 0x0157, 0x06 },
	//PROG_031
	{ 0x0158, 0x00 },
	{ 0x0159, 0xf0 },
	//PROG_032
	{ 0x015a, 0x02 },
	{ 0x015b, 0x56 },
	//PROG_033
	{ 0x015c, 0x00 },
	{ 0x015d, 0xcf },
	//PROG_034
	{ 0x015e, 0x00 },
	{ 0x015f, 0xf5 },
	//PROG_035
	{ 0x0160, 0x00 },
	{ 0x0161, 0xcf },
	//PROG_036
	{ 0x0162, 0x00 },
	{ 0x0163, 0xd9 },
	//PROG_037
	{ 0x0164, 0x00 },
	{ 0x0165, 0xf7 },
	//PROG_038
	{ 0x0166, 0x02 },
	{ 0x0167, 0x0b },
	//PROG_039
	{ 0x0168, 0x02 },
	{ 0x0169, 0x09 },
	//PROG_040
	{ 0x016a, 0x00 },
	{ 0x016b, 0xf3 },
	//PROG_041
	{ 0x016c, 0x02 },
	{ 0x016d, 0x09 },
	//PROG_042
	{ 0x016e, 0x02 },
	{ 0x016f, 0x0b },
	//PROG_043
	{ 0x0170, 0x01 },
	{ 0x0171, 0x47 },
	//PROG_044
	{ 0x0172, 0x00 },
	{ 0x0173, 0xf4 },
	//PROG_045
	{ 0x0174, 0x01 },
	{ 0x0175, 0xa7 },
	//PROG_046
	{ 0x0176, 0x02 },
	{ 0x0177, 0x04 },
	//PROG_047
	{ 0x0178, 0x02 },
	{ 0x0179, 0x0a },
	//PROG_048
	{ 0x017a, 0x00 },
	{ 0x017b, 0xd0 },
	//PROG_049
	{ 0x017c, 0x00 },
	{ 0x017d, 0xcf },
	//PROG_050
	{ 0x017e, 0x00 },
	{ 0x017f, 0xf1 },
	//PROG_051
	{ 0x0180, 0x02 },
	{ 0x0181, 0x28 },
	//PROG_052
	{ 0x0182, 0x00 },
	{ 0x0183, 0x01 },
	//PROG_053
	{ 0x0184, 0x00 },
	{ 0x0185, 0xcf },
	//PROG_054
	{ 0x0186, 0x00 },
	{ 0x0187, 0xd9 },
	//PROG_055
	{ 0x0188, 0x00 },
	{ 0x0189, 0xf7 },
	//PROG_056
	{ 0x018a, 0x02 },
	{ 0x018b, 0x0b },
	//PROG_057
	{ 0x018c, 0x02 },
	{ 0x018d, 0x09 },
	//PROG_058
	{ 0x018e, 0x00 },
	{ 0x018f, 0xf9 },
	//PROG_059
	{ 0x0190, 0x02 },
	{ 0x0191, 0x09 },
	//PROG_060
	{ 0x0192, 0x02 },
	{ 0x0193, 0x0b },
	//PROG_061
	{ 0x0194, 0x01 },
	{ 0x0195, 0x47 },
	//PROG_062
	{ 0x0196, 0x00 },
	{ 0x0197, 0xfa },
	//PROG_063
	{ 0x0198, 0x01 },
	{ 0x0199, 0xa7 },
	//PROG_064
	{ 0x019a, 0x02 },
	{ 0x019b, 0x04 },
	//PROG_065
	{ 0x019c, 0x02 },
	{ 0x019d, 0x0a },
	//PROG_066
	{ 0x019e, 0x00 },
	{ 0x019f, 0xd0 },
	//PROG_067
	{ 0x01a0, 0x02 },
	{ 0x01a1, 0x08 },
	//PROG_068
	{ 0x01a2, 0x00 },
	{ 0x01a3, 0x01 },
	//PROG_069
	{ 0x01a4, 0x00 },
	{ 0x01a5, 0x00 },
	//PROG_070
	{ 0x01a6, 0x00 },
	{ 0x01a7, 0x00 },
	//PROG_071
	{ 0x01a8, 0x00 },
	{ 0x01a9, 0x00 },
	//PROG_072
	{ 0x01aa, 0x00 },
	{ 0x01ab, 0x00 },
	//PROG_073
	{ 0x01ac, 0x00 },
	{ 0x01ad, 0x00 },
	//PROG_074
	{ 0x01ae, 0x00 },
	{ 0x01af, 0x00 },
	//PROG_075
	{ 0x01b0, 0x00 },
	{ 0x01b1, 0x00 },
	//PROG_076
	{ 0x01b2, 0x00 },
	{ 0x01b3, 0x00 },
	//PROG_077
	{ 0x01b4, 0x00 },
	{ 0x01b5, 0x00 },
	//PROG_078
	{ 0x01b6, 0x00 },
	{ 0x01b7, 0x00 },
	//PROG_079
	{ 0x01b8, 0x00 },
	{ 0x01b9, 0x00 },
	//PROG_080
	{ 0x01ba, 0x00 },
	{ 0x01bb, 0x00 },
	//PROG_081
	{ 0x01bc, 0x00 },
	{ 0x01bd, 0x00 },
	//PROG_082
	{ 0x01be, 0x00 },
	{ 0x01bf, 0x00 },
	//PROG_083
	{ 0x01c0, 0x00 },
	{ 0x01c1, 0x00 },
	//PROG_084
	{ 0x01c2, 0x00 },
	{ 0x01c3, 0x00 },
	//PROG_085
	{ 0x01c4, 0x00 },
	{ 0x01c5, 0x00 },
	//PROG_086
	{ 0x01c6, 0x00 },
	{ 0x01c7, 0x00 },
	//PROG_087
	{ 0x01c8, 0x00 },
	{ 0x01c9, 0x00 },
	//PROG_088
	{ 0x01ca, 0x00 },
	{ 0x01cb, 0x00 },
	//PROG_089
	{ 0x01cc, 0x00 },
	{ 0x01cd, 0x00 },
	//PROG_090
	{ 0x01ce, 0x00 },
	{ 0x01cf, 0x00 },
	//PROG_091
	{ 0x01d0, 0x00 },
	{ 0x01d1, 0x00 },
	//PROG_092
	{ 0x01d2, 0x00 },
	{ 0x01d3, 0x00 },
	//PROG_093
	{ 0x01d4, 0x00 },
	{ 0x01d5, 0x00 },
	//PROG_094
	{ 0x01d6, 0x00 },
	{ 0x01d7, 0x00 },
	//PROG_095
	{ 0x01d8, 0x00 },
	{ 0x01d9, 0x00 },
	//PROG_096
	{ 0x01da, 0x00 },
	{ 0x01db, 0x00 },
	//PROG_097
	{ 0x01dc, 0x00 },
	{ 0x01dd, 0x00 },
	//PROG_098
	{ 0x01de, 0x00 },
	{ 0x01df, 0x00 },
	//PROG_099
	{ 0x01e0, 0x00 },
	{ 0x01e1, 0x00 },
	//LUT_DEL_000
	{ 0x010a, 0x89 },
	//LUT_DEL_001
	{ 0x010b, 0x60 },
	//LUT_DEL_002
	{ 0x010c, 0x00 },
	//LUT_DEL_003
	{ 0x010d, 0x00 },
	//LUT_DEL_004
	{ 0x010e, 0x4f },
	//LUT_DEL_005
	{ 0x010f, 0x13 },
	//LUT_DEL_006
	{ 0x0110, 0x00 },
	//LUT_DEL_007
	{ 0x0111, 0x0f },
	//LUT_DEL_008
	{ 0x0112, 0x00 },
	//LUT_DEL_009
	{ 0x0113, 0x00 },
	//LUT_DEL_010
	{ 0x0114, 0x82 },
	//LUT_DEL_011
	{ 0x0115, 0x00 },
	//LUT_DEL_012
	{ 0x0116, 0x00 },
	//LUT_DEL_013
	{ 0x0117, 0x00 },
	//LUT_DEL_014
	{ 0x0118, 0x00 },
	//LUT_DEL_015
	{ 0x0119, 0x00 },
	//PTR_PIX
	{ 0x0109, 0x00 },
	//PTR_ANACOL
	{ 0x0108, 0x1e },
	//MIPI_TWAKEUP
	{ 0x008b, 0x00 },
	{ 0x008c, 0x31 },
	{ 0x008d, 0x00 },
	//PLL_CM
	{ 0x0075, 0x28 },
	//PLL_CN
	{ 0x0076, 0x1f },
	//PLL_CO
	{ 0x0077, 0x01 },
	//PLL_CO1
	{ 0x0078, 0x00 },
	//OTP_GRANULARITY
	{ 0x0067, 0x09 },
	//CLKGEN_ESC_DIV
	{ 0x00c7, 0x02 },
	//CLKGEN_NCP_DIV
	{ 0x00cd, 0x01 },
	//CLKGEN_PCP_DIV
	{ 0x00ca, 0x01 },
	//CLK_IN_SOFT_RST_N
	{ 0x00c6, 0x01 },
	//ESC_DOMAIN_NEW_CMD_TOGGLE
	{ 0x00c9, 0x01 },
	//PLL_PD
	{ 0x0074, 0x00 },
	//RW_CONTEXT
	{ 0x0000, 0x00 },
	//PHY_INTERFACE_MODE
	{ 0x00af, 0x00 },
	//MIPI_VC_ID
	{ 0x0084, 0x00 },
	//MIPI_FRAME_COUNT_WRAP
	{ 0x00a7, 0x00 },
	{ 0x00a8, 0x00 },
	//MIPI_CLOCK_LANE_MODE
	{ 0x0086, 0x00 },
	//MIPI_SCRAMBLE_EN
	{ 0x008e, 0x00 },
	//PHY_TX_RCAL
	{ 0x00bf, 0x01 },
	//DPHY_BYPASS_REG_HSTX
	{ 0x00bb, 0x00 },
	//PHY_VCAL_HSTX
	{ 0x00bd, 0x04 },
	//MIPI_ACCURATE_TIMING_MODE
	{ 0x00a9, 0x00 },
	//MIPI_LINE_COUNT_EN
	{ 0x00aa, 0x01 },
	//PHY_PD
	{ 0x00ac, 0x00 },
	//MIPI_INSERT_ERROR1_MODE
	{ 0x009e, 0x00 },
	//MIPI_INSERT_ERROR1_BYTE_NUMBER
	{ 0x009f, 0x00 },
	//MIPI_INSERT_ERROR1_FLIPBITS
	{ 0x00a0, 0x00 },
	//MIPI_INSERT_ERROR2_MODE
	{ 0x00a1, 0x00 },
	//MIPI_INSERT_ERROR2_BYTE_NUMBER
	{ 0x00a2, 0x00 },
	//MIPI_INSERT_ERROR2_FLIPBITS
	{ 0x00a3, 0x00 },
	//MAIN_SOFT_RST_N
	{ 0x00d2, 0x01 },
	//CMD_BYTE_SOFT_RST_N
	{ 0x00d6, 0x01 },
	//EN_CLK_MAIN
	{ 0x00d3, 0x01 },
	//EN_CLK_BYTE
	{ 0x00d7, 0x01 },
	//MAIN_DOMAIN_NEW_CMD_TOGGLE
	{ 0x00d1, 0x01 },
	//BYTE_DOMAIN_NEW_CMD_TOGGLE
	{ 0x00d5, 0x01 },
	//PHY_AUTO_PD_INACTIVE_LANES
	{ 0x00ae, 0x01 },
	//MIPI_CLK_LANE_EN
	{ 0x008f, 0x01 },
	//MIPI_DATA_LANE_EN
	{ 0x0090, 0x03 },
	//MIPI_NBR_LANES
	{ 0x0085, 0x02 },
	//CFA_SIZE
	{ 0x0038, 0x02 },
	//XWIN_OB_ENABLE
	{ 0x003e, 0x00 },
	//OUTPUT_OB_ROWS
	{ 0x0062, 0x00 },
	//IMAGE_ORIENTATION
	{ 0x0034, 0x00 },
	//OB_PEDESTAL
	{ 0x0053, 0x00 },
	{ 0x0054, 0x20 },
	//OB_BYPASS_ENABLE
	{ 0x0055, 0x00 },
	//OB_BYPASS_VALUE
	{ 0x0056, 0x00 },
	{ 0x0057, 0x20 },
	//OB_NROF_HIGH_REMOVE
	{ 0x0058, 0x04 },
	//OB_NROF_LOW_REMOVE
	{ 0x0059, 0x04 },
	//FRAME_TERMINATION_MODE
	{ 0x0009, 0x00 },
	//RW_CONTEXTZ
	{ 0x0000, 0x00 },
	//NROF_FRAMES
	{ 0x0008, 0x14 },
	//ROW_LENGTH
	{ 0x0010, 0x0b },
	{ 0x0011, 0x5e },
	//TARGET_FRAME_TIME
	{ 0x000a, 0x04 },
	{ 0x000b, 0x79 },
	//FRAMESYNC_PREDELAY
	{ 0x000d, 0x32 },
	//FRAME_OVERHEAD_TIME
	{ 0x000c, 0x03 },
	//PIX_CTRL_ELECTRICAL_BLACK
	{ 0x0017, 0x00 },
	//RW_CONTEXT
	{ 0x0000, 0x00 },
	//EXP_TIME
	{ 0x000e, 0x00 },
	{ 0x000f, 0x03 },
	//RW_CONTEXT
	{ 0x0000, 0x00 },
	//XWIN_LEFT
	{ 0x003a, 0x00 },
	{ 0x003b, 0x02 },
	//XWIN_RIGHT
	{ 0x003c, 0x04 },
	{ 0x003d, 0x39 },
	//XWIN_OB_LEFT
	{ 0x003f, 0x04 },
	{ 0x0040, 0x3e },
	//XWIN_OB_RIGHT
	{ 0x0041, 0x04 },
	{ 0x0042, 0x41 },
	//AUTO_RECALC_XPARAMS
	{ 0x0043, 0x01 },
	//RW_CONTEXT
	{ 0x0000, 0x00 },
	//YWIN_ENABLE
	{ 0x0012, 0x03 },
	//YWIN_OB_START
	{ 0x0019, 0x00 },
	{ 0x001a, 0x00 },
	//YWIN_OB_END
	{ 0x001b, 0x00 },
	{ 0x001c, 0x09 },
	//YWIN0_START
	{ 0x0022, 0x00 },
	{ 0x0023, 0x0a },
	//YWIN0_END
	{ 0x0024, 0x04 },
	{ 0x0025, 0x47 },
	//YWIN1_START
	{ 0x002b, 0x00 },
	{ 0x002c, 0x0a },
	//YWIN1_END
	{ 0x002d, 0x04 },
	{ 0x002e, 0x47 },
	//EXTERN_YADDR
	{ 0x0014, 0x00 },
	{ 0x0015, 0x00 },
	//FORCE_EXTERN_YADDR
	{ 0x0013, 0x00 },
	//RW_CONTEXT
	{ 0x0000, 0x00 },
	//YWIN_OB_SUBS_FACTOR
	{ 0x001d, 0x01 },
	//YWIN0_SUBS_FACTOR
	{ 0x0026, 0x01 },
	//YWIN1_SUBS_FACTOR
	{ 0x002f, 0x01 },
	//XSUBS_FACTOR
	{ 0x0039, 0x01 },
	//BINNING_MODE
	{ 0x0035, 0x00 },
	//BINNING_TYPE
	{ 0x0036, 0x11 },
	//BINNING_WEIGHTING
	{ 0x0037, 0x00 },
	//RW_CONTEXT
	{ 0x0000, 0x00 },
	//YWIN_CROP_ENABLE
	{ 0x0016, 0x01 },
	//YWIN_OB_CROP_OFFSET
	{ 0x001e, 0x00 },
	{ 0x001f, 0x02 },
	//YWIN_OB_CROP_HEIGHT
	{ 0x0020, 0x00 },
	{ 0x0021, 0x06 },
	//YWIN0_CROP_OFFSET
	{ 0x0027, 0x00 },
	{ 0x0028, 0x02 },
	//YWIN0_CROP_HEIGHT
	{ 0x0029, 0x04 },
	{ 0x002a, 0x3a },
	//YWIN1_CROP_OFFSET
	{ 0x0030, 0x00 },
	{ 0x0031, 0x02 },
	//YWIN1_CROP_HEIGHT
	{ 0x0032, 0x04 },
	{ 0x0033, 0x3a },
	//RW_CONTEXT
	{ 0x0000, 0x00 },
	//GDIG_CFA_IDX0
	{ 0x004e, 0x0f },
	//GDIG_CFA_IDX1
	{ 0x004f, 0x0f },
	//GDIG_CFA_IDX2
	{ 0x0050, 0x0f },
	//GDIG_CFA_IDX3
	{ 0x0051, 0x0f },
	//NEXT_ACTIVE_CONTEXT
	{ 0x0002, 0x00 },
	//NROF_FRAMES
	{ 0x0008, 0x00 },
	//META_INSERT_EN
	{ 0x0063, 0x00 },
	//MIPI_EMBEDDED_DATA_TYPE
	{ 0x00ab, 0x12 },
	//LVDS_EMBEDDED_DATA_TYPE
	{ 0x0064, 0x02 },
	//DPC_MODE
	{ 0x005a, 0x00 },
	//DPC_LIMIT_HIGH_SLOPE
	{ 0x005b, 0x02 },
	//DPC_LIMIT_HIGH_OFFSET
	{ 0x005c, 0x02 },
	//DPC_LIMIT_LOW_SLOPE
	{ 0x005d, 0x02 },
	//DPC_LIMIT_LOW_OFFSET
	{ 0x005e, 0x02 },
	//TEST_LVDS
	{ 0x007e, 0x00 },
	//TRAINING_WORD
	{ 0x007d, 0x0a },
	//MIPI_CLK_PN_CHANGE
	{ 0x00b3, 0x00 },
	//MIPI_DATA_PN_CHANGE
	{ 0x00b4, 0x00 },
	//MIPI_LANE0_SEL
	{ 0x0091, 0x00 },
	//MIPI_LANE1_SEL
	{ 0x0092, 0x01 },
	//DPHY_LANE0_SEL
	{ 0x00b5, 0x00 },
	//TSENS_ENABLE
	{ 0x006b, 0x01 },
	//TSENS_NOF_SAMPLES
	{ 0x0072, 0x01 },
	//TSENS_EWMA_ENABLE
	{ 0x006f, 0x01 },
	//TSENS_EWMA_FRAC_SEL
	{ 0x0070, 0x00 },
	//TSENS_RESET_EARLY
	{ 0x0073, 0x00 },
	//TSENS_CLKDIV
	{ 0x0071, 0x01 },
	//AMUX_BYPASS
	{ 0x021e, 0x01 },
	//ENABLE_AMUX
	{ 0x021d, 0x00 },
	//AMUX_SEL
	{ 0x021f, 0x3c },
	//TDIG_MODE
	{ 0x0222, 0x00 },
	//TDIG_FACTOR
	{ 0x0221, 0x00 },
	//DMUX_SEL
	{ 0x0220, 0x00 },
	//LFSR_EN
	{ 0x0052, 0x07 },
	//EN_AUTO_KERNEL_ENABLE_CALC
	{ 0x0046, 0x01 },
	//KERNEL_ENABLE_FORCE_VALUE
	{ 0x0047, 0x01 },
	//NR_ADC_CONVERSIONS
	{ 0x00f5, 0x0a },
	//KERNEL_ADDR_GLOBAL_CDS
	{ 0x00f6, 0x09 },
	//KERNEL_ADDR_COL0
	{ 0x00f7, 0x00 },
	//KERNEL_ADDR_COL1
	{ 0x00f8, 0x01 },
	//KERNEL_ADDR_COL2
	{ 0x00f9, 0x02 },
	//KERNEL_ADDR_COL3
	{ 0x00fa, 0x03 },
	//KERNEL_ADDR_COL4
	{ 0x00fb, 0x04 },
	//KERNEL_ADDR_COL5
	{ 0x00fc, 0x05 },
	//KERNEL_ADDR_COL6
	{ 0x00fd, 0x06 },
	//KERNEL_ADDR_COL7
	{ 0x00fe, 0x07 },
	//KERNEL_ADDR_COL8
	{ 0x00ff, 0x08 },
	//KERNEL_ADDR_COL9
	{ 0x0100, 0x09 },
	//KERNEL_ADDR_INVALID
	{ 0x0101, 0x0e },
	//NROF_FRAMES
	{ 0x0008, 0x00 },
	//MODE_SELECT
	{ 0x0007, 0x00 },
	//MODE_SELECT
	{ 0x0007, 0x01 },
	//RW_CONTEXT
	{ 0x0000, 0x00 },
	//RAMPGEN_SET
	{ 0x01fa, 0x00 },
	{ 0x01fb, 0x13 },
	//RW_CONTEXT
	{ 0x0000, 0x01 },
	//RAMPGEN_SET
	{ 0x01fa, 0x00 },
	{ 0x01fb, 0x13 },
	//RW_CONTEXT
	{ 0x0000, 0x02 },
	//RAMPGEN_SET
	{ 0x01fa, 0x00 },
	{ 0x01fb, 0x13 },
	//RW_CONTEXT
	{ 0x0000, 0x00 },
	//NEXT_ACTIVE_CONTEXT
	{ 0x0002, 0x00 },
	//OTP_ENABLE
	{ 0x0065, 0x01 },
	//CMD
	{ 0x0006, 0x28 },
	//CMD
	{ 0x0006, 0x20 },
	//OTP_BUSY
	//OTP_ENABLE
	{ 0x0065, 0x00 },
	//TRIM_SEL
	{ 0x01e2, 0x00 },
	//KERNEL_ADDR_GLOBAL_CDS
	{ 0x00f6, 0x09 },
};

//60 fps crop
static const struct poncha110_reg crop_980_10b_2lane_gain1_reg_pre_soft_reset[] = {
	//POLAR_PIX
	{ 0x0106, 0x02 },
	{ 0x0107, 0x83 },
	//POLAR_ANACOL
	{ 0x0104, 0x00 },
	{ 0x0105, 0x09 },
	//GRAN_PIX
	{ 0x0103, 0x01 },
	//GRAN_ANACOL
	{ 0x0102, 0x01 },
	//POS_DPP_TRIGGER
	{ 0x00e7, 0x00 },
	{ 0x00e8, 0x01 },
	//POS_ANACOL_TRIGGER
	{ 0x00e9, 0x00 },
	{ 0x00ea, 0x01 },
	//POS_ANACOL_YBIN_TRIGGER
	{ 0x00eb, 0x00 },
	{ 0x00ec, 0x01 },
	//POS_PIX_TRIGGER
	{ 0x00ed, 0x00 },
	{ 0x00ee, 0x01 },
	//POS_PIX_YBIN_TRIGGER
	{ 0x00ef, 0x00 },
	{ 0x00f0, 0x01 },
	//POS_YADDR_TRIGGER
	{ 0x00f1, 0x00 },
	{ 0x00f2, 0x01 },
	//POS_YADDR_YBIN_TRIGGER
	{ 0x00f3, 0x00 },
	{ 0x00f4, 0x01 },
	//NR_ADC_CLKS_RST
	{ 0x00dd, 0x00 },
	{ 0x00de, 0xe4 },
	//NR_ADC_CLKS_SIG
	{ 0x00df, 0x02 },
	{ 0x00e0, 0xe4 },
	//DPATH_BITMODE
	{ 0x004d, 0x01 },
	//OFFSET_CLIPPING
	{ 0x004a, 0x01 },
	{ 0x004b, 0xc8 },
	//ROW_LENGTH
	{ 0x0010, 0x06 },
	{ 0x0011, 0x72 },
	//ADCANA_BYPASS_CDS
	{ 0x01f1, 0x01 },
	//RAMPGEN_RSTN
	{ 0x01f2, 0x01 },
	//COLLOAD_PULSE_ENABLE
	{ 0x0018, 0x00 },
	//IDAC_SET_1
	{ 0x0200, 0x01 },
	//IDAC_SET_2
	{ 0x0201, 0x00 },
	//IDAC_SET_4
	{ 0x0203, 0x00 },
	//ADCANA_SEL_BW
	{ 0x01f0, 0x03 },
	//IDAC_SET_3
	{ 0x0202, 0x03 },
	//IDAC_SET_5
	{ 0x0204, 0x01 },
	//ANA_PWR_MGR_CTRL
	{ 0x021a, 0x00 },
	{ 0x021b, 0x00 },
	{ 0x021c, 0x01 },
	//ENABLE_IREF
	{ 0x01f8, 0x01 },
	//ENABLE_IDAC
	{ 0x01fd, 0x01 },
	{ 0x01fe, 0xff },
	{ 0x01ff, 0xff },
	//ENABLE_VDD38
	{ 0x01e3, 0x01 },
	//ENABLE_VDD28
	{ 0x01e7, 0x01 },
	//ENABLE_VSS16N
	{ 0x01eb, 0x01 },
	//PCP_DOMAIN_NEW_CMD_TOGGLE
	{ 0x00cc, 0x01 },
	//NCP_DOMAIN_NEW_CMD_TOGGLE
	{ 0x00cf, 0x01 },
	//ENABLE_VSS1N
	{ 0x01e9, 0x01 },
	//ENABLE_VDAC
	{ 0x0213, 0x0f },
	//ENABLE_IDAC_RAMP_IREF
	{ 0x01f9, 0x01 },
	//EN_CLK_F2I
	{ 0x00d9, 0x01 },
	//ENABLE_F2I
	{ 0x01f5, 0x01 },
	//ENABLE_ADCANA_KBC
	{ 0x01ef, 0x01 },
	//ENABLE_RAMPGEN
	{ 0x01f3, 0x01 },
	//ENABLE_COLLOAD_CIS
	{ 0x0218, 0x01 },
	//ENABLE_COLLOAD_LOGIC
	{ 0x0219, 0x01 },
	//CMD
	{ 0x0006, 0x20 },
	//PROG_000
	{ 0x011a, 0x02 },
	{ 0x011b, 0x07 },
	//PROG_001
	{ 0x011c, 0x00 },
	{ 0x011d, 0xf0 },
	//PROG_002
	{ 0x011e, 0x00 },
	{ 0x011f, 0x04 },
	//PROG_003
	{ 0x0120, 0x02 },
	{ 0x0121, 0x04 },
	//PROG_004
	{ 0x0122, 0x01 },
	{ 0x0123, 0x14 },
	//PROG_005
	{ 0x0124, 0x00 },
	{ 0x0125, 0xf5 },
	//PROG_006
	{ 0x0126, 0x00 },
	{ 0x0127, 0xd9 },
	//PROG_007
	{ 0x0128, 0x02 },
	{ 0x0129, 0x04 },
	//PROG_008
	{ 0x012a, 0x02 },
	{ 0x012b, 0x64 },
	//PROG_009
	{ 0x012c, 0x00 },
	{ 0x012d, 0xf7 },
	//PROG_010
	{ 0x012e, 0x00 },
	{ 0x012f, 0xf3 },
	//PROG_011
	{ 0x0130, 0x00 },
	{ 0x0131, 0xf4 },
	//PROG_012
	{ 0x0132, 0x00 },
	{ 0x0133, 0xd0 },
	//PROG_013
	{ 0x0134, 0x00 },
	{ 0x0135, 0xcf },
	//PROG_014
	{ 0x0136, 0x02 },
	{ 0x0137, 0x04 },
	//PROG_015
	{ 0x0138, 0x01 },
	{ 0x0139, 0x42 },
	//PROG_016
	{ 0x013a, 0x00 },
	{ 0x013b, 0xf1 },
	//PROG_017
	{ 0x013c, 0x00 },
	{ 0x013d, 0x01 },
	//PROG_018
	{ 0x013e, 0x02 },
	{ 0x013f, 0x02 },
	//PROG_019
	{ 0x0140, 0x00 },
	{ 0x0141, 0xd9 },
	//PROG_020
	{ 0x0142, 0x02 },
	{ 0x0143, 0x04 },
	//PROG_021
	{ 0x0144, 0x02 },
	{ 0x0145, 0x64 },
	//PROG_022
	{ 0x0146, 0x00 },
	{ 0x0147, 0xf9 },
	//PROG_023
	{ 0x0148, 0x00 },
	{ 0x0149, 0xfa },
	//PROG_024
	{ 0x014a, 0x00 },
	{ 0x014b, 0xf8 },
	//PROG_025
	{ 0x014c, 0x00 },
	{ 0x014d, 0xd0 },
	//PROG_026
	{ 0x014e, 0x02 },
	{ 0x014f, 0x04 },
	//PROG_027
	{ 0x0150, 0x02 },
	{ 0x0151, 0x04 },
	//PROG_028
	{ 0x0152, 0x01 },
	{ 0x0153, 0x17 },
	//PROG_029
	{ 0x0154, 0x00 },
	{ 0x0155, 0x00 },
	//PROG_030
	{ 0x0156, 0x01 },
	{ 0x0157, 0x64 },
	//PROG_031
	{ 0x0158, 0x00 },
	{ 0x0159, 0xf0 },
	//PROG_032
	{ 0x015a, 0x02 },
	{ 0x015b, 0x56 },
	//PROG_033
	{ 0x015c, 0x00 },
	{ 0x015d, 0xcf },
	//PROG_034
	{ 0x015e, 0x00 },
	{ 0x015f, 0xf5 },
	//PROG_035
	{ 0x0160, 0x02 },
	{ 0x0161, 0x04 },
	//PROG_036
	{ 0x0162, 0x00 },
	{ 0x0163, 0xd9 },
	//PROG_037
	{ 0x0164, 0x00 },
	{ 0x0165, 0xf7 },
	//PROG_038
	{ 0x0166, 0x02 },
	{ 0x0167, 0x0b },
	//PROG_039
	{ 0x0168, 0x02 },
	{ 0x0169, 0x09 },
	//PROG_040
	{ 0x016a, 0x00 },
	{ 0x016b, 0xf3 },
	//PROG_041
	{ 0x016c, 0x02 },
	{ 0x016d, 0x09 },
	//PROG_042
	{ 0x016e, 0x02 },
	{ 0x016f, 0x0b },
	//PROG_043
	{ 0x0170, 0x01 },
	{ 0x0171, 0x47 },
	//PROG_044
	{ 0x0172, 0x00 },
	{ 0x0173, 0xf4 },
	//PROG_045
	{ 0x0174, 0x01 },
	{ 0x0175, 0xa7 },
	//PROG_046
	{ 0x0176, 0x02 },
	{ 0x0177, 0x04 },
	//PROG_047
	{ 0x0178, 0x02 },
	{ 0x0179, 0x0a },
	//PROG_048
	{ 0x017a, 0x00 },
	{ 0x017b, 0xd0 },
	//PROG_049
	{ 0x017c, 0x02 },
	{ 0x017d, 0x04 },
	//PROG_050
	{ 0x017e, 0x00 },
	{ 0x017f, 0xf1 },
	//PROG_051
	{ 0x0180, 0x02 },
	{ 0x0181, 0x28 },
	//PROG_052
	{ 0x0182, 0x00 },
	{ 0x0183, 0x01 },
	//PROG_053
	{ 0x0184, 0x02 },
	{ 0x0185, 0x04 },
	//PROG_054
	{ 0x0186, 0x00 },
	{ 0x0187, 0xd9 },
	//PROG_055
	{ 0x0188, 0x00 },
	{ 0x0189, 0xf8 },
	//PROG_056
	{ 0x018a, 0x02 },
	{ 0x018b, 0x0b },
	//PROG_057
	{ 0x018c, 0x02 },
	{ 0x018d, 0x09 },
	//PROG_058
	{ 0x018e, 0x00 },
	{ 0x018f, 0xf9 },
	//PROG_059
	{ 0x0190, 0x02 },
	{ 0x0191, 0x09 },
	//PROG_060
	{ 0x0192, 0x02 },
	{ 0x0193, 0x0b },
	//PROG_061
	{ 0x0194, 0x01 },
	{ 0x0195, 0x47 },
	//PROG_062
	{ 0x0196, 0x00 },
	{ 0x0197, 0xfa },
	//PROG_063
	{ 0x0198, 0x01 },
	{ 0x0199, 0xa7 },
	//PROG_064
	{ 0x019a, 0x02 },
	{ 0x019b, 0x04 },
	//PROG_065
	{ 0x019c, 0x02 },
	{ 0x019d, 0x0a },
	//PROG_066
	{ 0x019e, 0x00 },
	{ 0x019f, 0xd0 },
	//PROG_067
	{ 0x01a0, 0x02 },
	{ 0x01a1, 0x08 },
	//PROG_068
	{ 0x01a2, 0x00 },
	{ 0x01a3, 0x01 },
	//PROG_069
	{ 0x01a4, 0x00 },
	{ 0x01a5, 0x00 },
	//PROG_070
	{ 0x01a6, 0x00 },
	{ 0x01a7, 0x00 },
	//PROG_071
	{ 0x01a8, 0x00 },
	{ 0x01a9, 0x00 },
	//PROG_072
	{ 0x01aa, 0x00 },
	{ 0x01ab, 0x00 },
	//PROG_073
	{ 0x01ac, 0x00 },
	{ 0x01ad, 0x00 },
	//PROG_074
	{ 0x01ae, 0x00 },
	{ 0x01af, 0x00 },
	//PROG_075
	{ 0x01b0, 0x00 },
	{ 0x01b1, 0x00 },
	//PROG_076
	{ 0x01b2, 0x00 },
	{ 0x01b3, 0x00 },
	//PROG_077
	{ 0x01b4, 0x00 },
	{ 0x01b5, 0x00 },
	//PROG_078
	{ 0x01b6, 0x00 },
	{ 0x01b7, 0x00 },
	//PROG_079
	{ 0x01b8, 0x00 },
	{ 0x01b9, 0x00 },
	//PROG_080
	{ 0x01ba, 0x00 },
	{ 0x01bb, 0x00 },
	//PROG_081
	{ 0x01bc, 0x00 },
	{ 0x01bd, 0x00 },
	//PROG_082
	{ 0x01be, 0x00 },
	{ 0x01bf, 0x00 },
	//PROG_083
	{ 0x01c0, 0x00 },
	{ 0x01c1, 0x00 },
	//PROG_084
	{ 0x01c2, 0x00 },
	{ 0x01c3, 0x00 },
	//PROG_085
	{ 0x01c4, 0x00 },
	{ 0x01c5, 0x00 },
	//PROG_086
	{ 0x01c6, 0x00 },
	{ 0x01c7, 0x00 },
	//PROG_087
	{ 0x01c8, 0x00 },
	{ 0x01c9, 0x00 },
	//PROG_088
	{ 0x01ca, 0x00 },
	{ 0x01cb, 0x00 },
	//PROG_089
	{ 0x01cc, 0x00 },
	{ 0x01cd, 0x00 },
	//PROG_090
	{ 0x01ce, 0x00 },
	{ 0x01cf, 0x00 },
	//PROG_091
	{ 0x01d0, 0x00 },
	{ 0x01d1, 0x00 },
	//PROG_092
	{ 0x01d2, 0x00 },
	{ 0x01d3, 0x00 },
	//PROG_093
	{ 0x01d4, 0x00 },
	{ 0x01d5, 0x00 },
	//PROG_094
	{ 0x01d6, 0x00 },
	{ 0x01d7, 0x00 },
	//PROG_095
	{ 0x01d8, 0x00 },
	{ 0x01d9, 0x00 },
	//PROG_096
	{ 0x01da, 0x00 },
	{ 0x01db, 0x00 },
	//PROG_097
	{ 0x01dc, 0x00 },
	{ 0x01dd, 0x00 },
	//PROG_098
	{ 0x01de, 0x00 },
	{ 0x01df, 0x00 },
	//PROG_099
	{ 0x01e0, 0x00 },
	{ 0x01e1, 0x00 },
	//LUT_DEL_000
	{ 0x010a, 0x89 },
	//LUT_DEL_001
	{ 0x010b, 0x60 },
	//LUT_DEL_002
	{ 0x010c, 0x00 },
	//LUT_DEL_003
	{ 0x010d, 0x00 },
	//LUT_DEL_004
	{ 0x010e, 0x17 },
	//LUT_DEL_005
	{ 0x010f, 0x13 },
	//LUT_DEL_006
	{ 0x0110, 0x00 },
	//LUT_DEL_007
	{ 0x0111, 0x07 },
	//LUT_DEL_008
	{ 0x0112, 0x09 },
	//LUT_DEL_009
	{ 0x0113, 0x00 },
	//LUT_DEL_010
	{ 0x0114, 0x4a },
	//LUT_DEL_011
	{ 0x0115, 0x00 },
	//LUT_DEL_012
	{ 0x0116, 0x00 },
	//LUT_DEL_013
	{ 0x0117, 0x00 },
	//LUT_DEL_014
	{ 0x0118, 0x00 },
	//LUT_DEL_015
	{ 0x0119, 0x00 },
	//PTR_PIX
	{ 0x0109, 0x00 },
	//PTR_ANACOL
	{ 0x0108, 0x1e },
	//MIPI_TWAKEUP
	{ 0x008b, 0x00 },
	{ 0x008c, 0x31 },
	{ 0x008d, 0x00 },
	//PLL_CM
	{ 0x0075, 0x28 },
	//PLL_CN
	{ 0x0076, 0x1f },
	//PLL_CO
	{ 0x0077, 0x01 },
	//PLL_CO1
	{ 0x0078, 0x00 },
	//OTP_GRANULARITY
	{ 0x0067, 0x09 },
	//CLKGEN_ESC_DIV
	{ 0x00c7, 0x02 },
	//CLKGEN_NCP_DIV
	{ 0x00cd, 0x01 },
	//CLKGEN_PCP_DIV
	{ 0x00ca, 0x01 },
	//CLK_IN_SOFT_RST_N
	{ 0x00c6, 0x01 },
	//ESC_DOMAIN_NEW_CMD_TOGGLE
	{ 0x00c9, 0x01 },
	//PLL_PD
	{ 0x0074, 0x00 },
	//RW_CONTEXT
	{ 0x0000, 0x00 },
	//PHY_INTERFACE_MODE
	{ 0x00af, 0x00 },
	//MIPI_VC_ID
	{ 0x0084, 0x00 },
	//MIPI_FRAME_COUNT_WRAP
	{ 0x00a7, 0x00 },
	{ 0x00a8, 0x00 },
	//MIPI_CLOCK_LANE_MODE
	{ 0x0086, 0x00 },
	//MIPI_SCRAMBLE_EN
	{ 0x008e, 0x00 },
	//PHY_TX_RCAL
	{ 0x00bf, 0x01 },
	//DPHY_BYPASS_REG_HSTX
	{ 0x00bb, 0x00 },
	//PHY_VCAL_HSTX
	{ 0x00bd, 0x04 },
	//MIPI_ACCURATE_TIMING_MODE
	{ 0x00a9, 0x00 },
	//MIPI_LINE_COUNT_EN
	{ 0x00aa, 0x01 },
	//PHY_PD
	{ 0x00ac, 0x00 },
	//MIPI_INSERT_ERROR1_MODE
	{ 0x009e, 0x00 },
	//MIPI_INSERT_ERROR1_BYTE_NUMBER
	{ 0x009f, 0x00 },
	//MIPI_INSERT_ERROR1_FLIPBITS
	{ 0x00a0, 0x00 },
	//MIPI_INSERT_ERROR2_MODE
	{ 0x00a1, 0x00 },
	//MIPI_INSERT_ERROR2_BYTE_NUMBER
	{ 0x00a2, 0x00 },
	//MIPI_INSERT_ERROR2_FLIPBITS
	{ 0x00a3, 0x00 },
	//MAIN_SOFT_RST_N
	{ 0x00d2, 0x01 },
	//CMD_BYTE_SOFT_RST_N
	{ 0x00d6, 0x01 },
	//EN_CLK_MAIN
	{ 0x00d3, 0x01 },
	//EN_CLK_BYTE
	{ 0x00d7, 0x01 },
	//MAIN_DOMAIN_NEW_CMD_TOGGLE
	{ 0x00d1, 0x01 },
	//BYTE_DOMAIN_NEW_CMD_TOGGLE
	{ 0x00d5, 0x01 },
	//PHY_AUTO_PD_INACTIVE_LANES
	{ 0x00ae, 0x01 },
	//MIPI_CLK_LANE_EN
	{ 0x008f, 0x01 },
	//MIPI_DATA_LANE_EN
	{ 0x0090, 0x03 },
	//MIPI_NBR_LANES
	{ 0x0085, 0x02 },
	//CFA_SIZE
	{ 0x0038, 0x02 },
	//XWIN_OB_ENABLE
	{ 0x003e, 0x00 },
	//OUTPUT_OB_ROWS
	{ 0x0062, 0x00 },
	//IMAGE_ORIENTATION
	{ 0x0034, 0x00 },
	//OB_PEDESTAL
	{ 0x0053, 0x00 },
	{ 0x0054, 0x20 },
	//OB_BYPASS_ENABLE
	{ 0x0055, 0x00 },
	//OB_BYPASS_VALUE
	{ 0x0056, 0x00 },
	{ 0x0057, 0x20 },
	//OB_NROF_HIGH_REMOVE
	{ 0x0058, 0x04 },
	//OB_NROF_LOW_REMOVE
	{ 0x0059, 0x04 },
	//FRAME_TERMINATION_MODE
	{ 0x0009, 0x00 },
	//RW_CONTEXT
	{ 0x0000, 0x00 },
	//NROF_FRAMES
	{ 0x0008, 0x14 },
	//ROW_LENGTH
	{ 0x0010, 0x06 },
	{ 0x0011, 0x72 },
	//TARGET_FRAME_TIME
	{ 0x000a, 0x07 },
	{ 0x000b, 0xe4 },
	//FRAMESYNC_PREDELAY
	{ 0x000d, 0x32 },
	//FRAME_OVERHEAD_TIME
	{ 0x000c, 0x03 },
	//PIX_CTRL_ELECTRICAL_BLACK
	{ 0x0017, 0x00 },
	//RW_CONTEXT
	{ 0x0000, 0x00 },
	//EXP_TIME
	{ 0x000e, 0x00 },
	{ 0x000f, 0x06 },
	//RW_CONTEXT
	{ 0x0000, 0x00 },
	//XWIN_LEFT
	{ 0x003a, 0x00 },
	{ 0x003b, 0x02 },
	//XWIN_RIGHT
	{ 0x003c, 0x04 },
	{ 0x003d, 0x39 },
	//XWIN_OB_LEFT
	{ 0x003f, 0x04 },
	{ 0x0040, 0x3e },
	//XWIN_OB_RIGHT
	{ 0x0041, 0x04 },
	{ 0x0042, 0x41 },
	//AUTO_RECALC_XPARAMS
	{ 0x0043, 0x01 },
	//RW_CONTEXT
	{ 0x0000, 0x00 },
	//YWIN_ENABLE
	{ 0x0012, 0x03 },
	//YWIN_OB_START
	{ 0x0019, 0x00 },
	{ 0x001a, 0x00 },
	//YWIN_OB_END
	{ 0x001b, 0x00 },
	{ 0x001c, 0x09 },
	//YWIN0_START
	{ 0x0022, 0x00 },
	{ 0x0023, 0x0a },
	//YWIN0_END
	{ 0x0024, 0x04 },
	{ 0x0025, 0x47 },
	//YWIN1_START
	{ 0x002b, 0x00 },
	{ 0x002c, 0x0a },
	//YWIN1_END
	{ 0x002d, 0x04 },
	{ 0x002e, 0x47 },
	//EXTERN_YADDR
	{ 0x0014, 0x00 },
	{ 0x0015, 0x00 },
	//FORCE_EXTERN_YADDR
	{ 0x0013, 0x00 },
	//RW_CONTEXT
	{ 0x0000, 0x00 },
	//YWIN_OB_SUBS_FACTOR
	{ 0x001d, 0x01 },
	//YWIN0_SUBS_FACTOR
	{ 0x0026, 0x01 },
	//YWIN1_SUBS_FACTOR
	{ 0x002f, 0x01 },
	//XSUBS_FACTOR
	{ 0x0039, 0x01 },
	//BINNING_MODE
	{ 0x0035, 0x00 },
	//BINNING_TYPE
	{ 0x0036, 0x11 },
	//BINNING_WEIGHTING
	{ 0x0037, 0x00 },
	//RW_CONTEXT
	{ 0x0000, 0x00 },
	//YWIN_CROP_ENABLE
	{ 0x0016, 0x01 },
	//YWIN_OB_CROP_OFFSET
	{ 0x001e, 0x00 },
	{ 0x001f, 0x02 },
	//YWIN_OB_CROP_HEIGHT
	{ 0x0020, 0x00 },
	{ 0x0021, 0x06 },
	//YWIN0_CROP_OFFSET
	{ 0x0027, 0x00 },
	{ 0x0028, 0x02 },
	//YWIN0_CROP_HEIGHT
	{ 0x0029, 0x03 },
	{ 0x002a, 0xd4 },
	//YWIN1_CROP_OFFSET
	{ 0x0030, 0x00 },
	{ 0x0031, 0x02 },
	//YWIN1_CROP_HEIGHT
	{ 0x0032, 0x03 },
	{ 0x0033, 0xd4 },
	//RW_CONTEXT
	{ 0x0000, 0x00 },
	//GDIG_CFA_IDX0
	{ 0x004e, 0x0f },
	//GDIG_CFA_IDX1
	{ 0x004f, 0x0f },
	//GDIG_CFA_IDX2
	{ 0x0050, 0x0f },
	//GDIG_CFA_IDX3
	{ 0x0051, 0x0f },
	//NEXT_ACTIVE_CONTEXT
	{ 0x0002, 0x00 },
	//NROF_FRAMES
	{ 0x0008, 0x00 },
	//META_INSERT_EN
	{ 0x0063, 0x00 },
	//MIPI_EMBEDDED_DATA_TYPE
	{ 0x00ab, 0x12 },
	//LVDS_EMBEDDED_DATA_TYPE
	{ 0x0064, 0x02 },
	//DPC_MODE
	{ 0x005a, 0x00 },
	//DPC_LIMIT_HIGH_SLOPE
	{ 0x005b, 0x02 },
	//DPC_LIMIT_HIGH_OFFSET
	{ 0x005c, 0x02 },
	//DPC_LIMIT_LOW_SLOPE
	{ 0x005d, 0x02 },
	//DPC_LIMIT_LOW_OFFSET
	{ 0x005e, 0x02 },
	//TEST_LVDS
	{ 0x007e, 0x00 },
	//TRAINING_WORD
	{ 0x007d, 0x0a },
	//MIPI_CLK_PN_CHANGE
	{ 0x00b3, 0x00 },
	//MIPI_DATA_PN_CHANGE
	{ 0x00b4, 0x00 },
	//MIPI_LANE0_SEL
	{ 0x0091, 0x00 },
	//MIPI_LANE1_SEL
	{ 0x0092, 0x01 },
	//DPHY_LANE0_SEL
	{ 0x00b5, 0x00 },
	//TSENS_ENABLE
	{ 0x006b, 0x01 },
	//TSENS_NOF_SAMPLES
	{ 0x0072, 0x01 },
	//TSENS_EWMA_ENABLE
	{ 0x006f, 0x01 },
	//TSENS_EWMA_FRAC_SEL
	{ 0x0070, 0x00 },
	//TSENS_RESET_EARLY
	{ 0x0073, 0x00 },
	//TSENS_CLKDIV
	{ 0x0071, 0x01 },
	//AMUX_BYPASS
	{ 0x021e, 0x01 },
	//ENABLE_AMUX
	{ 0x021d, 0x00 },
	//AMUX_SEL
	{ 0x021f, 0x3c },
	//TDIG_MODE
	{ 0x0222, 0x00 },
	//TDIG_FACTOR
	{ 0x0221, 0x00 },
	//DMUX_SEL
	{ 0x0220, 0x00 },
	//LFSR_EN
	{ 0x0052, 0x07 },
	//EN_AUTO_KERNEL_ENABLE_CALC
	{ 0x0046, 0x01 },
	//KERNEL_ENABLE_FORCE_VALUE
	{ 0x0047, 0x01 },
	//NR_ADC_CONVERSIONS
	{ 0x00f5, 0x0a },
	//KERNEL_ADDR_GLOBAL_CDS
	{ 0x00f6, 0x09 },
	//KERNEL_ADDR_COL0
	{ 0x00f7, 0x00 },
	//KERNEL_ADDR_COL1
	{ 0x00f8, 0x01 },
	//KERNEL_ADDR_COL2
	{ 0x00f9, 0x02 },
	//KERNEL_ADDR_COL3
	{ 0x00fa, 0x03 },
	//KERNEL_ADDR_COL4
	{ 0x00fb, 0x04 },
	//KERNEL_ADDR_COL5
	{ 0x00fc, 0x05 },
	//KERNEL_ADDR_COL6
	{ 0x00fd, 0x06 },
	//KERNEL_ADDR_COL7
	{ 0x00fe, 0x07 },
	//KERNEL_ADDR_COL8
	{ 0x00ff, 0x08 },
	//KERNEL_ADDR_COL9
	{ 0x0100, 0x09 },
	//KERNEL_ADDR_INVALID
	{ 0x0101, 0x0e },
	//NROF_FRAMES
	{ 0x0008, 0x00 },
	//MODE_SELECT
	{ 0x0007, 0x00 },
	//MODE_SELECT
	{ 0x0007, 0x01 },
	//RW_CONTEXT
	{ 0x0000, 0x00 },
	//RAMPGEN_SET
	{ 0x01fa, 0x00 },
	{ 0x01fb, 0x13 },
	//RW_CONTEXT
	{ 0x0000, 0x01 },
	//RAMPGEN_SET
	{ 0x01fa, 0x00 },
	{ 0x01fb, 0x13 },
	//RW_CONTEXT
	{ 0x0000, 0x02 },
	//RAMPGEN_SET
	{ 0x01fa, 0x00 },
	{ 0x01fb, 0x13 },
	//RW_CONTEXT
	{ 0x0000, 0x00 },
	//NEXT_ACTIVE_CONTEXT
	{ 0x0002, 0x00 },
	//OTP_ENABLE
	{ 0x0065, 0x01 },
	//CMD
	{ 0x0006, 0x28 },
	//CMD
	{ 0x0006, 0x20 },
	//OTP_BUSY
	//OTP_ENABLE
	{ 0x0065, 0x00 },
	//TRIM_SEL
	{ 0x01e2, 0x00 },
	//KERNEL_ADDR_GLOBAL_CDS
	{ 0x00f6, 0x09 },
	//KERNEL_ADDR_GLOBAL_CDS
	{ 0x00f6, 0x09 },
	//KERNEL_ADDR_GLOBAL_CDS
	{ 0x00f6, 0x09 },
	//KERNEL_ADDR_GLOBAL_CDS
	{ 0x00f6, 0x09 },


};


static const char *const poncha110_test_pattern_menu[] = {
	"Disabled",
	"Fixed Data",
	"2D Gradient",
};

static const int poncha110_test_pattern_val[] = {
	PONCHA110_TEST_PATTERN_DISABLE,
	PONCHA110_TEST_PATTERN_FIXED_DATA,
	PONCHA110_TEST_PATTERN_2D_GRADIENT,
};

/* regulator supplies */
static const char *const poncha110_supply_name[] = {
	// TODO(jalv): Check supply names
	/* Supplies can be enabled in any order */
	"VANA", /* Analog (2.8V) supply */
	"VDIG", /* Digital Core (1.8V) supply */
	"VDDL", /* IF (1.2V) supply */
};

#define PONCHA110_NUM_SUPPLIES ARRAY_SIZE(poncha110_supply_name)

/*
 * The supported formats. All flip/mirror combinations have the same byte order because the sensor
 * is monochrome
 */
static const u32 codes[] = {
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

/* Mode configs */
/*
 * Only one mode is exposed to the public (400x400 at 10 bit).
 * One code (10 bit) is exposed to public.
 * The public user specifies the code.
 * That is used to specify which internal supported_mode to use.
 */
#define PONCHA110_SUPPORTED_MODE_SIZE_PUBLIC 2
static const struct poncha110_mode supported_modes[] = {
	{
		/*gain 1-3 30 fps  mode */
		.width = PONCHA110_PIXEL_ARRAY_WIDTH,
		.height = PONCHA110_PIXEL_ARRAY_HEIGHT,
		.crop = {
			.left = PONCHA110_PIXEL_ARRAY_LEFT,
			.top = PONCHA110_PIXEL_ARRAY_TOP,
			.width = PONCHA110_PIXEL_ARRAY_WIDTH,
			.height = PONCHA110_PIXEL_ARRAY_HEIGHT},
		.reg_list_pre_soft_reset = {
			.num_of_regs = ARRAY_SIZE(full_10b_2lane_gain1_3_reg_pre_soft_reset),
			.regs = full_10b_2lane_gain1_3_reg_pre_soft_reset,
		},

		.min_vblank = PONCHA110_MIN_VBLANK,
		.max_vblank = PONCHA110_MAX_VBLANK,
		.hblank = PONCHA110_HBLANK_1_3, // TODO
		.row_length = PONCHA110_ROW_LENGTH_1_3,
		.bit_depth = 10,
		.code = MEDIA_BUS_FMT_SBGGR10_1X10,
		.gain_min = 0,
		.gain_max = 2, // this is means 0,1,2 correspond to 1x 2x 4x gain
	},
	{
		/* gain 1 10bit 60 fps crop mode */
		.width = PONCHA110_PIXEL_ARRAY_WIDTH,
		.height = 980,
		.crop = {
			.left = PONCHA110_PIXEL_ARRAY_LEFT,
			.top = PONCHA110_PIXEL_ARRAY_TOP,
			.width = PONCHA110_PIXEL_ARRAY_WIDTH,
			.height = 980},
		.reg_list_pre_soft_reset = {
			.num_of_regs = ARRAY_SIZE(crop_980_10b_2lane_gain1_reg_pre_soft_reset),
			.regs = crop_980_10b_2lane_gain1_reg_pre_soft_reset,
		},

		.min_vblank = PONCHA110_MIN_VBLANK,
		.max_vblank = PONCHA110_MAX_VBLANK,
		.hblank = PONCHA110_HBLANK_1, // TODO
		.row_length = PONCHA110_ROW_LENGTH_1,
		.bit_depth = 10,
		.code = MEDIA_BUS_FMT_SBGGR10_1X10,
		.gain_min = 0,
		.gain_max = 0, // this is means 0,1,2 correspond to 1x 2x 4x gain
	},


};

struct poncha110
{
	struct v4l2_subdev sd;
	struct media_pad pad[NUM_PADS];

	struct v4l2_mbus_framefmt fmt;

	struct clk *xclk; /* system clock to PONCHA110 */
	u32 xclk_freq;

	// struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[PONCHA110_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *gain;
	// custom v4l2 control
	struct v4l2_ctrl *mira_reg_w;
	struct v4l2_ctrl *mira_reg_r;
	u16 mira_reg_w_cached_addr;
	u8 mira_reg_w_cached_flag;

	/* Current mode */
	const struct poncha110_mode *mode;
	/* current bit depth, may defer from mode->bit_depth */
	u8 bit_depth;
	/* OTP_CALIBRATION_VALUE stored in OTP memory */
	u16 otp_cal_val;
	/* Whether to skip base register sequence upload */
	u32 skip_reg_upload;
	/* Whether to reset sensor when stream on/off */
	u32 skip_reset;
	/* Whether regulator and clk are powered on */
	u32 powered;

	/* A flag to force write_start/stop_streaming_regs even if (skip_reg_upload==1) */
	u32 target_frame_time;
	u32 row_length;
	u8 force_stream_ctrl;

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;

	/* pmic and uC */
	struct i2c_client *pmic_client;
	struct i2c_client *uc_client;
	struct i2c_client *led_client;
	/* User specified I2C device address */
	u32 tbd_client_i2c_addr;
};

static inline struct poncha110 *to_poncha110(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct poncha110, sd);
}

static int poncha110_read(struct poncha110 *poncha110, u16 reg, u8 *val)
{
	int ret;
	unsigned char data_w[2] = {reg >> 8, reg & 0xff};
	struct i2c_client *client = v4l2_get_subdevdata(&poncha110->sd);

	ret = i2c_master_send(client, data_w, 2);
	/*
	 * A negative return code, or sending the wrong number of bytes, both
	 * count as an error.
	 */
	if (ret != 2)
	{
		dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
		return ret;
	}

	ret = i2c_master_recv(client, val, 1);
	/*
	 * The only return value indicating success is 1. Anything else, even
	 * a non-negative value, indicates something went wrong.
	 */
	if (ret == 1)
	{
		ret = 0;
	}
	else
	{
		dev_dbg(&client->dev, "%s: i2c read error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
	}

	return ret;
}

static int poncha110_write(struct poncha110 *poncha110, u16 reg, u8 val)
{
	int ret;
	unsigned char data[3] = {reg >> 8, reg & 0xff, val};
	struct i2c_client *client = v4l2_get_subdevdata(&poncha110->sd);

	ret = i2c_master_send(client, data, 3);

	/*
	 * Writing the wrong number of bytes also needs to be flagged as an
	 * error. Success needs to produce a 0 return code.
	 */
	if (ret == 3)
	{
		ret = 0;
	}
	else
	{
		dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
	}

	/*
	 * The code below is for debug purpose.
	 * It reads back the written values.
	 * Some registers have different read and write addresses.
	 * These registers typically have WR addr 0xE... but RD addr 0x4...
	 */
	/*
	{
		usleep_range(50, 300);
		u8 ret_val;
		u16 ret_reg;
		if (((reg >>12) & 0x000F) == 0x000E) {
			ret_reg = ((reg & 0x0FFF) | 0x4000);
		} else {
			ret_reg = reg;
		}
		ret = poncha110_read(poncha110, ret_reg, &ret_val);
		printk(KERN_INFO "[PONCHA110]: Write reg 0x%4.4x, Read ret_reg 0x%4.4x, val = 0x%x.\n",
				reg, ret_reg, ret_val);
		if (val != ret_val) {
			printk(KERN_INFO "[PONCHA110]: WARNING Write reg 0x%4.4x, val = 0x%x, read ret_reg = 0x%4.4x, ret_val = 0x%x.\n",
				reg, val, ret_reg, ret_val);
		}
	}
	*/

	return ret;
}

/*
 * poncha110 is big-endian: msb of val goes to lower reg addr
 */
/* Temporary commented out, because it is not yet used. */

static int poncha110_write_be16(struct poncha110 *poncha110, u16 reg, u16 val)
{
	   int ret;
	   unsigned char data[4] = { reg >> 8, reg & 0xff, (val >> 8) & 0xff, val & 0xff };
	   struct i2c_client *client = v4l2_get_subdevdata(&poncha110->sd);

	   ret = i2c_master_send(client, data, 4);
	   //
	   // Writing the wrong number of bytes also needs to be flagged as an
	   // error. Success needs to produce a 0 return code.
	   //
	   if (ret == 4) {
			   ret = 0;
	   } else {
			   dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
							   __func__, reg);
			   if (ret >= 0)
					   ret = -EINVAL;
	   }

	   return ret;
}

/*
 * poncha110 is big-endian: msb of val goes to lower reg addr
 */
static int poncha110_write_be24(struct poncha110 *poncha110, u16 reg, u32 val)
{
	int ret;
	unsigned char data[5] = {reg >> 8, reg & 0xff, (val >> 16) & 0xff, (val >> 8) & 0xff, val & 0xff};
	struct i2c_client *client = v4l2_get_subdevdata(&poncha110->sd);

	ret = i2c_master_send(client, data, 5);
	/*
	 * Writing the wrong number of bytes also needs to be flagged as an
	 * error. Success needs to produce a 0 return code.
	 */
	if (ret == 5)
	{
		ret = 0;
	}
	else
	{
		dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
	}

	return ret;
}

/*
 * poncha110 is big-endian: msb of val goes to lower reg addr
 */
static int poncha110_write_be32(struct poncha110 *poncha110, u16 reg, u32 val)
{
	int ret;
	unsigned char data[6] = {reg >> 8, reg & 0xff, (val >> 24) & 0xff, (val >> 16) & 0xff, (val >> 8) & 0xff, val & 0xff};
	struct i2c_client *client = v4l2_get_subdevdata(&poncha110->sd);

	ret = i2c_master_send(client, data, 6);
	/*
	 * Writing the wrong number of bytes also needs to be flagged as an
	 * error. Success needs to produce a 0 return code.
	 */
	if (ret == 6)
	{
		ret = 0;
	}
	else
	{
		dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
	}

	return ret;
}

/*
 * poncha110 OTP 32-bit val on I2C is big-endian. However, val content can be little-endian.
 */
static int poncha110_read_be32(struct poncha110 *poncha110, u16 reg, u32 *val)
{
	int ret;
	unsigned char data_w[2] = {reg >> 8, reg & 0xff};
	/* Big-endian 32-bit buffer. */
	unsigned char data_r[4];
	struct i2c_client *client = v4l2_get_subdevdata(&poncha110->sd);

	ret = i2c_master_send(client, data_w, 2);
	/*
	 * A negative return code, or sending the wrong number of bytes, both
	 * count as an error.
	 */
	if (ret != 2)
	{
		dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
		return ret;
	}

	ret = i2c_master_recv(client, data_r, 4);
	*val = (u32)((data_r[0] << 24) | (data_r[1] << 16) | (data_r[2] << 8) | data_r[3]);
	/*
	 * The only return value indicating success is 4. Anything else, even
	 * a non-negative value, indicates something went wrong.
	 */
	if (ret == 4)
	{
		ret = 0;
	}
	else
	{
		dev_dbg(&client->dev, "%s: i2c read error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
	}

	return ret;
}

/* Write a list of registers */
static int poncha110_write_regs(struct poncha110 *poncha110,
							  const struct poncha110_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&poncha110->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++)
	{
		ret = poncha110_write(poncha110, regs[i].address, regs[i].val);
		if (ret)
		{
			dev_err_ratelimited(&client->dev,
								"Failed to write reg 0x%4.4x. error = %d\n",
								regs[i].address, ret);

			return ret;
		}
		else
		{
			// Debug code below
			// u8 val;
			// ret = poncha110_read(poncha110, regs[i].address, &val);
			// printk(KERN_INFO "[PONCHA110]: Read reg 0x%4.4x, val = 0x%x.\n",
			// 		regs[i].address, val);

		}
	}

	return 0;
}

/*
 * Read OTP memory: 8-bit addr and 32-bit value
 */
static int poncha110_otp_read(struct poncha110 *poncha110, u8 addr, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&poncha110->sd);
	u8 busy_status = 1;
	int poll_cnt = 0;
	int poll_cnt_max = 10;
	int ret;
	poncha110_write(poncha110, PONCHA110_OTP_COMMAND, 0);
	poncha110_write(poncha110, PONCHA110_OTP_ADDR, addr);
	poncha110_write(poncha110, PONCHA110_OTP_START, 1);
	usleep_range(5, 10);
	poncha110_write(poncha110, PONCHA110_OTP_START, 0);
	for (poll_cnt = 0; poll_cnt < poll_cnt_max; poll_cnt++)
	{
		poncha110_read(poncha110, PONCHA110_OTP_BUSY, &busy_status);
		if (busy_status == 0)
		{
			break;
		}
	}
	if (poll_cnt < poll_cnt_max && busy_status == 0)
	{
		ret = poncha110_read_be32(poncha110, PONCHA110_OTP_DOUT, val);
	}
	else
	{
		dev_dbg(&client->dev, "%s: OTP memory busy, skip raeding addr: 0x%X\n",
				__func__, addr);
		ret = -EINVAL;
	}

	return ret;
}

/* Write PMIC registers, and can be reused to write microcontroller reg. */
static int poncha110pmic_write(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	unsigned char data[2] = {reg & 0xff, val};

	ret = i2c_master_send(client, data, 2);
	/*
	 * Writing the wrong number of bytes also needs to be flagged as an
	 * error. Success needs to produce a 0 return code.
	 */
	if (ret == 2)
	{
		ret = 0;
	}
	else
	{
		dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
	}

	return ret;
}

static int poncha110pmic_read(struct i2c_client *client, u8 reg, u8 *val)
{
	struct i2c_msg msgs[2];
	u8 addr_buf[1] = {reg & 0xff};
	u8 data_buf[1] = {0};
	int ret;

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = &data_buf[0];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = (u8)(data_buf[0]);

	return 0;
}

/* Power/clock management functions */
static int poncha110_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct poncha110 *poncha110 = to_poncha110(sd);
	int ret = -EINVAL;

	printk(KERN_INFO "[PONCHA110]: Entering power on function.\n");

	if (poncha110->powered == 0)
	{
		ret = regulator_bulk_enable(PONCHA110_NUM_SUPPLIES, poncha110->supplies);
		if (ret)
		{
			dev_err(&client->dev, "%s: failed to enable regulators\n",
					__func__);
			return ret;
		}

		ret = clk_prepare_enable(poncha110->xclk);
		if (ret)
		{
			dev_err(&client->dev, "%s: failed to enable clock\n",
					__func__);
			goto reg_off;
		}
		usleep_range(PONCHA110_XCLR_MIN_DELAY_US,
					 PONCHA110_XCLR_MIN_DELAY_US + PONCHA110_XCLR_DELAY_RANGE_US);
		poncha110->powered = 1;
	}
	else
	{
		printk(KERN_INFO "[PONCHA110]: Skip regulator and clk enable, because poncha110->powered == %d.\n", poncha110->powered);
	}

	return 0;

reg_off:
	ret = regulator_bulk_disable(PONCHA110_NUM_SUPPLIES, poncha110->supplies);
	return ret;
}

static int poncha110_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct poncha110 *poncha110 = to_poncha110(sd);

	printk(KERN_INFO "[PONCHA110]: Entering power off function.\n");

	if (poncha110->skip_reset == 0)
	{
		if (poncha110->powered == 1)
		{
			regulator_bulk_disable(PONCHA110_NUM_SUPPLIES, poncha110->supplies);
			clk_disable_unprepare(poncha110->xclk);
			poncha110->powered = 0;
		}
		else
		{
			printk(KERN_INFO "[PONCHA110]: Skip disabling regulator and clk due to poncha110->powered == %d.\n", poncha110->powered);
		}
	}
	else
	{
		printk(KERN_INFO "[PONCHA110]: Skip disabling regulator and clk due to poncha110->skip_reset=%u.\n", poncha110->skip_reset);
	}

	return 0;
}

static int poncha110_v4l2_reg_w(struct poncha110 *poncha110, u32 value)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&poncha110->sd);
	u32 ret = 0;
	u32 tmp_flag;

	u16 reg_addr = (value >> 8) & 0xFFFF;
	u8 reg_val = value & 0xFF;
	u8 reg_flag = (value >> 24) & 0xFF;

	// printk(KERN_INFO "[PONCHA110]: %s reg_flag: 0x%02X; reg_addr: 0x%04X; reg_val: 0x%02X.\n",
	// 		__func__, reg_flag, reg_addr, reg_val);

	if (reg_flag & AMS_CAMERA_CID_PONCHA110_REG_FLAG_CMD_SEL)
	{
		if (reg_flag == AMS_CAMERA_CID_PONCHA110_REG_FLAG_SLEEP_US)
		{
			// If it is for sleep, combine all 24 bits of reg_addr and reg_val as sleep us.
			u32 sleep_us_val = value & 0x00FFFFFF;
			// Sleep range needs an interval, default to 1/8 of the sleep value.
			u32 sleep_us_interval = sleep_us_val >> 3;
			printk(KERN_INFO "[PONCHA110]: %s sleep_us: %u.\n", __func__, sleep_us_val);
			usleep_range(sleep_us_val, sleep_us_val + sleep_us_interval);
		}
		else if (reg_flag == AMS_CAMERA_CID_PONCHA110_REG_FLAG_RESET_ON)
		{
			printk(KERN_INFO "[PONCHA110]: %s Enable reset at stream on/off.\n", __func__);
			poncha110->skip_reset = 0;
		}
		else if (reg_flag == AMS_CAMERA_CID_PONCHA110_REG_FLAG_RESET_OFF)
		{
			printk(KERN_INFO "[PONCHA110]: %s Disable reset at stream on/off.\n", __func__);
			poncha110->skip_reset = 1;
		}
		else if (reg_flag == AMS_CAMERA_CID_PONCHA110_REG_FLAG_REG_UP_ON)
		{
			printk(KERN_INFO "[PONCHA110]: %s Enable base register sequence upload.\n", __func__);
			poncha110->skip_reg_upload = 0;
		}
		else if (reg_flag == AMS_CAMERA_CID_PONCHA110_REG_FLAG_REG_UP_OFF)
		{
			printk(KERN_INFO "[PONCHA110]: %s Disable base register sequence upload.\n", __func__);
			poncha110->skip_reg_upload = 1;
		}
		else if (reg_flag == AMS_CAMERA_CID_PONCHA110_REG_FLAG_POWER_ON)
		{
			printk(KERN_INFO "[PONCHA110]: %s Call power on function poncha110_power_on().\n", __func__);
			/* Temporarily disable skip_reset if manually doing power on/off */
			tmp_flag = poncha110->skip_reset;
			poncha110->skip_reset = 0;
			poncha110_power_on(&client->dev);
			poncha110->skip_reset = tmp_flag;
		}
		else if (reg_flag == AMS_CAMERA_CID_PONCHA110_REG_FLAG_POWER_OFF)
		{
			printk(KERN_INFO "[PONCHA110]: %s Call power off function poncha110_power_off().\n", __func__);
			/* Temporarily disable skip_reset if manually doing power on/off */
			tmp_flag = poncha110->skip_reset;
			poncha110->skip_reset = 0;
			poncha110_power_off(&client->dev);
			poncha110->skip_reset = tmp_flag;
		}
		else if (reg_flag == AMS_CAMERA_CID_PONCHA110_REG_FLAG_STREAM_CTRL_ON)
		{
			printk(KERN_INFO "[PONCHA110]: %s Force stream control even if (skip_reg_upload == 1).\n", __func__);
			poncha110->force_stream_ctrl = 1;
		}
		else if (reg_flag == AMS_CAMERA_CID_PONCHA110_REG_FLAG_STREAM_CTRL_OFF)
		{
			printk(KERN_INFO "[PONCHA110]: %s Disable stream control if (skip_reg_upload == 1).\n", __func__);
			poncha110->force_stream_ctrl = 0;
		}
		else
		{
			printk(KERN_INFO "[PONCHA110]: %s unknown command from flag %u, ignored.\n", __func__, reg_flag);
		}
	}
	else if (reg_flag & AMS_CAMERA_CID_PONCHA110_REG_FLAG_FOR_READ)
	{
		// If it is for read, skip reagister write, cache addr and flag for read.
		poncha110->mira_reg_w_cached_addr = reg_addr;
		poncha110->mira_reg_w_cached_flag = reg_flag;
	}
	else
	{
		// If it is for write, select which I2C device by the flag "I2C_SEL".
		if ((reg_flag & AMS_CAMERA_CID_PONCHA110_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_PONCHA110_REG_FLAG_I2C_MIRA)
		{
			// Before writing Poncha110 register, first optionally select BANK and CONTEXT
			if (reg_flag & AMS_CAMERA_CID_PONCHA110_REG_FLAG_USE_BANK)
			{
				u8 bank;
				u8 context;
				// Set conetxt bank 0 or 1
				if (reg_flag & AMS_CAMERA_CID_PONCHA110_REG_FLAG_BANK)
				{
					bank = 1;
				}
				else
				{
					bank = 0;
				}
				// printk(KERN_INFO "[PONCHA110]: %s select bank: %u.\n", __func__, bank);

				// Set context bank 1A or bank 1B
				if (reg_flag & AMS_CAMERA_CID_PONCHA110_REG_FLAG_CONTEXT)
				{
					context = 1;
				}
				else
				{
					context = 0;
				}
				// printk(KERN_INFO "[PONCHA110]: %s select context: %u.\n", __func__, context);
				ret = poncha110_write(poncha110, PONCHA110_CONTEXT_REG, context);
				if (ret)
				{
					dev_err(&client->dev, "Error setting RW_CONTEXT.");
					return ret;
				}
			}
			// Writing the actual Poncha110 register
			// printk(KERN_INFO "[PONCHA110]: %s write reg_addr: 0x%04X; reg_val: 0x%02X.\n", __func__, reg_addr, reg_val);
			ret = poncha110_write(poncha110, reg_addr, reg_val);
			if (ret)
			{
				dev_err_ratelimited(&client->dev, "Error AMS_CAMERA_CID_MIRA_REG_W reg_addr %X.\n", reg_addr);
				return -EINVAL;
			}
		}
		else if ((reg_flag & AMS_CAMERA_CID_PONCHA110_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_PONCHA110_REG_FLAG_I2C_SET_TBD)
		{
			/* User tries to set TBD I2C address, store reg_val to poncha110->tbd_client_i2c_addr. Skip write. */
			printk(KERN_INFO "[PONCHA110]: poncha110->tbd_client_i2c_addr = 0x%X.\n", reg_val);
			poncha110->tbd_client_i2c_addr = reg_val;
		}
		else if ((reg_flag & AMS_CAMERA_CID_PONCHA110_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_PONCHA110_REG_FLAG_I2C_TBD)
		{
			if (poncha110->tbd_client_i2c_addr == PONCHA110PMIC_I2C_ADDR)
			{
				// Write PMIC. Use pre-allocated poncha110->pmic_client.
				printk(KERN_INFO "[PONCHA110]: write pmic_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
				ret = poncha110pmic_write(poncha110->pmic_client, (u8)(reg_addr & 0xFF), reg_val);
			}
			else if (poncha110->tbd_client_i2c_addr == PONCHA110UC_I2C_ADDR)
			{
				// Write micro-controller. Use pre-allocated poncha110->uc_client.
				printk(KERN_INFO "[PONCHA110]: write uc_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
				ret = poncha110pmic_write(poncha110->uc_client, (u8)(reg_addr & 0xFF), reg_val);
			}
			else if (poncha110->tbd_client_i2c_addr == PONCHA110LED_I2C_ADDR)
			{
				// Write LED driver. Use pre-allocated poncha110->led_client.
				printk(KERN_INFO "[PONCHA110]: write led_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
				ret = poncha110pmic_write(poncha110->led_client, (u8)(reg_addr & 0xFF), reg_val);
			}
			else
			{
				/* Write other TBD I2C address.
				 * The TBD I2C address is set via AMS_CAMERA_CID_PONCHA110_REG_FLAG_I2C_SET_TBD.
				 * The TBD I2C address is stored in poncha110->tbd_client_i2c_addr.
				 * A temporary I2C client, tmp_client, is created and then destroyed (unregistered).
				 */
				struct i2c_client *tmp_client;
				tmp_client = i2c_new_dummy_device(client->adapter, poncha110->tbd_client_i2c_addr);
				if (IS_ERR(tmp_client))
					return PTR_ERR(tmp_client);
				printk(KERN_INFO "[PONCHA110]: write tbd_client, i2c_addr %u, reg_addr 0x%X, reg_val 0x%X.\n",
					   poncha110->tbd_client_i2c_addr, (u8)(reg_addr & 0xFF), reg_val);
				ret = poncha110pmic_write(tmp_client, (u8)(reg_addr & 0xFF), reg_val);
				i2c_unregister_device(tmp_client);
			}
		}
	}

	return 0;
}

static int poncha110_v4l2_reg_r(struct poncha110 *poncha110, u32 *value)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&poncha110->sd);
	u32 ret = 0;

	u16 reg_addr = poncha110->mira_reg_w_cached_addr;
	u8 reg_flag = poncha110->mira_reg_w_cached_flag;
	u8 reg_val = 0;

	*value = 0;

	if ((reg_flag & AMS_CAMERA_CID_PONCHA110_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_PONCHA110_REG_FLAG_I2C_MIRA)
	{
		if (reg_flag & AMS_CAMERA_CID_PONCHA110_REG_FLAG_USE_BANK)
		{
			u8 bank;
			u8 context;
			// Set conetxt bank 0 or 1
			if (reg_flag & AMS_CAMERA_CID_PONCHA110_REG_FLAG_BANK)
			{
				bank = 1;
			}
			else
			{
				bank = 0;
			}
			// printk(KERN_INFO "[PONCHA110]: %s select bank: %u.\n", __func__, bank);
			// Set context bank 1A or bank 1B
			if (reg_flag & AMS_CAMERA_CID_PONCHA110_REG_FLAG_CONTEXT)
			{
				context = 1;
			}
			else
			{
				context = 0;
			}
			// printk(KERN_INFO "[PONCHA110]: %s select context: %u.\n", __func__, context);
			ret = poncha110_write(poncha110, PONCHA110_CONTEXT_REG, context);
			if (ret)
			{
				dev_err(&client->dev, "Error setting RW_CONTEXT.");
				return ret;
			}
		}
		ret = poncha110_read(poncha110, reg_addr, &reg_val);
		if (ret)
		{
			dev_err_ratelimited(&client->dev, "Error AMS_CAMERA_CID_MIRA_REG_R reg_addr %X.\n", reg_addr);
			return -EINVAL;
		}
	}
	else if ((reg_flag & AMS_CAMERA_CID_PONCHA110_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_PONCHA110_REG_FLAG_I2C_TBD)
	{
		if (poncha110->tbd_client_i2c_addr == PONCHA110PMIC_I2C_ADDR)
		{
			// Read PMIC. Use pre-allocated poncha110->pmic_client.
			ret = poncha110pmic_read(poncha110->pmic_client, (u8)(reg_addr & 0xFF), &reg_val);
			printk(KERN_INFO "[PONCHA110]: read pmic_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
		}
		else if (poncha110->tbd_client_i2c_addr == PONCHA110UC_I2C_ADDR)
		{
			// Read micro-controller. Use pre-allocated poncha110->uc_client.
			ret = poncha110pmic_read(poncha110->uc_client, (u8)(reg_addr & 0xFF), &reg_val);
			printk(KERN_INFO "[PONCHA110]: read uc_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
		}
		else if (poncha110->tbd_client_i2c_addr == PONCHA110LED_I2C_ADDR)
		{
			// Read LED driver. Use pre-allocated poncha110->led_client.
			ret = poncha110pmic_read(poncha110->led_client, (u8)(reg_addr & 0xFF), &reg_val);
			printk(KERN_INFO "[PONCHA110]: read led_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
		}
		else
		{
			/* Read other TBD I2C address.
			 * The TBD I2C address is set via AMS_CAMERA_CID_PONCHA110_REG_FLAG_I2C_SET_TBD.
			 * The TBD I2C address is stored in poncha110->tbd_client_i2c_addr.
			 * A temporary I2C client, tmp_client, is created and then destroyed (unregistered).
			 */
			struct i2c_client *tmp_client;
			tmp_client = i2c_new_dummy_device(client->adapter, poncha110->tbd_client_i2c_addr);
			if (IS_ERR(tmp_client))
				return PTR_ERR(tmp_client);
			ret = poncha110pmic_read(tmp_client, (u8)(reg_addr & 0xFF), &reg_val);
			printk(KERN_INFO "[PONCHA110]: read tbd_client, i2c_addr %u, reg_addr 0x%X, reg_val 0x%X.\n",
				   poncha110->tbd_client_i2c_addr, (u8)(reg_addr & 0xFF), reg_val);
			i2c_unregister_device(tmp_client);
		}
	}

	// Return 32-bit value that includes flags, addr, and register value
	*value = ((u32)reg_flag << 24) | ((u32)reg_addr << 8) | (u32)reg_val;

	// printk(KERN_INFO "[PONCHA110]: poncha110_v4l2_reg_r() reg_flag: 0x%02X; reg_addr: 0x%04X, reg_val: 0x%02X.\n",
	// 		reg_flag, reg_addr, reg_val);

	return 0;
}



static int poncha110_write_analog_gain_reg(struct poncha110 *poncha110, u8 gain) {
	struct i2c_client* const client = v4l2_get_subdevdata(&poncha110->sd);
	u32 ret = 0;
	u8 gainval;
	if (gain | PONCHA110_ANALOG_GAIN_MAX) {
		//MODE_SELECT


		int poll_cnt = 0;
		int poll_cnt_max = 4;
		int ret;
		// poncha110_write(poncha110, PONCHA110_OTP_COMMAND, 0);
		// poncha110_write(poncha110, PONCHA110_OTP_ADDR, addr);
		// poncha110_write(poncha110, PONCHA110_OTP_START, 1);
		// poncha110_write(poncha110, PONCHA110_OTP_START, 0);

		usleep_range(70000, 150000);
		ret |= poncha110_write(poncha110, PONCHA110_CONTEXT_REG, 0);
		gainval = (gain<<5) | PONCHA110_ANALOG_GAIN_TRIM;
		ret |= poncha110_write(poncha110, PONCHA110_ANALOG_GAIN_REG, gainval);
		printk(KERN_INFO "[PONCHA110]: ANALOG GAIN gainval reg %u, gain %u.\n",gainval, gain);



	ret |= poncha110_write(poncha110, 0x0007, 0x00);

	ret |= poncha110_write(poncha110, 0x0007, 0x01);

	}
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
	}
	return 0;
}


// Returns the maximum exposure time in microseconds (reg value)
static u32 poncha110_calculate_max_exposure_time(u32 row_length, u32 vsize,
											   u32 vblank)
{
	(void)(row_length);
	(void)(vsize);
	(void)(vblank);
	/* Poncha110 does not have a max exposure limit besides register bits */
	// return row_length * (vsize + vblank) - PONCHA110_GLOB_NUM_CLK_CYCLES;
	return PONCHA110_EXPOSURE_MAX;
}

static int poncha110_write_exposure_reg(struct poncha110 *poncha110, u32 exposure)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&poncha110->sd);
	const u32 min_exposure = PONCHA110_EXPOSURE_MIN;
	const u32 max_exposure = PONCHA110_EXPOSURE_MAX;

	// u32 max_exposure = poncha110->exposure->maximum;
	u32 ret = 0;

	if (exposure < min_exposure)
	{
		exposure = min_exposure;
	}
	if (exposure > max_exposure)
	{
		exposure = max_exposure;
	}

	printk(KERN_INFO "[PONCHA110]: write exp reg = %d.  \n", exposure);
	// printk(KERN_INFO "[PONCHA110]: poncha110 write exp reg 0x%02X; reg_addr: 0x%04X, reg_val: 0x%02X.\n",
	/* Write Bank 1 context 0 */

	ret = poncha110_write_be16(poncha110, PONCHA110_CONTEXT_REG, 0);

	ret = poncha110_write_be16(poncha110, PONCHA110_EXPOSURE_REG, exposure);
	// ret = poncha110_write(poncha110, PONCHA110_BANK_SEL_REG, 1);
	// ret = poncha110_write_be32(poncha110, PONCHA110_EXP_TIME_L_REG, exposure);
	// /* Write Bank 1 context 1 */
	// ret = poncha110_write(poncha110, PONCHA110_CONTEXT_REG, 1);
	// ret = poncha110_write_be32(poncha110, PONCHA110_EXP_TIME_L_REG, exposure);
	if (ret)
	{
		dev_err_ratelimited(&client->dev, "Error setting exposure time to %d", exposure);
		return -EINVAL;
	}

	u8 val;

	ret = poncha110_read(poncha110, 0x00E, &val);
	printk(KERN_INFO "[PONCHA110]: Read reg 0x%4.4x, val = 0x%x.\n",
		   0x00E, val);
	ret = poncha110_read(poncha110, 0x00F, &val);
	printk(KERN_INFO "[PONCHA110]: Read reg  0x%4.4x, val = 0x%x.\n",
		   0x00F, val);
	// if (poncha110->illum_width_auto == 1)
	// {
	// 	poncha110->illum_width = exposure * PONCHA110_DATA_RATE / 8;
	// 	poncha110_write_illum_trig_regs(poncha110);
	// }
	// some delay
	// printk(KERN_INFO "[PONCHA110]: exposure function, before delay..\n");
	// usleep_range(PONCHA110_XCLR_MIN_DELAY_US,
	// 				 PONCHA110_XCLR_MIN_DELAY_US + PONCHA110_XCLR_DELAY_RANGE_US);
	// printk(KERN_INFO "[PONCHA110]: exposure function, after delay..\n");

	return 0;
}

static int poncha110_write_target_frame_time_reg(struct poncha110 *poncha110, u16 target_frame_time)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&poncha110->sd);
	u32 ret = 0;

	/* Write Bank 1 context 1 */
	//ret = poncha110_write(poncha110, PONCHA110_CONTEXT_REG, 1);
	ret = poncha110_write_be16(poncha110, PONCHA110_TARGET_FRAME_TIME_REG, target_frame_time);
	if (ret)
	{
		dev_err_ratelimited(&client->dev, "Error setting target frame time to %d", target_frame_time);
		return -EINVAL;
	}

	return 0;
}

static int poncha110_write_start_streaming_regs(struct poncha110 *poncha110)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&poncha110->sd);
	int ret = 0;

	// // mode_select
	// {0x0007, 0x00},
	// // mode_select
	// {0x0007, 0x01},

	// Raising CMD_REQ_1 to 1 for REQ_EXP
	ret = poncha110_write(poncha110, 0x0007,
						0);
	if (ret)
	{
		dev_err(&client->dev, "Error setting CMD_REQ_1 to 0 for REQ_EXP.");
		return ret;
	}

	usleep_range(10, 20);

	// Setting CMD_REQ_1 tp 0 for REQ_EXP
	ret = poncha110_write(poncha110, 0x0007,
						1);
	if (ret)
	{
		dev_err(&client->dev, "Error setting CMD_REQ_1 to 1 for REQ_EXP.");
		return ret;
	}
	usleep_range(10, 20);

	return ret;
}

static int poncha110_write_stop_streaming_regs(struct poncha110 *poncha110)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&poncha110->sd);
	int ret = 0;
	printk(KERN_INFO "[PONCHA110]: poncha110_write_stop_streaming_regs\n");


	return ret;
}

// Gets the format code if supported. Otherwise returns the default format code `codes[0]`
static u32 poncha110_validate_format_code_or_default(struct poncha110 *poncha110, u32 code)
{
	struct i2c_client *client = v4l2_get_subdevdata(&poncha110->sd);
	unsigned int i;
	printk(KERN_INFO "[PONCHA110]: validate format code or default. .\n");

	lockdep_assert_held(&poncha110->mutex);

	for (i = 0; i < ARRAY_SIZE(codes); i++)
		if (codes[i] == code)
			break;

	if (i >= ARRAY_SIZE(codes))
	{
		dev_err_ratelimited(&client->dev, "Could not set requested format code %u", code);
		dev_err_ratelimited(&client->dev, "Using default format %u", codes[0]);
		i = 0;
	}

	return codes[i];
}

static void poncha110_set_default_format(struct poncha110 *poncha110)
{
	struct v4l2_mbus_framefmt *fmt;
	printk(KERN_INFO "[PONCHA110]: poncha110_set_default_format\n");

	fmt = &poncha110->fmt;
	fmt->code = MEDIA_BUS_FMT_SBGGR10_1X10; // MEDIA_BUS_FMT_Y10_1X10;
	poncha110->bit_depth = 10;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
													  fmt->colorspace,
													  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width = supported_modes[0].width;
	fmt->height = supported_modes[0].height;
	fmt->field = V4L2_FIELD_NONE;
}

static int poncha110_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct poncha110 *poncha110 = to_poncha110(sd);
	struct v4l2_mbus_framefmt *try_fmt_img =
		v4l2_subdev_get_try_format(sd, fh->state, IMAGE_PAD);
	struct v4l2_mbus_framefmt *try_fmt_meta =
		v4l2_subdev_get_try_format(sd, fh->state, METADATA_PAD);
	struct v4l2_rect *try_crop;

	mutex_lock(&poncha110->mutex);

	/* Initialize try_fmt for the image pad */
	try_fmt_img->width = supported_modes[0].width;
	try_fmt_img->height = supported_modes[0].height;
	try_fmt_img->code = poncha110_validate_format_code_or_default(poncha110,
																MEDIA_BUS_FMT_SBGGR10_1X10);
	try_fmt_img->field = V4L2_FIELD_NONE;

	/* TODO(jalv): Initialize try_fmt for the embedded metadata pad */
	try_fmt_meta->width = PONCHA110_EMBEDDED_LINE_WIDTH;
	try_fmt_meta->height = PONCHA110_NUM_EMBEDDED_LINES;
	try_fmt_meta->code = MEDIA_BUS_FMT_SENSOR_DATA;
	try_fmt_meta->field = V4L2_FIELD_NONE;

	/* Initialize try_crop rectangle. */
	try_crop = v4l2_subdev_get_try_crop(sd, fh->state, 0);
	try_crop->top = PONCHA110_PIXEL_ARRAY_TOP;
	try_crop->left = PONCHA110_PIXEL_ARRAY_LEFT;
	try_crop->width = PONCHA110_PIXEL_ARRAY_WIDTH;
	try_crop->height = PONCHA110_PIXEL_ARRAY_HEIGHT;

	mutex_unlock(&poncha110->mutex);

	return 0;
}

static int poncha110_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct poncha110 *poncha110 =
		container_of(ctrl->handler, struct poncha110, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&poncha110->sd);
	int ret = 0;
	// u32 target_frame_time_us;

	// Debug print
	// printk(KERN_INFO "[PONCHA110]: poncha110_set_ctrl() id: 0x%X value: 0x%X.\n", ctrl->id, ctrl->val);

	// if (ctrl->id == V4L2_CID_VBLANK)
	// {
	// 	int exposure_max, exposure_def;

		/* Update max exposure while meeting expected vblanking */
	// 	exposure_max = poncha110_calculate_max_exposure_time(PONCHA110_MIN_ROW_LENGTH,
	// 													   poncha110->mode->height,
	// 													   ctrl->val);
	// 	exposure_def = (exposure_max < PONCHA110_DEFAULT_EXPOSURE) ? exposure_max : PONCHA110_DEFAULT_EXPOSURE;
	// 	__v4l2_ctrl_modify_range(poncha110->exposure,
	// 							 poncha110->exposure->minimum,
	// 							 (int)(1 + exposure_max / PONCHA110_MIN_ROW_LENGTH_US), poncha110->exposure->step,
	// 							 (int)(1 + exposure_def / PONCHA110_MIN_ROW_LENGTH_US));
	// }

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(&client->dev) == 0)
	{
		dev_info(&client->dev,
				 "device in use, ctrl(id:0x%x,val:0x%x) is not handled\n",
				 ctrl->id, ctrl->val);
		return 0;
	}

	if (poncha110->skip_reg_upload == 0)
	{
		switch (ctrl->id)
		{
		case V4L2_CID_ANALOGUE_GAIN:
			ret = poncha110_write_analog_gain_reg(poncha110, ctrl->val);

			printk(KERN_INFO "[PONCHA110]: exposure line = %u, exposure us = %u.\n", ctrl->val, ctrl->val);
			break;
		case V4L2_CID_EXPOSURE:
			printk(KERN_INFO "[PONCHA110]: exposure line = %u, exposure us = %u.\n", ctrl->val, ctrl->val);
			ret = poncha110_write_exposure_reg(poncha110, ctrl->val);
			break;
		case V4L2_CID_TEST_PATTERN:
			// Fixed data is hard coded to 0xAB.
			ret = poncha110_write(poncha110, PONCHA110_TRAINING_WORD_REG, 0xAB);
			// Gradient is hard coded to 45 degree.
			ret = poncha110_write(poncha110, PONCHA110_DELTA_TEST_IMG_REG, 0x01);
			ret = poncha110_write(poncha110, PONCHA110_TEST_PATTERN_REG,
								poncha110_test_pattern_val[ctrl->val]);
			break;
		case V4L2_CID_HFLIP:
			// TODO: HFLIP requires multiple register writes
			// ret = poncha110_write(poncha110, PONCHA110_HFLIP_REG,
			//		        ctrl->val);
			break;
		case V4L2_CID_VFLIP:
			// TODO: VFLIP seems not supported in Poncha110
			// ret = poncha110_write(poncha110, PONCHA110_VFLIP_REG,
			//		        ctrl->val);
			break;
		case V4L2_CID_VBLANK:
			/*
			 * In libcamera, frame time (== 1/framerate) is controlled by VBLANK:
			 * TARGET_FRAME_TIME (us) = 1000000 * ((1/PIXEL_RATE)*(WIDTH+HBLANK)*(HEIGHT+VBLANK))
			 */
			poncha110->target_frame_time = poncha110->mode->height + ctrl->val;
			// // Debug print
			printk(KERN_INFO "[PONCHA110]: poncha110_write_target_frame_time_reg target_frame_time = %u.\n",
			 	   poncha110->target_frame_time);
			// printk(KERN_INFO "[PONCHA110]: width %d, hblank %d, vblank %d, height %d, ctrl->val %d.\n",
			// 	   poncha110->mode->width, poncha110->mode->hblank, poncha110->mode->min_vblank, poncha110->mode->height, ctrl->val);
			ret = poncha110_write_target_frame_time_reg(poncha110, poncha110->target_frame_time);
			break;
		case V4L2_CID_HBLANK:
			printk(KERN_INFO "[PONCHA110]: V4L2_CID_HBLANK CALLED = %d.\n",
				ctrl->val);
			break;
		default:
			dev_info(&client->dev,
					 "ctrl(id:0x%x,val:0x%x) is not handled\n",
					 ctrl->id, ctrl->val);
			ret = -EINVAL;
			break;
		}
	}

	pm_runtime_put(&client->dev);

	// TODO: FIXIT
	return ret;
}

static int poncha110_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct poncha110 *poncha110 =
		container_of(ctrl->handler, struct poncha110, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&poncha110->sd);
	int ret = 0;

	// printk(KERN_INFO "[PONCHA110]: poncha110_s_ctrl() id: %X value: %X.\n", ctrl->id, ctrl->val);

	/* Previously, register writes when powered off will be buffered.
	 * The buffer will be written to sensor when start_streaming.
	 * Now, register writes happens immediately, even powered off.
	 * Register writes when powered off will fail.
	 * Users need to make sure first power on then write register.
	 */

	switch (ctrl->id)
	{
	case AMS_CAMERA_CID_MIRA_REG_W:
		ret = poncha110_v4l2_reg_w(poncha110, ctrl->val);
		break;
	default:
		dev_info(&client->dev,
				 "set ctrl(id:0x%x,val:0x%x) is not handled\n",
				 ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	// TODO: FIXIT
	return ret;
}

static int poncha110_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct poncha110 *poncha110 =
		container_of(ctrl->handler, struct poncha110, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&poncha110->sd);
	int ret = 0;

	// printk(KERN_INFO "[PONCHA110]: poncha110_g_ctrl() id: %X.\n", ctrl->id);

	/*
	 * Ideally, V4L2 register read should happen only when powered on.
	 * However, perhaps there are use cases that,
	 * reading other I2C addr is desired when mira sensor is powered off.
	 * Therefore, the check of "powered" flag is disabled for now.
	 */

	switch (ctrl->id)
	{
	case AMS_CAMERA_CID_MIRA_REG_R:
		ret = poncha110_v4l2_reg_r(poncha110, (u32 *)&ctrl->cur.val);
		ctrl->val = ctrl->cur.val;
		break;
	default:
		dev_info(&client->dev,
				 "get ctrl(id:0x%x) is not handled\n",
				 ctrl->id);
		ret = -EINVAL;
		break;
	}

	// TODO: FIXIT
	return ret;
}

static const struct v4l2_ctrl_ops poncha110_ctrl_ops = {
	.s_ctrl = poncha110_set_ctrl,
};

static const struct v4l2_ctrl_ops poncha110_custom_ctrl_ops = {
	.g_volatile_ctrl = poncha110_g_ctrl,
	.s_ctrl = poncha110_s_ctrl,
};

/* list of custom v4l2 ctls */
static struct v4l2_ctrl_config custom_ctrl_config_list[] = {
	/* Do not change the name field for the controls! */
	{
		.ops = &poncha110_custom_ctrl_ops,
		.id = AMS_CAMERA_CID_MIRA_REG_W,
		.name = "mira_reg_w",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = 0,
		.min = 0,
		.max = 0x7FFFFFFF,
		.def = 0,
		.step = 1,
	},
	{
		.ops = &poncha110_custom_ctrl_ops,
		.id = AMS_CAMERA_CID_MIRA_REG_R,
		.name = "mira_reg_r",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = 0,
		.min = 0,
		.max = 0x7FFFFFFF,
		.def = 0,
		.step = 1,
	},

};

// This function should enumerate all the media bus formats for the requested pads. If the requested
// format index is beyond the number of avaialble formats it shall return -EINVAL;
static int poncha110_enum_mbus_code(struct v4l2_subdev *sd,
								  struct v4l2_subdev_state *sd_state,
								  struct v4l2_subdev_mbus_code_enum *code)
{
	struct poncha110 *poncha110 = to_poncha110(sd);

	if (code->pad >= NUM_PADS)
		return -EINVAL;

	if (code->pad == IMAGE_PAD)
	{
		if (code->index >= ARRAY_SIZE(codes))
			return -EINVAL;

		code->code = poncha110_validate_format_code_or_default(poncha110,
															 codes[code->index]);
	}
	else
	{
		if (code->index > 0)
			return -EINVAL;

		code->code = MEDIA_BUS_FMT_SENSOR_DATA;
	}

	return 0;
}

static int poncha110_enum_frame_size(struct v4l2_subdev *sd,
								   struct v4l2_subdev_state *sd_state,
								   struct v4l2_subdev_frame_size_enum *fse)
{
	struct poncha110 *poncha110 = to_poncha110(sd);

	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	if (fse->pad == IMAGE_PAD)
	{
		/* Two options about how many modes to be exposed:
		 * - Expose all supported_modes by ARRAY_SIZE(supported_modes).
		 * - Expose less modes by PONCHA110_SUPPORTED_MODE_SIZE_PUBLIC.
		 */
		/* if (fse->index >= ARRAY_SIZE(supported_modes)) */
		if (fse->index >= PONCHA110_SUPPORTED_MODE_SIZE_PUBLIC)
			return -EINVAL;

		if (fse->code != poncha110_validate_format_code_or_default(poncha110, fse->code))
			return -EINVAL;

		fse->min_width = supported_modes[fse->index].width;
		fse->max_width = fse->min_width;
		fse->min_height = supported_modes[fse->index].height;
		fse->max_height = fse->min_height;
	}
	else
	{
		if (fse->code != MEDIA_BUS_FMT_SENSOR_DATA || fse->index > 0)
			return -EINVAL;

		fse->min_width = PONCHA110_EMBEDDED_LINE_WIDTH;
		fse->max_width = fse->min_width;
		fse->min_height = PONCHA110_NUM_EMBEDDED_LINES;
		fse->max_height = fse->min_height;
	}

	return 0;
}

static void poncha110_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
													  fmt->colorspace,
													  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void poncha110_update_image_pad_format(struct poncha110 *poncha110,
											const struct poncha110_mode *mode,
											struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	poncha110_reset_colorspace(&fmt->format);
}

static void poncha110_update_metadata_pad_format(struct v4l2_subdev_format *fmt)
{
	fmt->format.width = PONCHA110_EMBEDDED_LINE_WIDTH;
	fmt->format.height = PONCHA110_NUM_EMBEDDED_LINES;
	fmt->format.code = MEDIA_BUS_FMT_SENSOR_DATA;
	fmt->format.field = V4L2_FIELD_NONE;
}

static int __poncha110_get_pad_format(struct poncha110 *poncha110,
									struct v4l2_subdev_state *sd_state,
									struct v4l2_subdev_format *fmt)
{
	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
	{
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(&poncha110->sd, sd_state, fmt->pad);

		try_fmt->code = fmt->pad == IMAGE_PAD ? poncha110_validate_format_code_or_default(poncha110, try_fmt->code) : MEDIA_BUS_FMT_SENSOR_DATA;
		fmt->format = *try_fmt;
	}
	else
	{
		if (fmt->pad == IMAGE_PAD)
		{
			poncha110_update_image_pad_format(poncha110, poncha110->mode,
											fmt);
			fmt->format.code = poncha110_validate_format_code_or_default(poncha110,
																	   poncha110->fmt.code);
		}
		else
		{
			poncha110_update_metadata_pad_format(fmt);
		}
	}

	return 0;
}

static int poncha110_get_pad_format(struct v4l2_subdev *sd,
								  struct v4l2_subdev_state *sd_state,
								  struct v4l2_subdev_format *fmt)
{
	struct poncha110 *poncha110 = to_poncha110(sd);
	int ret;
	printk(KERN_INFO "[PONCHA110]: poncha110_get_pad_format\n");

	mutex_lock(&poncha110->mutex);
	ret = __poncha110_get_pad_format(poncha110, sd_state, fmt);
	mutex_unlock(&poncha110->mutex);

	return ret;
}

static int poncha110_set_pad_format(struct v4l2_subdev *sd,
								  struct v4l2_subdev_state *sd_state,
								  struct v4l2_subdev_format *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct poncha110 *poncha110 = to_poncha110(sd);
	const struct poncha110_mode *mode;
	struct v4l2_mbus_framefmt *framefmt;
	u32 max_exposure = 0;
	int rc = 0;
	
	printk(KERN_INFO "[PONCHA110]: poncha110_set_pad_format() .\n");

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&poncha110->mutex);

	if (fmt->pad == IMAGE_PAD)
	{
		printk(KERN_INFO "[PONCHA110]: fmt format code = %d.   \n", fmt->format.code);
		printk(KERN_INFO "[PONCHA110]: some code is  = %d.   \n", MEDIA_BUS_FMT_SBGGR10_1X10);

		/* Validate format or use default */
		fmt->format.code = poncha110_validate_format_code_or_default(poncha110,
																   fmt->format.code);

		// switch (fmt->format.code)
		// {
		// case MEDIA_BUS_FMT_SBGGR10_1X10:
		// 	printk(KERN_INFO "[PONCHA110]: fmt->format.code() selects 10 bit mode.\n");
		// 	poncha110->mode = &supported_modes[0];
		// 	poncha110->bit_depth = 10;
		// 	// return 0;
		// 	break;


		// default:
		// 	printk(KERN_ERR "Unknown format requested fmt->format.code() %d", fmt->format.code);
		// }

		mode = v4l2_find_nearest_size(supported_modes,
									  ARRAY_SIZE(supported_modes),
									  width, height,
									  fmt->format.width,
									  fmt->format.height);
		poncha110_update_image_pad_format(poncha110, mode, fmt);
		printk(KERN_INFO "[PONCHA110]: Poncha110 height mode 0 = %d.   mode 1 is %d \n", supported_modes[0].height, supported_modes[1].height);

		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		{
			printk(KERN_INFO "[PONCHA110]:   = v4l2_subdev_get_try_format.  \n");
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
												  fmt->pad);
			*framefmt = fmt->format;
		}
		else if (poncha110->mode != mode ||
				 poncha110->fmt.code != fmt->format.code)
		{
			printk(KERN_INFO "[PONCHA110]: Poncha110 bitdepth  = %d.   \n", poncha110->mode->bit_depth);

			printk(KERN_INFO "[PONCHA110]: Poncha110 mode  = %d.   mode is %d \n", poncha110->mode->code, mode->code);
			printk(KERN_INFO "[PONCHA110]: Poncha110 fmt  = %d.   fmt is %d \n", poncha110->fmt.code, fmt->format.code);
			printk(KERN_INFO "[PONCHA110]: Poncha110 width  = %d.   height is %d \n", poncha110->mode->width, poncha110->mode->height);

			poncha110->fmt = fmt->format;
			poncha110->mode = mode;

			// Update controls based on new mode (range and current value).
			// max_exposure = poncha110_calculate_max_exposure_time(PONCHA110_MIN_ROW_LENGTH,
			// 												   poncha110->mode->height,
			// 												   poncha110->mode->min_vblank);
			// default_exp = PONCHA110_DEFAULT_EXPOSURE > max_exposure ? max_exposure : PONCHA110_DEFAULT_EXPOSURE;
			// rc = __v4l2_ctrl_modify_range(poncha110->exposure,
			// 							  poncha110->exposure->minimum,
			// 							  (int)(1 + max_exposure), poncha110->exposure->step,
			// 							  (int)(1 + default_exp));
			// if (rc)
			// {
			// 	dev_err(&client->dev, "Error setting exposure range");
			// }

			// printk(KERN_INFO "[PONCHA110]: Poncha110 VBLANK  = %u.\n",
			// 	   poncha110->mode->min_vblank);

			// #FIXME #TODO
			//  rc = __v4l2_ctrl_modify_range(mira050->gain,
			//  					 0, ARRAY_SIZE(fine_gain_lut_8bit_16x) - 1, 1, 0);
			rc = __v4l2_ctrl_modify_range(poncha110->gain,
										  poncha110->mode->gain_min,
										  poncha110->mode->gain_max,
										  1,
										  0);
			if (rc)
			{
				dev_err(&client->dev, "Error setting gain range");
			}

			rc = __v4l2_ctrl_modify_range(poncha110->vblank,
										  poncha110->mode->min_vblank,
										  poncha110->mode->max_vblank,
										  1,
										  PONCHA110_DEFAULT_VBLANK_30);
			if (rc)
			{
				dev_err(&client->dev, "Error setting exposure range");
			}


			rc = __v4l2_ctrl_modify_range(poncha110->hblank,
										  poncha110->mode->hblank,
										  poncha110->mode->hblank,
										  1,
										  poncha110->mode->hblank);
			if (rc)
			{
				dev_err(&client->dev, "Error setting hblank range");
			}

			// Set the current vblank value
			rc = __v4l2_ctrl_s_ctrl(poncha110->vblank, PONCHA110_DEFAULT_VBLANK_30);
			if (rc)
			{
				dev_err(&client->dev, "Error setting vblank value to %u",
						poncha110->mode->min_vblank);
			}

		}
	}
	else
	{
		printk(KERN_INFO "[PONCHA110]: ERROR4 in  poncha110_set_pad_format() .\n");

		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		{
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
												  fmt->pad);
			*framefmt = fmt->format;
		}
		else
		{
			/* Only one embedded data mode is supported */
			poncha110_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&poncha110->mutex);

	return 0;
}

static int poncha110_set_framefmt(struct poncha110 *poncha110)
{
	// TODO: There is no easy way to change frame format
	switch (poncha110->fmt.code)
	{
	case MEDIA_BUS_FMT_SBGGR10_1X10:
		printk(KERN_INFO "[PONCHA110]: poncha110_set_framefmt() selects 10 bit mode.\n");
		// poncha110->mode = &supported_modes[0];
		poncha110->bit_depth = 10;
		return 0;

	default:
		printk(KERN_ERR "Unknown format requested %d", poncha110->fmt.code);
	}

	return -EINVAL;
}

static const struct v4l2_rect *
__poncha110_get_pad_crop(struct poncha110 *poncha110, struct v4l2_subdev_state *sd_state,
					   unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which)
	{
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&poncha110->sd, sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &poncha110->mode->crop;
	}

	return NULL;
}

static int poncha110_get_selection(struct v4l2_subdev *sd,
								 struct v4l2_subdev_state *sd_state,
								 struct v4l2_subdev_selection *sel)
{
	switch (sel->target)
	{
	case V4L2_SEL_TGT_CROP:
	{
		struct poncha110 *poncha110 = to_poncha110(sd);

		mutex_lock(&poncha110->mutex);
		sel->r = *__poncha110_get_pad_crop(poncha110, sd_state, sel->pad,
										 sel->which);
		mutex_unlock(&poncha110->mutex);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = PONCHA110_NATIVE_WIDTH;
		sel->r.height = PONCHA110_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = PONCHA110_PIXEL_ARRAY_TOP;
		sel->r.left = PONCHA110_PIXEL_ARRAY_LEFT;
		sel->r.width = PONCHA110_PIXEL_ARRAY_WIDTH;
		sel->r.height = PONCHA110_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

static int poncha110_start_streaming(struct poncha110 *poncha110)
{
	struct i2c_client *client = v4l2_get_subdevdata(&poncha110->sd);
	const struct poncha110_reg_list *reg_list;
	u32 otp_cal_val;
	int ret;

	printk(KERN_INFO "[PONCHA110]: Entering start streaming function.\n");

	/* Follow examples of other camera driver, here use pm_runtime_resume_and_get */
	ret = pm_runtime_resume_and_get(&client->dev);

	if (ret < 0)
	{
		printk(KERN_INFO "[PONCHA110]: get_sync failed, but continue.\n");
		pm_runtime_put_noidle(&client->dev);
		return ret;
	}

	/* Set current mode according to frame format bit depth */
	ret = poncha110_set_framefmt(poncha110);
	if (ret)
	{
		dev_err(&client->dev, "%s failed to set frame format: %d\n",
				__func__, ret);
		goto err_rpm_put;
	}
	printk(KERN_INFO "[PONCHA110]: Register sequence for %d bit mode will be used.\n", poncha110->mode->bit_depth);
	usleep_range(30000, 50000);

	if (poncha110->skip_reg_upload == 0)
	{
		/* Apply pre soft reset default values of current mode */
		reg_list = &poncha110->mode->reg_list_pre_soft_reset;
		printk(KERN_INFO "[PONCHA110]: Write %d regs.\n", reg_list->num_of_regs);
		ret = poncha110_write_regs(poncha110, reg_list->regs, reg_list->num_of_regs);
		if (ret)
		{
			dev_err(&client->dev, "%s failed to set mode\n", __func__);
			goto err_rpm_put;
		}


	}
	else
	{
		printk(KERN_INFO "[PONCHA110]: Skip base register sequence upload, due to poncha110->skip_reg_upload=%u.\n", poncha110->skip_reg_upload);
	}

	printk(KERN_INFO "[PONCHA110]: Entering v4l2 ctrl handler setup function.\n");

	/* Apply customized values from user */
	ret = __v4l2_ctrl_handler_setup(poncha110->sd.ctrl_handler);
	printk(KERN_INFO "[PONCHA110]: __v4l2_ctrl_handler_setup ret = %d.\n", ret);
	if (ret)
		goto err_rpm_put;

	/* Read OTP memory for OTP_CALIBRATION_VALUE */
	// ret = poncha110_otp_read(poncha110, 0x01, &otp_cal_val);
	/* OTP_CALIBRATION_VALUE is little-endian, LSB at [7:0], MSB at [15:8] */
	// poncha110->otp_cal_val = (u16)(otp_cal_val & 0x0000FFFF);
	// if (ret)
	// {
	// 	dev_err(&client->dev, "%s failed to read OTP addr 0x01.\n", __func__);
	// 	/* Even if OTP reading fails, continue with the rest. */
	// 	poncha110->otp_cal_val = PONCHA110_OTP_CAL_VALUE_DEFAULT;
	// 	printk(KERN_INFO "[PONCHA110]: Due to OTP reading failure, use default poncha110->otp_cal_val : %u.\n", poncha110->otp_cal_val);
	// 	/* goto err_rpm_put; */
	// }
	// else
	// {
	// 	printk(KERN_INFO "[PONCHA110]: OTP_CALIBRATION_VALUE: %u, extracted from 32-bit 0x%X.\n", poncha110->otp_cal_val, otp_cal_val);
	// 	if ((otp_cal_val & 0xFFFF0000) != 0xFFFF0000)
	// 	{
	// 		poncha110->otp_cal_val = PONCHA110_OTP_CAL_VALUE_DEFAULT;
	// 		printk(KERN_INFO "[PONCHA110]: Due to higher 16-bit not all 1, use default poncha110->otp_cal_val : %u.\n", poncha110->otp_cal_val);
	// 	}
	// 	else if (poncha110->otp_cal_val < PONCHA110_OTP_CAL_VALUE_MIN)
	// 	{
	// 		poncha110->otp_cal_val = PONCHA110_OTP_CAL_VALUE_DEFAULT;
	// 		printk(KERN_INFO "[PONCHA110]: Due to extracted value < %u, likely an error, use default poncha110->otp_cal_val : %u.\n", PONCHA110_OTP_CAL_VALUE_MIN, poncha110->otp_cal_val);
	// 	}
	// 	else if (poncha110->otp_cal_val > PONCHA110_OTP_CAL_VALUE_MAX)
	// 	{
	// 		poncha110->otp_cal_val = PONCHA110_OTP_CAL_VALUE_DEFAULT;
	// 		printk(KERN_INFO "[PONCHA110]: Due to extracted value > %u, likely an error, use default poncha110->otp_cal_val : %u.\n", PONCHA110_OTP_CAL_VALUE_MAX, poncha110->otp_cal_val);
	// 	}
	// }

	if (poncha110->skip_reg_upload == 0 ||
		(poncha110->skip_reg_upload == 1 && poncha110->force_stream_ctrl == 1))
	{
		printk(KERN_INFO "[PONCHA110]: Writing start streaming regs.\n");
		ret = poncha110_write_start_streaming_regs(poncha110);
		if (ret)
		{
			dev_err(&client->dev, "Could not write stream-on sequence");
			goto err_rpm_put;
		}
	}
	else
	{
		printk(KERN_INFO "[PONCHA110]: Skip write_start_streaming_regs due to skip_reg_upload == %d and force_stream_ctrl == %d.\n",
			   poncha110->skip_reg_upload, poncha110->force_stream_ctrl);
	}

	/* vflip and hflip cannot change during streaming */
	printk(KERN_INFO "[PONCHA110]: Entering v4l2 ctrl grab vflip grab vflip.\n");
	__v4l2_ctrl_grab(poncha110->vflip, true);
	printk(KERN_INFO "[PONCHA110]: Entering v4l2 ctrl grab vflip grab hflip.\n");
	__v4l2_ctrl_grab(poncha110->hflip, true);

	// poncha110_write_illum_trig_regs(poncha110);

	return 0;

err_rpm_put:
	pm_runtime_put(&client->dev);
	return ret;
}

static void poncha110_stop_streaming(struct poncha110 *poncha110)
{
	struct i2c_client *client = v4l2_get_subdevdata(&poncha110->sd);
	int ret = 0;

	/* Unlock controls for vflip and hflip */
	__v4l2_ctrl_grab(poncha110->vflip, false);
	__v4l2_ctrl_grab(poncha110->hflip, false);

	if (poncha110->skip_reset == 0)
	{
		if (poncha110->skip_reg_upload == 0 ||
			(poncha110->skip_reg_upload == 1 && poncha110->force_stream_ctrl == 1))
		{
			printk(KERN_INFO "[PONCHA110]: Writing stop streaming regs.\n");
			ret = poncha110_write_stop_streaming_regs(poncha110);
			if (ret)
			{
				dev_err(&client->dev, "Could not write the stream-off sequence");
			}
		}
		else
		{
			printk(KERN_INFO "[PONCHA110]: Skip write_stop_streaming_regs due to skip_reg_upload == %d and force_stream_ctrl == %d.\n",
				   poncha110->skip_reg_upload, poncha110->force_stream_ctrl);
		}
	}
	else
	{
		printk(KERN_INFO "[PONCHA110]: Skip write_stop_streaming_regs due to poncha110->skip_reset == %d.\n", poncha110->skip_reset);
	}

	pm_runtime_put(&client->dev);
}

static int poncha110_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct poncha110 *poncha110 = to_poncha110(sd);
	int ret = 0;

	mutex_lock(&poncha110->mutex);
	if (poncha110->streaming == enable)
	{
		mutex_unlock(&poncha110->mutex);
		return 0;
	}

	printk(KERN_INFO "[PONCHA110]: Entering poncha110_set_stream enable: %d.\n", enable);

	if (enable)
	{
		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = poncha110_start_streaming(poncha110);
		if (ret)
			goto err_unlock;
	}
	else
	{
		poncha110_stop_streaming(poncha110);
	}

	poncha110->streaming = enable;

	mutex_unlock(&poncha110->mutex);

	printk(KERN_INFO "[PONCHA110]: Returning poncha110_set_stream with ret: %d.\n", ret);

	return ret;

err_unlock:
	mutex_unlock(&poncha110->mutex);

	return ret;
}

static int __maybe_unused poncha110_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct poncha110 *poncha110 = to_poncha110(sd);

	printk(KERN_INFO "[PONCHA110]: Entering suspend function.\n");

	if (poncha110->streaming)
		poncha110_stop_streaming(poncha110);

	return 0;
}

static int __maybe_unused poncha110_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct poncha110 *poncha110 = to_poncha110(sd);
	int ret;

	printk(KERN_INFO "[PONCHA110]: Entering resume function.\n");

	if (poncha110->streaming)
	{
		ret = poncha110_start_streaming(poncha110);
		if (ret)
			goto error;
	}

	return 0;

error:
	poncha110_stop_streaming(poncha110);
	poncha110->streaming = false;

	return ret;
}

static int poncha110_get_regulators(struct poncha110 *poncha110)
{
	struct i2c_client *client = v4l2_get_subdevdata(&poncha110->sd);
	unsigned int i;

	for (i = 0; i < PONCHA110_NUM_SUPPLIES; i++)
		poncha110->supplies[i].supply = poncha110_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
								   PONCHA110_NUM_SUPPLIES,
								   poncha110->supplies);
}

/* Verify chip ID */
static int poncha110_identify_module(struct poncha110 *poncha110)
{
	int ret;
	u8 val;

	ret = poncha110_read(poncha110, 0x25, &val);
	printk(KERN_INFO "[PONCHA110]: Read reg 0x%4.4x, val = 0x%x.\n",
		   0x25, val);
	ret = poncha110_read(poncha110, 0x3, &val);
	printk(KERN_INFO "[PONCHA110]: Read reg 0x%4.4x, val = 0x%x.\n",
		   0x3, val);
	ret = poncha110_read(poncha110, 0x4, &val);
	printk(KERN_INFO "[PONCHA110]: Read reg 0x%4.4x, val = 0x%x.\n",
		   0x4, val);

	return 0;
}

static const struct v4l2_subdev_core_ops poncha110_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops poncha110_video_ops = {
	.s_stream = poncha110_set_stream,
};

static const struct v4l2_subdev_pad_ops poncha110_pad_ops = {
	.enum_mbus_code = poncha110_enum_mbus_code,
	.get_fmt = poncha110_get_pad_format,
	.set_fmt = poncha110_set_pad_format,
	.get_selection = poncha110_get_selection,
	.enum_frame_size = poncha110_enum_frame_size,
};

static const struct v4l2_subdev_ops poncha110_subdev_ops = {
	.core = &poncha110_core_ops,
	.video = &poncha110_video_ops,
	.pad = &poncha110_pad_ops,
};

static const struct v4l2_subdev_internal_ops poncha110_internal_ops = {
	.open = poncha110_open,
};

/* Initialize control handlers */
static int poncha110_init_controls(struct poncha110 *poncha110)
{
	struct i2c_client *client = v4l2_get_subdevdata(&poncha110->sd);
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct v4l2_fwnode_device_properties props;
	int ret;
	struct v4l2_ctrl_config *mira_reg_w;
	struct v4l2_ctrl_config *mira_reg_r;

	ctrl_hdlr = &poncha110->ctrl_handler;
	/* v4l2_ctrl_handler_init gives a hint/guess of the number of v4l2_ctrl_new */
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 16);
	if (ret)
		return ret;

	mutex_init(&poncha110->mutex);
	ctrl_hdlr->lock = &poncha110->mutex;

	printk(KERN_INFO "[PONCHA110]: %s V4L2_CID_PIXEL_RATE %X.\n", __func__, V4L2_CID_PIXEL_RATE);

	/* By default, PIXEL_RATE is read only */
	poncha110->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &poncha110_ctrl_ops,
											V4L2_CID_PIXEL_RATE,
											PONCHA110_PIXEL_RATE,
											PONCHA110_PIXEL_RATE, 1,
											PONCHA110_PIXEL_RATE);

	printk(KERN_INFO "[PONCHA110]: %s V4L2_CID_VBLANK %X.\n", __func__, V4L2_CID_VBLANK);

	poncha110->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &poncha110_ctrl_ops,
										V4L2_CID_VBLANK, poncha110->mode->min_vblank,
										poncha110->mode->max_vblank, 1,
										PONCHA110_DEFAULT_VBLANK_30);

	printk(KERN_INFO "[PONCHA110]: %s V4L2_CID_HBLANK %X.\n", __func__, V4L2_CID_HBLANK);

	poncha110->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &poncha110_ctrl_ops,
										V4L2_CID_HBLANK, poncha110->mode->hblank,
										poncha110->mode->hblank, 1,
										poncha110->mode->hblank);

	// Make the vblank control read only. This could be changed to allow changing framerate in
	// runtime, but would require adapting other settings
	poncha110->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	// Exposure is indicated in number of lines here
	// Max is determined by vblank + vsize and Tglob.
	printk(KERN_INFO "[PONCHA110]: %s V4L2_CID_EXPOSURE %X.\n", __func__, V4L2_CID_EXPOSURE);

	poncha110->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &poncha110_ctrl_ops,
										  V4L2_CID_EXPOSURE,
										  PONCHA110_EXPOSURE_MIN, PONCHA110_EXPOSURE_MAX,
										  1,
										  PONCHA110_DEFAULT_EXPOSURE);

	printk(KERN_INFO "[PONCHA110]: %s V4L2_CID_ANALOGUE_GAIN %X.\n", __func__, V4L2_CID_ANALOGUE_GAIN);

	poncha110->gain = v4l2_ctrl_new_std(ctrl_hdlr, &poncha110_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
									  PONCHA110_ANALOG_GAIN_MIN, PONCHA110_ANALOG_GAIN_MAX,
									  PONCHA110_ANALOG_GAIN_STEP, PONCHA110_ANALOG_GAIN_DEFAULT);

	printk(KERN_INFO "[PONCHA110]: %s V4L2_CID_HFLIP %X.\n", __func__, V4L2_CID_HFLIP);

	poncha110->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &poncha110_ctrl_ops,
									   V4L2_CID_HFLIP, 0, 0, 1, 0);
	if (poncha110->hflip)
		poncha110->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	printk(KERN_INFO "[PONCHA110]: %s V4L2_CID_VFLIP %X.\n", __func__, V4L2_CID_VFLIP);

	poncha110->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &poncha110_ctrl_ops,
									   V4L2_CID_VFLIP, 0, 0, 1, 0);
	if (poncha110->vflip)
		poncha110->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	printk(KERN_INFO "[PONCHA110]: %s V4L2_CID_TEST_PATTERN %X.\n", __func__, V4L2_CID_TEST_PATTERN);
	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &poncha110_ctrl_ops,
								 V4L2_CID_TEST_PATTERN,
								 ARRAY_SIZE(poncha110_test_pattern_menu) - 1,
								 0, 0, poncha110_test_pattern_menu);
	/*
	 * Custom op
	 */
	mira_reg_w = &custom_ctrl_config_list[0];
	printk(KERN_INFO "[PONCHA110]: %s AMS_CAMERA_CID_MIRA_REG_W %X.\n", __func__, AMS_CAMERA_CID_MIRA_REG_W);
	poncha110->mira_reg_w = v4l2_ctrl_new_custom(ctrl_hdlr, mira_reg_w, NULL);

	mira_reg_r = &custom_ctrl_config_list[1];
	printk(KERN_INFO "[PONCHA110]: %s AMS_CAMERA_CID_MIRA_REG_R %X.\n", __func__, AMS_CAMERA_CID_MIRA_REG_R);
	poncha110->mira_reg_r = v4l2_ctrl_new_custom(ctrl_hdlr, mira_reg_r, NULL);
	if (poncha110->mira_reg_r)
		poncha110->mira_reg_r->flags |= (V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY);

	if (ctrl_hdlr->error)
	{
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
				__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &poncha110_ctrl_ops,
										  &props);
	if (ret)
		goto error;

	poncha110->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&poncha110->mutex);

	return ret;
}

static void poncha110_free_controls(struct poncha110 *poncha110)
{
	v4l2_ctrl_handler_free(poncha110->sd.ctrl_handler);
	mutex_destroy(&poncha110->mutex);
}

static int poncha110_check_hwcfg(struct device *dev)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY};
	int ret = -EINVAL;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint)
	{
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg))
	{
		dev_err(dev, "could not parse endpoint\n");
		goto error_out;
	}

	/* Check the link frequency set in device tree */
	if (!ep_cfg.nr_of_link_frequencies)
	{
		dev_err(dev, "link-frequency property not found in DT\n");
		goto error_out;
	}

	if (ep_cfg.nr_of_link_frequencies != 1 ||
		ep_cfg.link_frequencies[0] != PONCHA110_DEFAULT_LINK_FREQ)
	{
		dev_err(dev, "Link frequency not supported: %lld\n",
				ep_cfg.link_frequencies[0]);
		goto error_out;
	}

	// TODO(jalv): Check device tree configuration and make sure it is supported by the driver
	ret = 0;

error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);

	return ret;
}

static int poncha110pmic_init_controls(struct i2c_client *pmic_client, struct i2c_client *uc_client)
{
	int ret;
	u8 val;

	// uC, set atb and jtag high
	// according to old uC fw (svn rev41)
	// 12[3] ldo en
	// 11[4,5] atpg jtag
	// 11/12 i/o direction, 15/16 output high/low
	// uC, set atb and jtag high
	// WARNING this only works on interposer v2 if R307 is not populated. otherwise, invert the bit for ldo
	ret = poncha110pmic_write(uc_client, 12, 0xF7);
	ret = poncha110pmic_write(uc_client, 16, 0xFF); // ldo en:1
	ret = poncha110pmic_write(uc_client, 11, 0XCF);
	ret = poncha110pmic_write(uc_client, 15, 0xFF);
	ret = poncha110pmic_write(uc_client, 6, 1); // write

	// Disable master switch //
	ret = poncha110pmic_write(pmic_client, 0x62, 0x00);

	// Set all voltages to 0

	// DCDC1=0V
	ret = poncha110pmic_write(pmic_client, 0x05, 0x00);
	// DCDC4=0V
	ret = poncha110pmic_write(pmic_client, 0x0E, 0x0);
	// LDO1=0V VDDLO_PLL
	ret = poncha110pmic_write(pmic_client, 0x11, 0x0);
	// LDO2=0.0V
	ret = poncha110pmic_write(pmic_client, 0x14, 0x00);
	// LDO3=0.0V
	ret = poncha110pmic_write(pmic_client, 0x17, 0x00);
	// LDO4=0V
	ret = poncha110pmic_write(pmic_client, 0x1A, 0x00);
	// LDO5=0.0V
	ret = poncha110pmic_write(pmic_client, 0x1C, 0x00);
	// LDO6=0.0V
	ret = poncha110pmic_write(pmic_client, 0x1D, 0x00);
	// LDO7=0V
	ret = poncha110pmic_write(pmic_client, 0x1E, 0x0);
	// LDO8=0.0V
	ret = poncha110pmic_write(pmic_client, 0x1F, 0x00);
	// Disable LDO9 Lock
	ret = poncha110pmic_write(pmic_client, 0x24, 0x48);
	// LDO9=0V VDDHI
	ret = poncha110pmic_write(pmic_client, 0x20, 0x00);
	// LDO10=0V VDDLO_ANA
	ret = poncha110pmic_write(pmic_client, 0x21, 0x0);

	// Enable master switch //
	usleep_range(50, 60);
	ret = poncha110pmic_write(pmic_client, 0x62, 0x0D); // enable master switch
	usleep_range(50, 60);

	// start PMIC
	// Keep LDOs always on
	ret = poncha110pmic_write(pmic_client, 0x27, 0xFF);
	ret = poncha110pmic_write(pmic_client, 0x28, 0xFF);
	ret = poncha110pmic_write(pmic_client, 0x29, 0x00);
	ret = poncha110pmic_write(pmic_client, 0x2A, 0x00);
	ret = poncha110pmic_write(pmic_client, 0x2B, 0x00);

	// Unused LDO off //
	usleep_range(50, 60);
	// set GPIO1=0
	ret = poncha110pmic_write(pmic_client, 0x41, 0x04);
	// DCDC2=0.0V SPARE_PWR1
	ret = poncha110pmic_write(pmic_client, 0x01, 0x00);
	ret = poncha110pmic_write(pmic_client, 0x08, 0x00);
	// DCDC3=0V SPARE_PWR1
	ret = poncha110pmic_write(pmic_client, 0x02, 0x00);
	ret = poncha110pmic_write(pmic_client, 0x0B, 0x00);
	// LDO2=0.0V
	ret = poncha110pmic_write(pmic_client, 0x14, 0x00);
	// LDO3=0.0V
	ret = poncha110pmic_write(pmic_client, 0x17, 0x00);
	// LDO5=0.0V
	ret = poncha110pmic_write(pmic_client, 0x1C, 0x00);
	// LDO6=0.0V
	ret = poncha110pmic_write(pmic_client, 0x1D, 0x00);
	// LDO8=0.0V
	ret = poncha110pmic_write(pmic_client, 0x1F, 0x00);

	ret = poncha110pmic_write(pmic_client, 0x42, 4);

	// Enable 1.80V //
	usleep_range(50, 60);
	// DCDC1=1.8V VINLDO1p8 >=1P8
	ret = poncha110pmic_write(pmic_client, 0x00, 0x00);
	ret = poncha110pmic_write(pmic_client, 0x04, 0x34);
	ret = poncha110pmic_write(pmic_client, 0x06, 0xBF);
	ret = poncha110pmic_write(pmic_client, 0x05, 0xB4);
	// DCDC4=1.8V VDDIO
	ret = poncha110pmic_write(pmic_client, 0x03, 0x00);
	ret = poncha110pmic_write(pmic_client, 0x0D, 0x34);
	ret = poncha110pmic_write(pmic_client, 0x0F, 0xBF);
	ret = poncha110pmic_write(pmic_client, 0x0E, 0xB4);

	// Enable 2.85V //
	usleep_range(50, 60);
	// LDO4=2.85V VDDHI alternativ
	ret = poncha110pmic_write(pmic_client, 0x1A, 0xB8); // Either 0x00 or 0xB8
	// Disable LDO9 Lock
	ret = poncha110pmic_write(pmic_client, 0x24, 0x48);
	// LDO9=2.85V VDDHI
	ret = poncha110pmic_read(pmic_client, 0x20, &val);
	dev_err(&pmic_client->dev, "Read 0x20 with val %x\n", val);
	ret = poncha110pmic_write(pmic_client, 0x20, 0xB9);
	ret = poncha110pmic_read(pmic_client, 0x20, &val);
	dev_err(&pmic_client->dev, "Read 0x20 with val %x\n", val);

	// VPIXH on cob = vdd25A on interposer = LDO4 on pmic
	// VPIXH should connect to VDD28 on pcb, or enable 4th supply
	ret = poncha110pmic_read(pmic_client, 0x19, &val);
	dev_err(&pmic_client->dev, "Read 0x19 with val %x\n", val);
	ret = poncha110pmic_write(pmic_client, 0x19, 0x38);
	ret = poncha110pmic_read(pmic_client, 0x19, &val);
	dev_err(&pmic_client->dev, "Read 0x19 with val %x\n", val);

	// Enable 1.2V //
	usleep_range(700, 710);
	// LDO1=1.2V VDDLO_PLL
	ret = poncha110pmic_write(pmic_client, 0x12, 0x16);
	ret = poncha110pmic_write(pmic_client, 0x10, 0x16);
	ret = poncha110pmic_write(pmic_client, 0x11, 0x90);
	// LDO7=1.2V VDDLO_DIG
	ret = poncha110pmic_write(pmic_client, 0x1E, 0x90);
	// LDO10=1.2V VDDLO_ANA
	ret = poncha110pmic_write(pmic_client, 0x21, 0x90);

	// Enable green LED //
	usleep_range(50, 60);
	ret = poncha110pmic_write(pmic_client, 0x42, 0x15); // gpio2
	// ret = poncha110pmic_write(pmic_client, 0x43, 0x40); // leda
	// ret = poncha110pmic_write(pmic_client, 0x44, 0x40); // ledb
	ret = poncha110pmic_write(pmic_client, 0x45, 0x40); // ledc

	// ret = poncha110pmic_write(pmic_client, 0x47, 0x02); // leda ctrl1
	// ret = poncha110pmic_write(pmic_client, 0x4F, 0x02); // ledb ctrl1
	ret = poncha110pmic_write(pmic_client, 0x57, 0x02); // ledc ctrl1

	// ret = poncha110pmic_write(pmic_client, 0x4D, 0x01); // leda ctrl1
	// ret = poncha110pmic_write(pmic_client, 0x55, 0x10); // ledb ctrl7
	ret = poncha110pmic_write(pmic_client, 0x5D, 0x10); // ledc ctrl7
	ret = poncha110pmic_write(pmic_client, 0x61, 0x10); // led seq -- use this to turn on leds. abc0000- 1110000 for all leds

	// uC, set atb and jtag high and ldo_en
	ret = poncha110pmic_write(uc_client, 12, 0xF7);
	ret = poncha110pmic_write(uc_client, 16, 0xF7); // ldo en:0
	/*
	 * In Poncha110-bringup.py, write 11, 0xCF; 15: 0x30.
	 * In poncha110.py, write 11, 0x8D; 15, 0xFD.
	 */
	ret = poncha110pmic_write(uc_client, 11, 0X8D);
	ret = poncha110pmic_write(uc_client, 15, 0xFD);
	ret = poncha110pmic_write(uc_client, 6, 1); // write

	usleep_range(2000000, 2001000);

	return 0;
}

static int poncha110_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct poncha110 *poncha110;
	int ret;

	printk(KERN_INFO "[PONCHA110]: probing v4l2 sensor.\n");
	printk(KERN_INFO "[PONCHA110]: Driver Version 0.0.\n");

	dev_err(dev, "[PONCHA110] name: %s.\n", client->name);

	poncha110 = devm_kzalloc(&client->dev, sizeof(*poncha110), GFP_KERNEL);
	if (!poncha110)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&poncha110->sd, client, &poncha110_subdev_ops);

	/* Check the hardware configuration in device tree */
	if (poncha110_check_hwcfg(dev))
		return -EINVAL;

	/* Parse device tree to check if dtoverlay has param skip-reg-upload=1 */
	device_property_read_u32(dev, "skip-reg-upload", &poncha110->skip_reg_upload);
	printk(KERN_INFO "[PONCHA110]: skip-reg-upload %d.\n", poncha110->skip_reg_upload);
	/* Set default TBD I2C device address to LED I2C Address*/
	poncha110->tbd_client_i2c_addr = PONCHA110LED_I2C_ADDR;
	printk(KERN_INFO "[PONCHA110]: User defined I2C device address defaults to LED driver I2C address 0x%X.\n", poncha110->tbd_client_i2c_addr);

	/* Get system clock (xclk) */
	poncha110->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(poncha110->xclk))
	{
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(poncha110->xclk);
	}

	poncha110->xclk_freq = clk_get_rate(poncha110->xclk);
	if (poncha110->xclk_freq != PONCHA110_SUPPORTED_XCLK_FREQ)
	{
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
				poncha110->xclk_freq);
		return -EINVAL;
	}

	ret = poncha110_get_regulators(poncha110);
	if (ret)
	{
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	{
		printk(KERN_INFO "[PONCHA110]: Init PMIC and uC and led driver.\n");
		poncha110->pmic_client = i2c_new_dummy_device(client->adapter,
													PONCHA110PMIC_I2C_ADDR);
		if (IS_ERR(poncha110->pmic_client))
			return PTR_ERR(poncha110->pmic_client);
		poncha110->uc_client = i2c_new_dummy_device(client->adapter,
												  PONCHA110UC_I2C_ADDR);
		if (IS_ERR(poncha110->uc_client))
			return PTR_ERR(poncha110->uc_client);
		poncha110->led_client = i2c_new_dummy_device(client->adapter,
												   PONCHA110LED_I2C_ADDR);
		if (IS_ERR(poncha110->led_client))
			return PTR_ERR(poncha110->led_client);

		// poncha110pmic_init_controls(poncha110->pmic_client, poncha110->uc_client);
	}

	dev_err(dev, "[PONCHA110] Sleep for 1 second to let PMIC driver complete init.\n");
	usleep_range(1000000, 1000000 + 100);
	// set some defaults

	/*
	 * The sensor must be powered for poncha110_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = poncha110_power_on(dev);
	if (ret)
		return ret;

	printk(KERN_INFO "[PONCHA110]: Entering identify function.\n");

	ret = poncha110_identify_module(poncha110);
	if (ret)
		goto error_power_off;

	printk(KERN_INFO "[PONCHA110]: Setting support function.\n");

	/* Initialize default illumination trigger parameters */
	/* ILLUM_WIDTH is in unit of SEQ_TIME_BASE, equal to (8/PONCHA110_DATA_RATE) us. */
	// poncha110->illum_width = PONCHA110_ILLUM_WIDTH_DEFAULT;
	// /* ILLUM_WIDTH AUTO will match illum to exposure pulse width*/
	// poncha110->illum_width_auto = PONCHA110_ILLUM_SYNC_DEFAULT;
	// /* ILLUM_ENABLE is True or False, enabling it will activate illum trig. */
	// poncha110->illum_enable = PONCHA110_ILLUM_ENABLE_DEFAULT;
	// /* ILLUM_DELAY is in unit of TIME_UNIT, equal to 1 us. In continuous stream mode, zero delay is 1<<19. */
	// poncha110->illum_delay = PONCHA110_ILLUM_DELAY_DEFAULT;
	/* Set default mode to max resolution */
	poncha110->mode = &supported_modes[0];

	printk(KERN_INFO "[PONCHA110]: Entering init controls function.\n");

	ret = poncha110_init_controls(poncha110);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	poncha110->sd.internal_ops = &poncha110_internal_ops;
	poncha110->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
						 V4L2_SUBDEV_FL_HAS_EVENTS;
	poncha110->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	poncha110->pad[IMAGE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	poncha110->pad[METADATA_PAD].flags = MEDIA_PAD_FL_SOURCE;

	printk(KERN_INFO "[PONCHA110]: Entering set default format function.\n");

	/* Initialize default format */
	poncha110_set_default_format(poncha110);

	printk(KERN_INFO "[PONCHA110]: Entering pads init function.\n");

	ret = media_entity_pads_init(&poncha110->sd.entity, NUM_PADS, poncha110->pad);
	if (ret)
	{
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	printk(KERN_INFO "[PONCHA110]: Entering subdev sensor common function.\n");

	ret = v4l2_async_register_subdev_sensor(&poncha110->sd);
	if (ret < 0)
	{
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	/* For debug purpose */
	// poncha110_start_streaming(poncha110);

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

error_media_entity:
	media_entity_cleanup(&poncha110->sd.entity);

error_handler_free:
	poncha110_free_controls(poncha110);

error_power_off:
	poncha110_power_off(dev);

	i2c_unregister_device(poncha110->pmic_client);
	i2c_unregister_device(poncha110->uc_client);
	i2c_unregister_device(poncha110->led_client);

	return ret;
}

static void poncha110_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct poncha110 *poncha110 = to_poncha110(sd);

	i2c_unregister_device(poncha110->pmic_client);
	i2c_unregister_device(poncha110->uc_client);
	i2c_unregister_device(poncha110->led_client);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	poncha110_free_controls(poncha110);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		poncha110_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static const struct dev_pm_ops poncha110_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(poncha110_suspend, poncha110_resume)
		SET_RUNTIME_PM_OPS(poncha110_power_off, poncha110_power_on, NULL)};

#endif // __PONCHA110_INL__
