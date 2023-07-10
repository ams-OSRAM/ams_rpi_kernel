// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for ams MIRA016 cameras.
 * Copyright (C) 2022, ams-OSRAM
 *
 * Based on Sony IMX219 camera driver
 * Copyright (C) 2019, Raspberry Pi (Trading) Ltd
 */

#ifndef __MIRA016_INL__
#define __MIRA016_INL__

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
 * Introduce new v4l2 control
 */
#include <linux/v4l2-controls.h>
#define AMS_CAMERA_CID_BASE		(V4L2_CTRL_CLASS_CAMERA | 0x2000)
#define AMS_CAMERA_CID_MIRA_REG_W	(AMS_CAMERA_CID_BASE+0)
#define AMS_CAMERA_CID_MIRA_REG_R	(AMS_CAMERA_CID_BASE+1)

/* Most significant Byte is flag, and most significant bit is unused. */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_FOR_READ    0b00000001
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_USE_BANK    0b00000010
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_BANK        0b00000100
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_CONTEXT     0b00001000
/* Use bit 5 to indicate special command, bit 2,3,4 for command. */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_CMD_SEL     0b00010000
/* Special command for sleep. The other 3 Bytes is sleep values in us. */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_SLEEP_US    0b00010000
/* Special command to enable power on (/off) when stream on (/off). */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_RESET_ON    0b00010010
/* Special command to disable power on (/off) when stream on (/off). */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_RESET_OFF   0b00010100
/* Special command to enable base register sequence upload, overwrite skip-reg-upload in dtoverlay */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_REG_UP_ON   0b00010110
/* Special command to disable base register sequence upload, overwrite skip-reg-upload in dtoverlay */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_REG_UP_OFF  0b00011000
/*
 * Bit 6&7 of flag are combined to specify I2C dev (default is Mira).
 * If bit 6&7 is 0b01, the reg_addr and reg_val are for a TBD I2C address.
 * The TBD I2C address is default to MIRA016LED_I2C_ADDR.
 * To change the TBD I2C address, set bit 6&7 to 0b10,
 * then the reg_val will become TBD I2C address.
 * The TBD I2C address is stored in mira016->tbd_client_i2c_addr.
 */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_SEL     0b01100000
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_MIRA    0b00000000
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_TBD     0b00100000
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_SET_TBD 0b01000000

/* Pre-allocated i2c_client */
#define MIRA016PMIC_I2C_ADDR 0x2D
#define MIRA016UC_I2C_ADDR 0x0A
#define MIRA016LED_I2C_ADDR 0x53

#define MIRA016_NATIVE_WIDTH			400U
#define MIRA016_NATIVE_HEIGHT			400U

#define MIRA016_PIXEL_ARRAY_LEFT		0U
#define MIRA016_PIXEL_ARRAY_TOP			0U
#define MIRA016_PIXEL_ARRAY_WIDTH		400U
#define MIRA016_PIXEL_ARRAY_HEIGHT		400U

/* Set analog gain min and max to 0 to avoid user changing it. */
#define MIRA016_ANALOG_GAIN_MAX			0
#define MIRA016_ANALOG_GAIN_MIN			0
#define MIRA016_ANALOG_GAIN_STEP		1
#define MIRA016_ANALOG_GAIN_DEFAULT		MIRA016_ANALOG_GAIN_MIN

#define MIRA016_BIT_DEPTH_REG			0x209E
#define MIRA016_BIT_DEPTH_12_BIT		0x02
#define MIRA016_BIT_DEPTH_10_BIT		0x04
#define MIRA016_BIT_DEPTH_8_BIT			0x06

#define MIRA016_CSI_DATA_TYPE_REG		0x208D
#define MIRA016_CSI_DATA_TYPE_12_BIT	0x04
#define MIRA016_CSI_DATA_TYPE_10_BIT	0x02
#define MIRA016_CSI_DATA_TYPE_8_BIT		0x01

#define MIRA016_BANK_SEL_REG			0xE000
#define MIRA016_RW_CONTEXT_REG			0xE004
#define MIRA016_CMD_REQ_1_REG			0x000A
#define MIRA016_CMD_HALT_BLOCK_REG		0x000C

#define MIRA016_NB_OF_FRAMES_LO_REG		0x10F2
#define MIRA016_NB_OF_FRAMES_HI_REG		0x10F3

#define MIRA016_POWER_MODE_REG			0x0043
#define MIRA016_POWER_MODE_SLEEP		0x01
#define MIRA016_POWER_MODE_IDLE			0x02
#define MIRA016_POWER_MODE_ACTIVE		0x0C

// Exposure time is indicated in us
#define MIRA016_EXP_TIME_L_REG			0x000E
#define MIRA016_EXP_TIME_S_REG			0x0012

// VBLANK is indicated in number of rows
#define MIRA016_VBLANK_LO_REG			0x1012
#define MIRA016_VBLANK_HI_REG			0x1013

#define MIRA016_EXT_EXP_PW_SEL_REG		0x1001
#define MIRA016_EXT_EXP_PW_SEL_USE_REG		1
#define MIRA016_EXT_EXP_PW_SEL_USE_EXT		0

// Exposure delay is indicated in number of rows
#define MIRA016_EXT_EXP_DELAY_LO_REG		0x10D0
#define MIRA016_EXT_EXP_DELAY_HI_REG		0x10D1

#define MIRA016_VSIZE1_LO_REG			0x1087
#define MIRA016_VSIZE1_HI_REG			0x1088
#define MIRA016_VSIZE1_MASK			0x7FF

#define MIRA016_VSTART1_LO_REG			0x107D
#define MIRA016_VSTART1_HI_REG			0x107E
#define MIRA016_VSTART1_MASK			0x7FF

// HSIZE units are number of columns / 2
#define MIRA016_HSIZE_LO_REG			0x2008
#define MIRA016_HSIZE_HI_REG			0x2009
#define MIRA016_HSIZE_MASK			0x3FF

// HSTART units are number of columns / 2
#define MIRA016_HSTART_LO_REG			0x200A
#define MIRA016_HSTART_HI_REG			0x200B
#define MIRA016_HSTART_MASK			0x3FF

// MIPI_HSIZE units are number of columns (HSIZE * 2)
#define MIRA016_MIPI_HSIZE_LO_REG		0x207D
#define MIRA016_MIPI_HSIZE_HI_REG		0x207E
#define MIRA016_MIPI_HSIZE_MASK			0xFFFF

#define MIRA016_HFLIP_REG			0x209C
#define MIRA016_HFLIP_ENABLE_MIRROR		1
#define MIRA016_HFLIP_DISABLE_MIRROR		0

#define MIRA016_VFLIP_REG			0x1095
#define MIRA016_VFLIP_ENABLE_FLIP		1
#define MIRA016_VFLIP_DISABLE_FLIP		0

#define MIRA016_BIT_ORDER_REG			0x2063
#define MIRA016_BIT_ORDER_NORMAL		0
#define MIRA016_BIT_ORDER_REVERSED		1

#define MIRA016_BSP_REG				0x4006
#define MIRA016_BSP_ENABLE			0x08
#define MIRA016_BSP_DISABLE			0x0F

#define MIRA016_MIPI_SOFT_RESET_REG		0x5004
#define MIRA016_MIPI_SOFT_RESET_DPHY		0x01
#define MIRA016_MIPI_SOFT_RESET_NONE		0x00

#define MIRA016_FSYNC_EOF_MAX_CTR_LO_REG	0x2066
#define MIRA016_FSYNC_EOF_MAX_CTR_HI_REG	0x2067

#define MIRA016_FSYNC_EOF_VEND_ST_LO_REG	0x206E
#define MIRA016_FSYNC_EOF_VEND_ST_HI_REG	0x206F

#define MIRA016_FSYNC_EOF_HSTART_EMB_ST_LO_REG	0x2076
#define MIRA016_FSYNC_EOF_HSTART_EMB_ST_HI_REG	0x2077

#define MIRA016_FSYNC_EOF_DSTART_EMB_ST_LO_REG	0x2078
#define MIRA016_FSYNC_EOF_DSTART_EMB_ST_HI_REG	0x2079

#define MIRA016_FSYNC_EOF_HEND_EMB_ST_LO_REG	0x207A
#define MIRA016_FSYNC_EOF_HEND_EMB_ST_HI_REG	0x207B

#define MIRA016_GLOB_NUM_CLK_CYCLES		1928

#define MIRA016_SUPPORTED_XCLK_FREQ		24000000

// Default exposure is adjusted to 1 ms
#define MIRA016_LUT_DEL_008			0
#define MIRA016_GRAN_TG				50
// TODO: Check Mira016 data rate in reg sequence
#define MIRA016_DATA_RATE			1000 // Mbit/s
// ROW_LENGTH register is 0x0032, with value 696 (10 bit). Choose smaller one for safety.
#define MIRA016_MIN_ROW_LENGTH			696
// Row time in millisecond is ROW_LENGTH times SEQ_TIME_BASE
#define MIRA016_MIN_ROW_LENGTH_US		(MIRA016_MIN_ROW_LENGTH * 8 / MIRA016_DATA_RATE)
// Mira016 EXP_TIME registe is in microsecond. V4L2 exposure value is in row time.
// Min exposure is set according to Mira016 datasheet, in microsecond.
#define MIRA016_EXPOSURE_MIN_US			(int)(1 + (151 + MIRA016_LUT_DEL_008) * MIRA016_GRAN_TG * 8 / MIRA016_DATA_RATE)
// Min exposure for V4L2 is in row time.
#define MIRA016_EXPOSURE_MIN_RT			(int)(1 + (151 + MIRA016_LUT_DEL_008) * MIRA016_GRAN_TG / MIRA016_MIN_ROW_LENGTH)
// Max exposure is set to TARGET_FRAME_TIME (32-bit reg 0x0008), in microsecond.
#define MIRA016_EXPOSURE_MAX_US			(33333)
// Max exposure for V4L2 is in row time.
#define MIRA016_EXPOSURE_MAX_RT			(int)(1 + MIRA016_EXPOSURE_MAX_US / MIRA016_MIN_ROW_LENGTH_US)
// Default exposure register is in microseconds
#define MIRA016_DEFAULT_EXPOSURE_US		1000
// Default exposure for V4L2 is in row time
#define MIRA016_DEFAULT_EXPOSURE_RT		(int)(1 + MIRA016_DEFAULT_EXPOSURE_US / MIRA016_MIN_ROW_LENGTH_US)

#define MIRA016_MIN_VBLANK			(11 + MIRA016_GLOB_NUM_CLK_CYCLES \
						    / MIRA016_MIN_ROW_LENGTH)

// Power on function timing
#define MIRA016_XCLR_MIN_DELAY_US		150000
#define MIRA016_XCLR_DELAY_RANGE_US		3000

/* Chip ID */
#define MIRA016_REG_CHIP_ID		0x0000
#define MIRA016_CHIP_ID			0x0054

#define MIRA016_REG_VALUE_08BIT		1
#define MIRA016_REG_VALUE_16BIT		2

// pixel_rate = link_freq * 2 * nr_of_lanes / bits_per_sample
// 1.Gb/s * 2 * 1 / 12 = 178956970
#define MIRA016_PIXEL_RATE		(178956970)
/* Should match device tree link freq */
#define MIRA016_DEFAULT_LINK_FREQ	456000000

/* Trick the libcamera with achievable fps via hblank */

/* Formular in libcamera to derive TARGET_FPS:
 * TARGET_FPS=1/((1/MIRA016_PIXEL_RATE)*(WIDTH+HBLANK)*(HEIGHT+MIRA016_MIN_VBLANK))
 * Example with HBLANK=0 and MIRA016_MIN_VBLANK=12
 * TARGET_FPS=1/((1/178956970)*400*(400+12))=1086
 * 
 * Inverse the above formula to derive HBLANK from TARGET_FPS:
 * HBLANK=1/((1/MIRA016_PIXEL_RATE)*TARGET_FPS*(HEIGHT+MIRA016_MIN_VBLANK))-WIDTH
 * Example with TARGET_FPS of 60 fps
 * HBLANK=1/((1/178956970)*60*(400+12))-400=6839
 */
#define MIRA016_HBLANK_60FPS			6839

// For test pattern with fixed data
#define MIRA016_TRAINING_WORD_REG		0x0060
// For test pattern with 2D gradiant
#define MIRA016_DELTA_TEST_IMG_REG		0x0056
// For setting test pattern type
#define MIRA016_TEST_PATTERN_REG		0x0062
#define	MIRA016_TEST_PATTERN_DISABLE		0x00
#define	MIRA016_TEST_PATTERN_FIXED_DATA		0x01
#define	MIRA016_TEST_PATTERN_2D_GRADIENT	0x02

/* Embedded metadata stream structure */
#define MIRA016_EMBEDDED_LINE_WIDTH 16384
#define MIRA016_NUM_EMBEDDED_LINES 1

/* From Jetson driver */
#define MIRA016_DEFAULT_LINE_LENGTH    (0xA80)
#define MIRA016_DEFAULT_PIXEL_CLOCK    (160)
#define MIRA016_DEFAULT_FRAME_LENGTH    (0x07C0)

#define MIRA016_PLL_LOCKED_REG			0x207C
#define MIRA016_CSI2_TX_HS_ACTIVE_REG		0x209A

#define MIRA016_CURRENT_ACTIVE_CONTEXT	0x4002

#define MIRA016_GDIG_AMP		0x0024
#define MIRA016_BIAS_RG_ADCGAIN		0x01F0
#define MIRA016_BIAS_RG_MULT		0x01F3
#define MIRA016_OFFSET_CLIPPING		0x0193

#define MIRA016_OTP_COMMAND	0x0066
#define MIRA016_OTP_ADDR	0x0067
#define MIRA016_OTP_START	0x0064
#define MIRA016_OTP_BUSY	0x0065
#define MIRA016_OTP_DOUT	0x006C

enum pad_types {
	IMAGE_PAD,
	METADATA_PAD,
	NUM_PADS
};

struct mira016_reg {
	u16 address;
	u8 val;
};

struct mira016_fine_gain_lut {
	u8 gdig_amp;
	u8 rg_adcgain;
	u8 rg_mult;
};

struct mira016_reg_list {
	unsigned int num_of_regs;
	const struct mira016_reg *regs;
};

struct mira016_v4l2_reg {
	u32 val;
};

struct mira016_v4l2_reg_list {
	unsigned int num_of_regs;
	struct mira016_v4l2_reg *regs;
};


/* Mode : resolution and related config&values */
struct mira016_mode {
	/* Frame width */
	unsigned int width;
	/* Frame height */
	unsigned int height;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* Default register values */
	struct mira016_reg_list reg_list_pre_soft_reset;
	struct mira016_reg_list reg_list_post_soft_reset;

	u32 vblank;
	u32 hblank;

	/* Format code */
	u32 code;

	/* bit_depth needed for analog gain selection */
	u8 bit_depth;
};

// Allocate a buffer to store custom reg write
#define AMS_CAMERA_CID_MIRA016_REG_W_BUF_SIZE	2048
static struct mira016_v4l2_reg s_ctrl_mira016_reg_w_buf[AMS_CAMERA_CID_MIRA016_REG_W_BUF_SIZE];
static struct mira016_v4l2_reg_list reg_list_s_ctrl_mira016_reg_w_buf = {
	.num_of_regs = 0,
        .regs = s_ctrl_mira016_reg_w_buf,
};

// 400_400_60fps_10b_1lane
static const struct mira016_reg full_400_400_60fps_10b_1lane_reg_pre_soft_reset[] = {
	//Base Configuration
	{0xE000, 0},
	{0xE17E, 1},
	{0xE000, 0},
	{0x01E4, 0},
	{0x01E5, 19},
	{0x01E2, 23},
	{0x01E3, 168},
	{0x01E6, 0},
	{0x01E7, 202},
	{0x016C, 1},
	{0x016B, 1},
	{0x0208, 1},
	{0x0209, 240},
	{0x020A, 3},
	{0x020B, 77},
	{0x020C, 2},
	{0x020D, 16},
	{0x020E, 3},
	{0x020F, 1},
	{0x0210, 0},
	{0x0211, 19},
	{0x0212, 0},
	{0x0213, 3},
	{0x0214, 3},
	{0x0215, 239},
	{0x0216, 3},
	{0x0217, 243},
	{0x0218, 3},
	{0x0219, 244},
	{0x021A, 1},
	{0x021B, 241},
	{0x021C, 3},
	{0x021D, 36},
	{0x021E, 0},
	{0x021F, 2},
	{0x0220, 1},
	{0x0221, 242},
	{0x0222, 3},
	{0x0223, 47},
	{0x0224, 0},
	{0x0225, 33},
	{0x0226, 3},
	{0x0227, 240},
	{0x0228, 3},
	{0x0229, 241},
	{0x022A, 3},
	{0x022B, 242},
	{0x022C, 3},
	{0x022D, 245},
	{0x022E, 3},
	{0x022F, 246},
	{0x0230, 0},
	{0x0231, 193},
	{0x0232, 0},
	{0x0233, 2},
	{0x0234, 1},
	{0x0235, 242},
	{0x0236, 3},
	{0x0237, 107},
	{0x0238, 3},
	{0x0239, 255},
	{0x023A, 3},
	{0x023B, 49},
	{0x023C, 1},
	{0x023D, 240},
	{0x023E, 3},
	{0x023F, 135},
	{0x0240, 0},
	{0x0241, 10},
	{0x0242, 0},
	{0x0243, 11},
	{0x0244, 1},
	{0x0245, 249},
	{0x0246, 3},
	{0x0247, 13},
	{0x0248, 0},
	{0x0249, 7},
	{0x024A, 3},
	{0x024B, 239},
	{0x024C, 3},
	{0x024D, 243},
	{0x024E, 3},
	{0x024F, 244},
	{0x0250, 3},
	{0x0251, 0},
	{0x0252, 0},
	{0x0253, 7},
	{0x0254, 0},
	{0x0255, 12},
	{0x0256, 1},
	{0x0257, 241},
	{0x0258, 3},
	{0x0259, 67},
	{0x025A, 1},
	{0x025B, 248},
	{0x025C, 3},
	{0x025D, 16},
	{0x025E, 0},
	{0x025F, 7},
	{0x0260, 3},
	{0x0261, 240},
	{0x0262, 3},
	{0x0263, 241},
	{0x0264, 3},
	{0x0265, 242},
	{0x0266, 3},
	{0x0267, 245},
	{0x0268, 3},
	{0x0269, 246},
	{0x026A, 3},
	{0x026B, 0},
	{0x026C, 2},
	{0x026D, 135},
	{0x026E, 0},
	{0x026F, 1},
	{0x0270, 3},
	{0x0271, 255},
	{0x0272, 3},
	{0x0273, 0},
	{0x0274, 3},
	{0x0275, 255},
	{0x0276, 2},
	{0x0277, 135},
	{0x0278, 3},
	{0x0279, 2},
	{0x027A, 3},
	{0x027B, 15},
	{0x027C, 3},
	{0x027D, 247},
	{0x027E, 0},
	{0x027F, 22},
	{0x0280, 0},
	{0x0281, 51},
	{0x0282, 0},
	{0x0283, 4},
	{0x0284, 0},
	{0x0285, 17},
	{0x0286, 3},
	{0x0287, 9},
	{0x0288, 0},
	{0x0289, 2},
	{0x028A, 0},
	{0x028B, 32},
	{0x028C, 0},
	{0x028D, 181},
	{0x028E, 0},
	{0x028F, 229},
	{0x0290, 0},
	{0x0291, 18},
	{0x0292, 0},
	{0x0293, 181},
	{0x0294, 0},
	{0x0295, 229},
	{0x0296, 0},
	{0x0297, 0},
	{0x0298, 0},
	{0x0299, 18},
	{0x029A, 0},
	{0x029B, 18},
	{0x029C, 0},
	{0x029D, 32},
	{0x029E, 0},
	{0x029F, 181},
	{0x02A0, 0},
	{0x02A1, 229},
	{0x02A2, 0},
	{0x02A3, 0},
	{0x02A4, 0},
	{0x02A5, 18},
	{0x02A6, 0},
	{0x02A7, 18},
	{0x02A8, 0},
	{0x02A9, 32},
	{0x02AA, 0},
	{0x02AB, 71},
	{0x02AC, 0},
	{0x02AD, 39},
	{0x02AE, 0},
	{0x02AF, 181},
	{0x02B0, 0},
	{0x02B1, 229},
	{0x02B2, 0},
	{0x02B3, 0},
	{0x02B4, 0},
	{0x02B5, 4},
	{0x02B6, 0},
	{0x02B7, 67},
	{0x02B8, 0},
	{0x02B9, 1},
	{0x02BA, 3},
	{0x02BB, 2},
	{0x02BC, 0},
	{0x02BD, 8},
	{0x02BE, 3},
	{0x02BF, 255},
	{0x02C0, 2},
	{0x02C1, 135},
	{0x02C2, 3},
	{0x02C3, 137},
	{0x02C4, 3},
	{0x02C5, 247},
	{0x02C6, 0},
	{0x02C7, 119},
	{0x02C8, 0},
	{0x02C9, 23},
	{0x02CA, 0},
	{0x02CB, 8},
	{0x02CC, 3},
	{0x02CD, 255},
	{0x02CE, 0},
	{0x02CF, 56},
	{0x02D0, 0},
	{0x02D1, 23},
	{0x02D2, 0},
	{0x02D3, 8},
	{0x02D4, 3},
	{0x02D5, 255},
	{0x02D6, 3},
	{0x02D7, 255},
	{0x02D8, 3},
	{0x02D9, 255},
	{0x02DA, 3},
	{0x02DB, 255},
	{0x02DC, 3},
	{0x02DD, 255},
	{0x02DE, 3},
	{0x02DF, 255},
	{0x02E0, 3},
	{0x02E1, 255},
	{0x02E2, 3},
	{0x02E3, 255},
	{0x02E4, 3},
	{0x02E5, 255},
	{0x02E6, 3},
	{0x02E7, 255},
	{0x02E8, 3},
	{0x02E9, 255},
	{0x02EA, 3},
	{0x02EB, 255},
	{0x02EC, 3},
	{0x02ED, 255},
	{0x02EE, 3},
	{0x02EF, 255},
	{0x02F0, 3},
	{0x02F1, 255},
	{0x02F2, 3},
	{0x02F3, 255},
	{0x02F4, 3},
	{0x02F5, 255},
	{0x02F6, 3},
	{0x02F7, 255},
	{0x02F8, 3},
	{0x02F9, 255},
	{0x02FA, 3},
	{0x02FB, 255},
	{0x02FC, 3},
	{0x02FD, 255},
	{0x02FE, 3},
	{0x02FF, 255},
	{0x0300, 3},
	{0x0301, 255},
	{0x0302, 3},
	{0x0303, 255},
	{0x01E9, 0},
	{0x01E8, 25},
	{0x01EA, 53},
	{0x01EB, 55},
	{0x01EC, 92},
	{0x01ED, 99},
	{0x01F8, 15},
	{0x01D8, 1},
	{0x01DC, 1},
	{0x01DE, 1},
	{0x0189, 1},
	{0x01B7, 1},
	{0x01C1, 7},
	{0x01C2, 246},
	{0x01C3, 255},
	{0x01C9, 7},
	{0x0325, 0},
	{0x033A, 0},
	{0x01B8, 1},
	{0x01BA, 51},
	{0x01BE, 116},
	{0x01BF, 54},
	{0x01C0, 83},
	{0x00EF, 0},
	{0x0326, 0},
	{0x00F0, 0},
	{0x00F1, 0},
	{0x0327, 0},
	{0x00F2, 0},
	{0x0071, 1},
	{0x01B4, 1},
	{0x01B5, 1},
	{0x01F1, 1},
	{0x01F4, 1},
	{0x01F5, 1},
	{0x0314, 1},
	{0x0315, 1},
	{0x0316, 1},
	{0x0207, 0},
	{0x4207, 2},
	{0x2207, 2},
	{0xE088, 1},
	{0xE08D, 0},
	{0xE08E, 48},
	{0xE08F, 212},
	{0xE089, 78},
	{0xE08A, 16},
	{0xE08B, 31},
	{0xE08C, 15},
	{0xE0A6, 0},
	{0xE0A9, 16},
	{0xE0AA, 0},
	{0xE0AD, 10},
	{0xE0A8, 48},
	{0xE0A7, 15},
	{0xE0AC, 16},
	{0xE0AB, 15},
	{0x209D, 0},
	{0x0328, 0},
	{0x0063, 1},
	{0x01F7, 15},
	{0xE0E3, 1},
	{0xE0E7, 3},
	{0xE33B, 0},
	{0xE336, 0},
	{0xE337, 0},
	{0xE338, 0},
	{0xE339, 0},
	{0x00E9, 1},
	{0x00EA, 254},
	{0x0309, 3},
	{0x030A, 2},
	{0x030B, 2},
	{0x030C, 5},
	{0x030E, 21},
	{0x030D, 20},
	{0x030F, 1},
	{0x0310, 13},
	{0x01D0, 31},
	{0x01D1, 31},
	{0x01CD, 17},
	{0x0016, 0},
	{0x0017, 5},
	{0x00E8, 3},
	{0x01F0, 37},
	{0x01F2, 0},
	{0x00D0, 0},
	{0xE0C0, 0},
	{0xE0C1, 16},
	{0xE0C2, 0},
	{0xE0C3, 16},
	{0x016A, 1},
	{0x0168, 43},
	{0x005C, 0},
	{0x005D, 22},
	{0x2000, 0},
	{0xE000, 0},
	{0xE0A2, 0},
	{0xE07B, 0},
	{0xE078, 0},
	{0xE079, 1},
	{0x2077, 0},
	{0x2076, 189},
	{0x00CE, 1},
	{0x0070, 9},
	{0x016D, 50},
	{0x0176, 0},
	{0xE136, 0},
	{0xE0C6, 0},
	{0xE0C7, 0},
	{0xE0C8, 1},
	{0xE0C9, 0},
	{0xE0CA, 0},
	{0xE0CB, 1},
	{0xE0CC, 128},
	{0xE0CD, 128},
	{0xE0BE, 2},
	{0xE0BF, 2},
	{0xE0C4, 8},
	{0xE0C5, 8},
	{0x2075, 0},
	{0x2000, 0},
	{0xE000, 0},
	{0x001E, 0},
	{0xE000, 0},
	{0x207E, 0},
	{0xE084, 0},
	{0xE085, 0},
	{0xE086, 1},
	{0xE087, 0},
	{0x207F, 0},
	{0x2080, 0},
	{0x2081, 3},
	{0x2082, 0},
	{0x2083, 2},
	{0x0090, 1},
	{0x2097, 0},
	{0xE000, 0},
	{0x0011, 3},
	{0x011D, 1},
	{0xE000, 0},
	{0x0012, 0},
	{0x0013, 24},
	{0x015A, 0},
	{0x015B, 51},
	{0x015C, 0},
	{0x015D, 51},
	{0x015E, 0},
	{0x015F, 51},
	{0x0162, 0},
	{0x0163, 5},
	{0x0164, 4},
	{0x0165, 121},
	{0x0166, 4},
	{0x0167, 121},
	{0xE000, 0},
	{0x01BB, 180},
	{0x01BC, 172},
	{0x01F3, 2},
	{0x016E, 61},
	{0x0172, 0},
	{0x0173, 0},
	{0x016F, 126},
	{0x0170, 0},
	{0x0171, 61},
	{0x0174, 0},
	{0x0175, 0},
	{0x018B, 1},
	{0x018C, 14},
	{0x018D, 2},
	{0x018E, 86},
	{0x018F, 3},
	{0x0190, 14},
	{0x01EE, 27},
	{0x01EF, 70},
	{0x01A2, 0},
	{0x01A3, 1},
	{0x031F, 0},
	{0x0320, 10},
	{0x01A6, 0},
	{0x01A7, 152},
	{0x01A4, 2},
	{0x01A5, 167},
	{0x0321, 2},
	{0x0322, 176},
	{0x01A8, 3},
	{0x01A9, 62},
	{0x01A0, 0},
	{0x01A1, 154},
	{0x01B2, 0},
	{0x01B3, 188},
	{0x01B0, 0},
	{0x01B1, 173},
	{0x01AC, 0},
	{0x01AD, 184},
	{0xE000, 0},
	{0x0193, 2},
	{0x0194, 4},
	{0xE000, 0},
	{0xE009, 1},
	{0x212F, 1},
	{0x2130, 1},
	{0x2131, 1},
	{0x2132, 1},
	{0x2133, 1},
	{0x2134, 1},
	{0x2135, 1},
	{0xE0E1, 1},
	{0x018A, 1},
	{0x00E0, 1},
	{0xE32E, 1},
	{0xE340, 1},
	{0xE000, 0},
	{0xE1D9, 1},
	{0xE000, 0},
	{0xE0EB, 16},
	{0xE000, 0},
	{0xE1DD, 1},
	{0xE000, 0},
	{0xE0ED, 14},
	{0xE000, 0},
	{0xE1DF, 1},
	{0xE000, 0},
	{0xE0EE, 4},
	{0xE000, 0},
	{0xE335, 1},
	{0xE000, 0},
	{0xE324, 42},
	{0xE000, 0},
	{0xE1FA, 1},
	{0xE004, 0},
	{0xE000, 1},
	{0x001E, 0},
	{0x001F, 2},
	{0x002B, 0},
	{0xE004, 1},
	{0x001E, 0},
	{0x001F, 2},
	{0x002B, 0},
	{0xE000, 0},
	{0x001F, 0},
	{0x0020, 0},
	{0x0023, 0},
	{0x0024, 1},
	{0x0025, 144},
	{0x0026, 0},
	{0x0027, 14},
	{0x0028, 0},
	{0x0029, 1},
	{0x002A, 144},
	{0x002B, 0},
	{0x002C, 14},
	{0x002D, 0},
	{0x002E, 0},
	{0x002F, 0},
	{0x0030, 0},
	{0x0031, 0},
	{0x0032, 0},
	{0x0033, 0},
	{0x0034, 0},
	{0x0035, 0},
	{0x0036, 0},
	{0x0037, 0},
	{0x0038, 0},
	{0x0039, 0},
	{0x003A, 0},
	{0x003B, 0},
	{0x003C, 0},
	{0x003D, 0},
	{0x003E, 0},
	{0x003F, 0},
	{0x0040, 0},
	{0x0041, 0},
	{0x0042, 0},
	{0x0043, 0},
	{0x0044, 0},
	{0x0045, 0},
	{0x0046, 0},
	{0x0047, 0},
	{0x0048, 0},
	{0x0049, 0},
	{0x004A, 0},
	{0x004B, 0},
	{0x004C, 0},
	{0x004D, 0},
	{0x004E, 0},
	{0x004F, 0},
	{0x0050, 0},
	{0x0051, 0},
	{0x0052, 0},
	{0x0053, 0},
	{0x0054, 0},
	{0x0055, 0},
	{0xE004, 0},
	{0xE000, 1},
	{0xE02C, 0},
	{0xE02D, 14},
	{0xE02E, 1},
	{0xE02F, 157},
	{0xE030, 0},
	{0xE025, 0},
	{0xE02A, 0},
	{0x2029, 40},
	{0x0034, 0},
	{0x0035, 200},
	{0xE004, 0},
	{0x001E, 0},
	{0x001F, 2},
	{0x002B, 0},
	{0xE004, 0},
	{0x000E, 0},
	{0x000F, 0},
	{0x0010, 3},
	{0x0011, 232},
	{0x0012, 0},
	{0x0013, 0},
	{0x0014, 0},
	{0x0015, 0},
	{0xE004, 0},
	{0x0032, 2},
	{0x0033, 184},
	{0xE004, 0},
	{0x0007, 1},
	{0x0008, 0},
	{0x0009, 0},
	{0x000A, 65},
	{0x000B, 26},
	{0xE004, 0},
	{0x0024, 15},
	{0xE004, 0},
	{0x0031, 0},
	{0xE004, 0},
	{0x0026, 0},
	{0xE004, 0},
	{0x001C, 0},
	{0x0019, 0},
	{0x001A, 7},
	{0x001B, 83},
	{0x0016, 8},
	{0x0017, 0},
	{0x0018, 0},
	{0xE004, 0},
	{0x001D, 0},
	{0xE000, 0},
	{0xE000, 0},
	{0xE32C, 1},
	{0xE000, 0},
	{0xE32D, 1},
	{0xE000, 0},
	{0xE14E, 0},
	{0xE000, 0},
	{0xE312, 127},
	{0xE004, 0},
	{0x0000, 1},
	{0x001E, 0},
	{0x001F, 2},
	{0xE004, 1},
	{0x0000, 1},
	{0x001E, 0},
	{0x001F, 2},
	{0xE000, 0},
	{0xE329, 3},
	{0xE000, 0},
	{0xE32B, 0},
	{0xE000, 0},
	{0xE331, 4},
	{0xE000, 0},
	{0xE332, 8},
	{0xE000, 0},
	{0xE32A, 0},
	{0xE000, 0},
	{0xE11E, 1},
	{0xE000, 0},
	{0xE000, 0},
	{0xE062, 0},
	{0xE000, 0},
	{0xE056, 0},
	{0xE000, 0},
	{0xE060, 3},
	{0xE061, 232},
	{0xE000, 0},
	{0x0066, 0},
	{0x0067, 100},
	{0x0064, 1},
	{0x0064, 0},
	{0x0000, 0},
	{0x0000, 0},
	{0xE004, 0},
	{0xE000, 1},
	{0x000E, 0},
	{0x000F, 0},
	{0x0010, 3},
	{0x0011, 232},
	{0x0012, 0},
	{0x0013, 0},
	{0x0014, 0},
	{0x0015, 0},
	{0xE004, 1},
	{0x000E, 0},
	{0x000F, 0},
	{0x0010, 3},
	{0x0011, 232},
	{0x0012, 0},
	{0x0013, 0},
	{0x0014, 0},
	{0x0015, 0},
	{0xE004, 0},
	{0xE000, 1},
	{0x0007, 2},
	{0x0008, 0},
	{0x0009, 0},
	{0x000A, 65},
	{0x000B, 26},
	{0xE004, 1},
	{0x0007, 1},
	{0x0008, 0},
	{0x0009, 0},
	{0x000A, 65},
	{0x000B, 27},

	// Below are manually added after reg seq txt
	{0x0335,1} ,//iref sel
	{0x0324 , 43}, //iref val

	{0x1d9 , 1},// #vddana sel
	{0x0EB,15} ,//#vddana val trim

	{0x1dd ,1} ,//# vsspc sel
	{0x1ed , 20} ,//# vsspc val

	{0x1df,1} ,//#cp sel
	{0x0ee,4} ,//#cp trim,	
};

static const struct mira016_reg full_400_400_60fps_10b_1lane_reg_post_soft_reset[] = {
	// Release Soft Reset
	{57344,  0},
};

// 400_400_60fps_12b_1lane
static const struct mira016_reg full_400_400_60fps_12b_1lane_reg_pre_soft_reset[] = {
	//Base Configuration
	{0xE000, 0},
	{0xE17E, 1},
	{0xE000, 0},
	{0x01E4, 0},
	{0x01E5, 19},
	{0x01E2, 23},
	{0x01E3, 168},
	{0x01E6, 0},
	{0x01E7, 202},
	{0x016C, 1},
	{0x016B, 1},
	{0x0208, 1},
	{0x0209, 240},
	{0x020A, 3},
	{0x020B, 77},
	{0x020C, 2},
	{0x020D, 16},
	{0x020E, 3},
	{0x020F, 1},
	{0x0210, 0},
	{0x0211, 19},
	{0x0212, 0},
	{0x0213, 3},
	{0x0214, 3},
	{0x0215, 239},
	{0x0216, 3},
	{0x0217, 243},
	{0x0218, 3},
	{0x0219, 244},
	{0x021A, 1},
	{0x021B, 241},
	{0x021C, 3},
	{0x021D, 36},
	{0x021E, 0},
	{0x021F, 2},
	{0x0220, 1},
	{0x0221, 242},
	{0x0222, 3},
	{0x0223, 47},
	{0x0224, 0},
	{0x0225, 33},
	{0x0226, 3},
	{0x0227, 240},
	{0x0228, 3},
	{0x0229, 241},
	{0x022A, 3},
	{0x022B, 242},
	{0x022C, 3},
	{0x022D, 245},
	{0x022E, 3},
	{0x022F, 246},
	{0x0230, 0},
	{0x0231, 193},
	{0x0232, 0},
	{0x0233, 2},
	{0x0234, 1},
	{0x0235, 242},
	{0x0236, 3},
	{0x0237, 107},
	{0x0238, 3},
	{0x0239, 255},
	{0x023A, 3},
	{0x023B, 49},
	{0x023C, 1},
	{0x023D, 240},
	{0x023E, 3},
	{0x023F, 135},
	{0x0240, 0},
	{0x0241, 10},
	{0x0242, 0},
	{0x0243, 11},
	{0x0244, 1},
	{0x0245, 249},
	{0x0246, 3},
	{0x0247, 13},
	{0x0248, 0},
	{0x0249, 7},
	{0x024A, 3},
	{0x024B, 239},
	{0x024C, 3},
	{0x024D, 243},
	{0x024E, 3},
	{0x024F, 244},
	{0x0250, 3},
	{0x0251, 0},
	{0x0252, 0},
	{0x0253, 7},
	{0x0254, 0},
	{0x0255, 12},
	{0x0256, 1},
	{0x0257, 241},
	{0x0258, 3},
	{0x0259, 67},
	{0x025A, 1},
	{0x025B, 248},
	{0x025C, 3},
	{0x025D, 16},
	{0x025E, 0},
	{0x025F, 7},
	{0x0260, 3},
	{0x0261, 240},
	{0x0262, 3},
	{0x0263, 241},
	{0x0264, 3},
	{0x0265, 242},
	{0x0266, 3},
	{0x0267, 245},
	{0x0268, 3},
	{0x0269, 246},
	{0x026A, 3},
	{0x026B, 0},
	{0x026C, 2},
	{0x026D, 135},
	{0x026E, 0},
	{0x026F, 1},
	{0x0270, 3},
	{0x0271, 255},
	{0x0272, 3},
	{0x0273, 0},
	{0x0274, 3},
	{0x0275, 255},
	{0x0276, 2},
	{0x0277, 135},
	{0x0278, 3},
	{0x0279, 2},
	{0x027A, 3},
	{0x027B, 15},
	{0x027C, 3},
	{0x027D, 247},
	{0x027E, 0},
	{0x027F, 22},
	{0x0280, 0},
	{0x0281, 51},
	{0x0282, 0},
	{0x0283, 4},
	{0x0284, 0},
	{0x0285, 17},
	{0x0286, 3},
	{0x0287, 9},
	{0x0288, 0},
	{0x0289, 2},
	{0x028A, 0},
	{0x028B, 32},
	{0x028C, 0},
	{0x028D, 181},
	{0x028E, 0},
	{0x028F, 229},
	{0x0290, 0},
	{0x0291, 18},
	{0x0292, 0},
	{0x0293, 181},
	{0x0294, 0},
	{0x0295, 229},
	{0x0296, 0},
	{0x0297, 0},
	{0x0298, 0},
	{0x0299, 18},
	{0x029A, 0},
	{0x029B, 18},
	{0x029C, 0},
	{0x029D, 32},
	{0x029E, 0},
	{0x029F, 181},
	{0x02A0, 0},
	{0x02A1, 229},
	{0x02A2, 0},
	{0x02A3, 0},
	{0x02A4, 0},
	{0x02A5, 18},
	{0x02A6, 0},
	{0x02A7, 18},
	{0x02A8, 0},
	{0x02A9, 32},
	{0x02AA, 0},
	{0x02AB, 71},
	{0x02AC, 0},
	{0x02AD, 39},
	{0x02AE, 0},
	{0x02AF, 181},
	{0x02B0, 0},
	{0x02B1, 229},
	{0x02B2, 0},
	{0x02B3, 0},
	{0x02B4, 0},
	{0x02B5, 4},
	{0x02B6, 0},
	{0x02B7, 67},
	{0x02B8, 0},
	{0x02B9, 1},
	{0x02BA, 3},
	{0x02BB, 2},
	{0x02BC, 0},
	{0x02BD, 8},
	{0x02BE, 3},
	{0x02BF, 255},
	{0x02C0, 2},
	{0x02C1, 135},
	{0x02C2, 3},
	{0x02C3, 137},
	{0x02C4, 3},
	{0x02C5, 247},
	{0x02C6, 0},
	{0x02C7, 119},
	{0x02C8, 0},
	{0x02C9, 23},
	{0x02CA, 0},
	{0x02CB, 8},
	{0x02CC, 3},
	{0x02CD, 255},
	{0x02CE, 0},
	{0x02CF, 56},
	{0x02D0, 0},
	{0x02D1, 23},
	{0x02D2, 0},
	{0x02D3, 8},
	{0x02D4, 3},
	{0x02D5, 255},
	{0x02D6, 3},
	{0x02D7, 255},
	{0x02D8, 3},
	{0x02D9, 255},
	{0x02DA, 3},
	{0x02DB, 255},
	{0x02DC, 3},
	{0x02DD, 255},
	{0x02DE, 3},
	{0x02DF, 255},
	{0x02E0, 3},
	{0x02E1, 255},
	{0x02E2, 3},
	{0x02E3, 255},
	{0x02E4, 3},
	{0x02E5, 255},
	{0x02E6, 3},
	{0x02E7, 255},
	{0x02E8, 3},
	{0x02E9, 255},
	{0x02EA, 3},
	{0x02EB, 255},
	{0x02EC, 3},
	{0x02ED, 255},
	{0x02EE, 3},
	{0x02EF, 255},
	{0x02F0, 3},
	{0x02F1, 255},
	{0x02F2, 3},
	{0x02F3, 255},
	{0x02F4, 3},
	{0x02F5, 255},
	{0x02F6, 3},
	{0x02F7, 255},
	{0x02F8, 3},
	{0x02F9, 255},
	{0x02FA, 3},
	{0x02FB, 255},
	{0x02FC, 3},
	{0x02FD, 255},
	{0x02FE, 3},
	{0x02FF, 255},
	{0x0300, 3},
	{0x0301, 255},
	{0x0302, 3},
	{0x0303, 255},
	{0x01E9, 0},
	{0x01E8, 25},
	{0x01EA, 53},
	{0x01EB, 55},
	{0x01EC, 92},
	{0x01ED, 99},
	{0x01F8, 15},
	{0x01D8, 1},
	{0x01DC, 1},
	{0x01DE, 1},
	{0x0189, 1},
	{0x01B7, 1},
	{0x01C1, 7},
	{0x01C2, 246},
	{0x01C3, 255},
	{0x01C9, 7},
	{0x0325, 0},
	{0x033A, 0},
	{0x01B8, 1},
	{0x01BA, 51},
	{0x01BE, 116},
	{0x01BF, 54},
	{0x01C0, 83},
	{0x00EF, 0},
	{0x0326, 0},
	{0x00F0, 0},
	{0x00F1, 0},
	{0x0327, 0},
	{0x00F2, 0},
	{0x0071, 1},
	{0x01B4, 1},
	{0x01B5, 1},
	{0x01F1, 1},
	{0x01F4, 1},
	{0x01F5, 1},
	{0x0314, 1},
	{0x0315, 1},
	{0x0316, 1},
	{0x0207, 0},
	{0x4207, 2},
	{0x2207, 2},
	{0xE088, 1},
	{0xE08D, 0},
	{0xE08E, 48},
	{0xE08F, 212},
	{0xE089, 78},
	{0xE08A, 16},
	{0xE08B, 31},
	{0xE08C, 15},
	{0xE0A6, 0},
	{0xE0A9, 16},
	{0xE0AA, 0},
	{0xE0AD, 10},
	{0xE0A8, 48},
	{0xE0A7, 15},
	{0xE0AC, 16},
	{0xE0AB, 15},
	{0x209D, 0},
	{0x0328, 0},
	{0x0063, 1},
	{0x01F7, 15},
	{0xE0E3, 1},
	{0xE0E7, 3},
	{0xE33B, 0},
	{0xE336, 0},
	{0xE337, 0},
	{0xE338, 0},
	{0xE339, 0},
	{0x00E9, 1},
	{0x00EA, 254},
	{0x0309, 3},
	{0x030A, 2},
	{0x030B, 2},
	{0x030C, 5},
	{0x030E, 21},
	{0x030D, 20},
	{0x030F, 1},
	{0x0310, 13},
	{0x01D0, 31},
	{0x01D1, 31},
	{0x01CD, 17},
	{0x0016, 0},
	{0x0017, 5},
	{0x00E8, 3},
	{0x01F0, 7},
	{0x01F2, 0},
	{0x00D0, 0},
	{0xE0C0, 0},
	{0xE0C1, 32},
	{0xE0C2, 0},
	{0xE0C3, 32},
	{0x016A, 2},
	{0x0168, 44},
	{0x005C, 0},
	{0x005D, 91},
	{0x2000, 0},
	{0xE000, 0},
	{0xE0A2, 0},
	{0xE07B, 0},
	{0xE078, 0},
	{0xE079, 1},
	{0x2077, 0},
	{0x2076, 189},
	{0x00CE, 1},
	{0x0070, 9},
	{0x016D, 50},
	{0x0176, 0},
	{0xE136, 0},
	{0xE0C6, 0},
	{0xE0C7, 0},
	{0xE0C8, 1},
	{0xE0C9, 0},
	{0xE0CA, 0},
	{0xE0CB, 1},
	{0xE0CC, 128},
	{0xE0CD, 128},
	{0xE0BE, 2},
	{0xE0BF, 2},
	{0xE0C4, 8},
	{0xE0C5, 8},
	{0x2075, 0},
	{0x2000, 0},
	{0xE000, 0},
	{0x001E, 0},
	{0xE000, 0},
	{0x207E, 0},
	{0xE084, 0},
	{0xE085, 0},
	{0xE086, 1},
	{0xE087, 0},
	{0x207F, 0},
	{0x2080, 0},
	{0x2081, 3},
	{0x2082, 0},
	{0x2083, 2},
	{0x0090, 1},
	{0x2097, 0},
	{0xE000, 0},
	{0x0011, 3},
	{0x011D, 1},
	{0xE000, 0},
	{0x0012, 0},
	{0x0013, 24},
	{0x015A, 0},
	{0x015B, 51},
	{0x015C, 0},
	{0x015D, 51},
	{0x015E, 0},
	{0x015F, 51},
	{0x0162, 0},
	{0x0163, 5},
	{0x0164, 4},
	{0x0165, 121},
	{0x0166, 4},
	{0x0167, 121},
	{0xE000, 0},
	{0x01BB, 180},
	{0x01BC, 172},
	{0x01F3, 1},
	{0x016E, 253},
	{0x0172, 0},
	{0x0173, 0},
	{0x016F, 255},
	{0x0170, 255},
	{0x0171, 253},
	{0x0174, 0},
	{0x0175, 0},
	{0x018B, 4},
	{0x018C, 14},
	{0x018D, 2},
	{0x018E, 86},
	{0x018F, 12},
	{0x0190, 14},
	{0x01EE, 27},
	{0x01EF, 70},
	{0x01A2, 0},
	{0x01A3, 1},
	{0x031F, 0},
	{0x0320, 10},
	{0x01A6, 0},
	{0x01A7, 152},
	{0x01A4, 5},
	{0x01A5, 167},
	{0x0321, 5},
	{0x0322, 176},
	{0x01A8, 6},
	{0x01A9, 62},
	{0x01A0, 0},
	{0x01A1, 192},
	{0x01B2, 0},
	{0x01B3, 226},
	{0x01B0, 0},
	{0x01B1, 211},
	{0x01AC, 0},
	{0x01AD, 222},
	{0xE000, 0},
	{0x0193, 7},
	{0x0194, 188},
	{0xE000, 0},
	{0xE009, 1},
	{0x212F, 1},
	{0x2130, 1},
	{0x2131, 1},
	{0x2132, 1},
	{0x2133, 1},
	{0x2134, 1},
	{0x2135, 1},
	{0xE0E1, 1},
	{0x018A, 1},
	{0x00E0, 1},
	{0xE32E, 1},
	{0xE340, 1},
	{0xE000, 0},
	{0xE1D9, 1},
	{0xE000, 0},
	{0xE0EB, 16},
	{0xE000, 0},
	{0xE1DD, 1},
	{0xE000, 0},
	{0xE0ED, 14},
	{0xE000, 0},
	{0xE1DF, 1},
	{0xE000, 0},
	{0xE0EE, 4},
	{0xE000, 0},
	{0xE335, 1},
	{0xE000, 0},
	{0xE324, 42},
	{0xE000, 0},
	{0xE1FA, 1},
	{0xE004, 0},
	{0xE000, 1},
	{0x001E, 0},
	{0x001F, 2},
	{0x002B, 0},
	{0xE004, 1},
	{0x001E, 0},
	{0x001F, 2},
	{0x002B, 0},
	{0xE000, 0},
	{0x001F, 0},
	{0x0020, 0},
	{0x0023, 0},
	{0x0024, 1},
	{0x0025, 144},
	{0x0026, 0},
	{0x0027, 14},
	{0x0028, 0},
	{0x0029, 1},
	{0x002A, 144},
	{0x002B, 0},
	{0x002C, 14},
	{0x002D, 0},
	{0x002E, 0},
	{0x002F, 0},
	{0x0030, 0},
	{0x0031, 0},
	{0x0032, 0},
	{0x0033, 0},
	{0x0034, 0},
	{0x0035, 0},
	{0x0036, 0},
	{0x0037, 0},
	{0x0038, 0},
	{0x0039, 0},
	{0x003A, 0},
	{0x003B, 0},
	{0x003C, 0},
	{0x003D, 0},
	{0x003E, 0},
	{0x003F, 0},
	{0x0040, 0},
	{0x0041, 0},
	{0x0042, 0},
	{0x0043, 0},
	{0x0044, 0},
	{0x0045, 0},
	{0x0046, 0},
	{0x0047, 0},
	{0x0048, 0},
	{0x0049, 0},
	{0x004A, 0},
	{0x004B, 0},
	{0x004C, 0},
	{0x004D, 0},
	{0x004E, 0},
	{0x004F, 0},
	{0x0050, 0},
	{0x0051, 0},
	{0x0052, 0},
	{0x0053, 0},
	{0x0054, 0},
	{0x0055, 0},
	{0xE004, 0},
	{0xE000, 1},
	{0xE02C, 0},
	{0xE02D, 14},
	{0xE02E, 1},
	{0xE02F, 157},
	{0xE030, 0},
	{0xE025, 0},
	{0xE02A, 0},
	{0x2029, 40},
	{0x0034, 0},
	{0x0035, 200},
	{0xE004, 0},
	{0x001E, 0},
	{0x001F, 2},
	{0x002B, 0},
	{0xE004, 0},
	{0x000E, 0},
	{0x000F, 0},
	{0x0010, 3},
	{0x0011, 232},
	{0x0012, 0},
	{0x0013, 0},
	{0x0014, 0},
	{0x0015, 0},
	{0xE004, 0},
	{0x0032, 5},
	{0x0033, 184},
	{0xE004, 0},
	{0x0007, 1},
	{0x0008, 0},
	{0x0009, 0},
	{0x000A, 65},
	{0x000B, 26},
	{0xE004, 0},
	{0x0024, 15},
	{0xE004, 0},
	{0x0031, 0},
	{0xE004, 0},
	{0x0026, 0},
	{0xE004, 0},
	{0x001C, 0},
	{0x0019, 0},
	{0x001A, 7},
	{0x001B, 83},
	{0x0016, 8},
	{0x0017, 0},
	{0x0018, 0},
	{0xE004, 0},
	{0x001D, 0},
	{0xE000, 0},
	{0xE000, 0},
	{0xE32C, 1},
	{0xE000, 0},
	{0xE32D, 1},
	{0xE000, 0},
	{0xE14E, 0},
	{0xE000, 0},
	{0xE312, 127},
	{0xE004, 0},
	{0x0000, 1},
	{0x001E, 0},
	{0x001F, 2},
	{0xE004, 1},
	{0x0000, 1},
	{0x001E, 0},
	{0x001F, 2},
	{0xE000, 0},
	{0xE329, 3},
	{0xE000, 0},
	{0xE32B, 0},
	{0xE000, 0},
	{0xE331, 4},
	{0xE000, 0},
	{0xE332, 8},
	{0xE000, 0},
	{0xE32A, 0},
	{0xE000, 0},
	{0xE11E, 1},
	{0xE000, 0},
	{0xE000, 0},
	{0xE062, 0},
	{0xE000, 0},
	{0xE056, 0},
	{0xE000, 0},
	{0xE060, 3},
	{0xE061, 232},
	{0xE000, 0},
	{0x0066, 0},
	{0x0067, 100},
	{0x0064, 1},
	{0x0064, 0},
	{0x0000, 0},
	{0x0000, 0},
	{0xE004, 0},
	{0xE000, 1},
	{0x000E, 0},
	{0x000F, 0},
	{0x0010, 3},
	{0x0011, 232},
	{0x0012, 0},
	{0x0013, 0},
	{0x0014, 0},
	{0x0015, 0},
	{0xE004, 1},
	{0x000E, 0},
	{0x000F, 0},
	{0x0010, 3},
	{0x0011, 232},
	{0x0012, 0},
	{0x0013, 0},
	{0x0014, 0},
	{0x0015, 0},
	{0xE004, 0},
	{0xE000, 1},
	{0x0007, 2},
	{0x0008, 0},
	{0x0009, 0},
	{0x000A, 65},
	{0x000B, 26},
	{0xE004, 1},
	{0x0007, 1},
	{0x0008, 0},
	{0x0009, 0},
	{0x000A, 65},
	{0x000B, 27},
	// Below are manually added after reg seq txt
	{0x0335,1} ,//iref sel
	{0x0324 , 43}, //iref val

	{0x1d9 , 1},// #vddana sel
	{0x0EB,15} ,//#vddana val trim

	{0x1dd ,1} ,//# vsspc sel
	{0x1ed , 20} ,//# vsspc val

	{0x1df,1} ,//#cp sel
	{0x0ee,4} ,//#cp trim,	

};

static const struct mira016_reg full_400_400_60fps_12b_1lane_reg_post_soft_reset[] = {
	// Release Soft Reset
	{57344,  0},
};


static const char * const mira016_test_pattern_menu[] = {
	"Disabled",
	"Fixed Data",
	"2D Gradient",
};

static const int mira016_test_pattern_val[] = {
	MIRA016_TEST_PATTERN_DISABLE,
	MIRA016_TEST_PATTERN_FIXED_DATA,
	MIRA016_TEST_PATTERN_2D_GRADIENT,
};


/* regulator supplies */
static const char * const mira016_supply_name[] = {
	// TODO(jalv): Check supply names
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.8V) supply */
	"VDDL",  /* IF (1.2V) supply */
};

#define MIRA016_NUM_SUPPLIES ARRAY_SIZE(mira016_supply_name)

/*
 * The supported formats. All flip/mirror combinations have the same byte order because the sensor
 * is monochrome
 */
static const u32 codes[] = {
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGRBG12_1X12,
};

/* Mode configs */
/*
 * Only one mode is exposed to the public (400x400 at 10 bit).
 * One code (10 bit) is exposed to public.
 * The public user specifies the code.
 * That is used to specify which internal supported_mode to use.
 */
#define MIRA016_SUPPORTED_MODE_SIZE_PUBLIC 1
static const struct mira016_mode supported_modes[] = {
	{
		/* 400x400 10bit 60fps mode */
		.width = 400,
		.height = 400,
		.crop = {
			.left = MIRA016_PIXEL_ARRAY_LEFT,
			.top = MIRA016_PIXEL_ARRAY_TOP,
			.width = 400,
			.height = 400
		},
		.reg_list_pre_soft_reset = {
			.num_of_regs = ARRAY_SIZE(full_400_400_60fps_10b_1lane_reg_pre_soft_reset),
			.regs = full_400_400_60fps_10b_1lane_reg_pre_soft_reset,
		},
		.reg_list_post_soft_reset = {
			.num_of_regs = ARRAY_SIZE(full_400_400_60fps_10b_1lane_reg_post_soft_reset),
			.regs = full_400_400_60fps_10b_1lane_reg_post_soft_reset,
		},
		.vblank = 2866,
		.hblank = MIRA016_HBLANK_60FPS, // TODO
		.bit_depth = 10,
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
	},
	{
		/* 400x400 12 bit 60fps mode */
		.width = 400,
		.height = 400,
		.crop = {
			.left = MIRA016_PIXEL_ARRAY_LEFT,
			.top = MIRA016_PIXEL_ARRAY_TOP,
			.width = 400,
			.height = 400
		},
		.reg_list_pre_soft_reset = {
			.num_of_regs = ARRAY_SIZE(full_400_400_60fps_12b_1lane_reg_pre_soft_reset),
			.regs = full_400_400_60fps_12b_1lane_reg_pre_soft_reset,
		},
		.reg_list_post_soft_reset = {
			.num_of_regs = ARRAY_SIZE(full_400_400_60fps_12b_1lane_reg_post_soft_reset),
			.regs = full_400_400_60fps_12b_1lane_reg_post_soft_reset,
		},
		.vblank = 2866,
		.hblank = MIRA016_HBLANK_60FPS, // TODO
		.bit_depth = 12,
		.code = MEDIA_BUS_FMT_SGRBG12_1X12,
	},

};

struct mira016 {
	struct v4l2_subdev sd;
	struct media_pad pad[NUM_PADS];

	struct v4l2_mbus_framefmt fmt;

	struct clk *xclk; /* system clock to MIRA016 */
	u32 xclk_freq;

	//struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[MIRA016_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *gain;
	// custom v4l2 control
	struct v4l2_ctrl *mira016_reg_w;
	struct v4l2_ctrl *mira016_reg_r;
	u16 mira016_reg_w_cached_addr;
	u8 mira016_reg_w_cached_flag;

	/* Current mode */
	const struct mira016_mode *mode;
	/* current bit depth, may defer from mode->bit_depth */
	u8 bit_depth;
	/* OTP_CALIBRATION_VALUE stored in OTP memory */
	u16 otp_cal_val;
	/* Whether to skip base register sequence upload */
	u32 skip_reg_upload;
	/* Whether to reset sensor when stream on/off */
	u32 skip_reset;


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

static inline struct mira016 *to_mira016(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct mira016, sd);
}

static int mira016_read(struct mira016 *mira016, u16 reg, u8 *val)
{
	int ret;
	unsigned char data_w[2] = { reg >> 8, reg & 0xff };
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);

	ret = i2c_master_send(client, data_w, 2);
	/*
	 * A negative return code, or sending the wrong number of bytes, both
	 * count as an error.
	 */
	if (ret != 2) {
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
	if (ret == 1) {
		ret = 0;
	} else {
		dev_dbg(&client->dev, "%s: i2c read error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
	}

	return ret;
}

static int mira016_write(struct mira016 *mira016, u16 reg, u8 val)
{
	int ret;
	unsigned char data[3] = { reg >> 8, reg & 0xff, val};
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);

	ret = i2c_master_send(client, data, 3);
	
	
	/*
	 * Writing the wrong number of bytes also needs to be flagged as an
	 * error. Success needs to produce a 0 return code.
	 */
	if (ret == 3) {
		ret = 0;
	} else {
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
		ret = mira016_read(mira016, ret_reg, &ret_val);
		printk(KERN_INFO "[MIRA016]: Write reg 0x%4.4x, Read ret_reg 0x%4.4x, val = 0x%x.\n",
				reg, ret_reg, ret_val);
		if (val != ret_val) {
			printk(KERN_INFO "[MIRA016]: WARNING Write reg 0x%4.4x, val = 0x%x, read ret_reg = 0x%4.4x, ret_val = 0x%x.\n",
				reg, val, ret_reg, ret_val);
		}
	}
	*/

	return ret;
}

/*
 * mira016 is big-endian: msb of val goes to lower reg addr
 */
/* Temporary commented out, because it is not yet used. */
/*
static int mira016_write_be16(struct mira016 *mira016, u16 reg, u16 val)
{
       int ret;
       unsigned char data[4] = { reg >> 8, reg & 0xff, (val >> 8) & 0xff, val & 0xff };
       struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);

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
*/

/*
 * mira016 is big-endian: msb of val goes to lower reg addr
 */
static int mira016_write_be32(struct mira016 *mira016, u16 reg, u32 val)
{
       int ret;
       unsigned char data[6] = { reg >> 8, reg & 0xff, (val >> 24) & 0xff, (val >> 16) & 0xff, (val >> 8) & 0xff, val & 0xff };
       struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);

       ret = i2c_master_send(client, data, 6);
       /*
        * Writing the wrong number of bytes also needs to be flagged as an
        * error. Success needs to produce a 0 return code.
        */
       if (ret == 6) {
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
 * mira016 OTP 32-bit val on I2C is big-endian. However, val content can be little-endian.
 */
static int mira016_read_be32(struct mira016 *mira016, u16 reg, u32 *val)
{
	int ret;
	unsigned char data_w[2] = { reg >> 8, reg & 0xff };
	/* Big-endian 32-bit buffer. */
	unsigned char data_r[4];
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);

	ret = i2c_master_send(client, data_w, 2);
	/*
	 * A negative return code, or sending the wrong number of bytes, both
	 * count as an error.
	 */
	if (ret != 2) {
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
	if (ret == 4) {
		ret = 0;
	} else {
		dev_dbg(&client->dev, "%s: i2c read error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
	}

	return ret;
}


/* Write a list of registers */
static int mira016_write_regs(struct mira016 *mira016,
			     const struct mira016_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = mira016_write(mira016, regs[i].address, regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    regs[i].address, ret);

			return ret;
		} else {
			// Debug code below
			// u8 val;
			// ret = mira016_read(mira016, regs[i].address, &val);
			// printk(KERN_INFO "[MIRA016]: Read reg 0x%4.4x, val = 0x%x.\n",
			// 		regs[i].address, val);
		}
	}

	return 0;
}

/*
 * Read OTP memory: 8-bit addr and 32-bit value
 */
static int mira016_otp_read(struct mira016 *mira016, u8 addr, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);
	u8 busy_status = 1;
	int poll_cnt = 0;
	int poll_cnt_max = 10;
	int ret;
	mira016_write(mira016, MIRA016_BANK_SEL_REG, 0);
	mira016_write(mira016, MIRA016_OTP_COMMAND, 0);
	mira016_write(mira016, MIRA016_OTP_ADDR, addr);
	mira016_write(mira016, MIRA016_OTP_START, 1);
	usleep_range(5, 10);
	mira016_write(mira016, MIRA016_OTP_START, 0);
	for (poll_cnt = 0; poll_cnt < poll_cnt_max; poll_cnt++) {
		mira016_read(mira016, MIRA016_OTP_BUSY, &busy_status);
		if (busy_status == 0) {
			break;
		}
	}
	if (poll_cnt < poll_cnt_max && busy_status == 0) {
		ret = mira016_read_be32(mira016, MIRA016_OTP_DOUT, val);
	} else {
		dev_dbg(&client->dev, "%s: OTP memory busy, skip raeding addr: 0x%X\n",
			__func__, addr);
		ret = -EINVAL;
	}

	return ret;
}


/* Write PMIC registers, and can be reused to write microcontroller reg. */
static int mira016pmic_write(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	unsigned char data[2] = { reg & 0xff, val};

	ret = i2c_master_send(client, data, 2);
	/*
	 * Writing the wrong number of bytes also needs to be flagged as an
	 * error. Success needs to produce a 0 return code.
	 */
	if (ret == 2) {
		ret = 0;
	} else {
		dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
	}

	return ret;
}

static int mira016_v4l2_reg_w(struct mira016 *mira016, u32 value) {
	struct i2c_client* const client = v4l2_get_subdevdata(&mira016->sd);
	u32 ret = 0;

	u16 reg_addr = (value >> 8) & 0xFFFF;
	u8 reg_val = value & 0xFF;
	u8 reg_flag = (value >> 24) & 0xFF;

	// printk(KERN_INFO "[MIRA016]: %s reg_flag: 0x%02X; reg_addr: 0x%04X; reg_val: 0x%02X.\n",
	// 		__func__, reg_flag, reg_addr, reg_val);

	if (reg_flag & AMS_CAMERA_CID_MIRA016_REG_FLAG_CMD_SEL) {
		if (reg_flag == AMS_CAMERA_CID_MIRA016_REG_FLAG_SLEEP_US) {
			// If it is for sleep, combine all 24 bits of reg_addr and reg_val as sleep us.
			u32 sleep_us_val = value & 0x00FFFFFF;
			// Sleep range needs an interval, default to 1/8 of the sleep value.
			u32 sleep_us_interval = sleep_us_val >> 3;
			printk(KERN_INFO "[MIRA016]: %s sleep_us: %u.\n", __func__, sleep_us_val);
			usleep_range(sleep_us_val, sleep_us_val + sleep_us_interval);
		} else if (reg_flag == AMS_CAMERA_CID_MIRA016_REG_FLAG_RESET_ON) {
			printk(KERN_INFO "[MIRA016]: %s Enable reset at stream on/off.\n", __func__);
			mira016->skip_reset = 0;
		} else if (reg_flag == AMS_CAMERA_CID_MIRA016_REG_FLAG_RESET_OFF) {
			printk(KERN_INFO "[MIRA016]: %s Disable reset at stream on/off.\n", __func__);
			mira016->skip_reset = 1;
		} else if (reg_flag == AMS_CAMERA_CID_MIRA016_REG_FLAG_REG_UP_ON) {
			printk(KERN_INFO "[MIRA016]: %s Enable base register sequence upload.\n", __func__);
			mira016->skip_reg_upload = 0;
		} else if (reg_flag == AMS_CAMERA_CID_MIRA016_REG_FLAG_REG_UP_OFF) {
			printk(KERN_INFO "[MIRA016]: %s Disable base register sequence upload.\n", __func__);
			mira016->skip_reg_upload = 1;
		} else {
			printk(KERN_INFO "[MIRA016]: %s unknown command from flag %u, ignored.\n", __func__, reg_flag);
		}
	} else if (reg_flag & AMS_CAMERA_CID_MIRA016_REG_FLAG_FOR_READ) {
		// If it is for read, skip reagister write, cache addr and flag for read.
		mira016->mira016_reg_w_cached_addr = reg_addr;
		mira016->mira016_reg_w_cached_flag = reg_flag;
	} else {
		// If it is for write, select which I2C device by the flag "I2C_SEL".
		if ((reg_flag & AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_MIRA) {
			// Before writing Mira016 register, first optionally select BANK and CONTEXT
			if (reg_flag & AMS_CAMERA_CID_MIRA016_REG_FLAG_USE_BANK){
				u8 bank;
				u8 context;
				// Set conetxt bank 0 or 1
				if (reg_flag & AMS_CAMERA_CID_MIRA016_REG_FLAG_BANK) {
					bank = 1;
				} else {
					bank = 0;
				}
				// printk(KERN_INFO "[MIRA016]: %s select bank: %u.\n", __func__, bank);
				ret = mira016_write(mira016, MIRA016_BANK_SEL_REG, bank);
				if (ret) {
					dev_err(&client->dev, "Error setting BANK_SEL_REG.");
					return ret;
				}
				// Set context bank 1A or bank 1B
				if (reg_flag & AMS_CAMERA_CID_MIRA016_REG_FLAG_CONTEXT) {
					context = 1;
				} else {
					context = 0;
				}
				// printk(KERN_INFO "[MIRA016]: %s select context: %u.\n", __func__, context);
				ret = mira016_write(mira016, MIRA016_RW_CONTEXT_REG, context);
				if (ret) {
					dev_err(&client->dev, "Error setting RW_CONTEXT.");
					return ret;
				}
			}
			// Writing the actual Mira016 register
			// printk(KERN_INFO "[MIRA016]: %s write reg_addr: 0x%04X; reg_val: 0x%02X.\n", __func__, reg_addr, reg_val);
			ret = mira016_write(mira016, reg_addr, reg_val);
			if (ret) {
				dev_err_ratelimited(&client->dev, "Error AMS_CAMERA_CID_MIRA_REG_W reg_addr %X.\n", reg_addr);
				return -EINVAL;
			}
		} else if ((reg_flag & AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_SET_TBD) {
			/* User tries to set TBD I2C address, store reg_val to mira016->tbd_client_i2c_addr. Skip write. */
			printk(KERN_INFO "[MIRA016]: mira016->tbd_client_i2c_addr = 0x%X.\n", reg_val);
			mira016->tbd_client_i2c_addr = reg_val;
		} else if ((reg_flag & AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_TBD) {
			if (mira016->tbd_client_i2c_addr == MIRA016PMIC_I2C_ADDR) {
				// Write PMIC. Use pre-allocated mira016->pmic_client.
				printk(KERN_INFO "[MIRA016]: write pmic_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
				ret = mira016pmic_write(mira016->pmic_client, (u8)(reg_addr & 0xFF), reg_val);
			} else if (mira016->tbd_client_i2c_addr == MIRA016UC_I2C_ADDR) {
				// Write micro-controller. Use pre-allocated mira016->uc_client.
				printk(KERN_INFO "[MIRA016]: write uc_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
				ret = mira016pmic_write(mira016->uc_client, (u8)(reg_addr & 0xFF), reg_val);
			} else if (mira016->tbd_client_i2c_addr == MIRA016LED_I2C_ADDR) {
				// Write LED driver. Use pre-allocated mira016->led_client.
				printk(KERN_INFO "[MIRA016]: write led_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
				ret = mira016pmic_write(mira016->led_client, (u8)(reg_addr & 0xFF), reg_val);
			} else {
				/* Write other TBD I2C address.
				 * The TBD I2C address is set via AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_SET_TBD.
				 * The TBD I2C address is stored in mira016->tbd_client_i2c_addr.
				 * A temporary I2C client, tmp_client, is created and then destroyed (unregistered).
				 */
				struct i2c_client *tmp_client;
				tmp_client = i2c_new_dummy_device(client->adapter, mira016->tbd_client_i2c_addr);
				if (IS_ERR(tmp_client))
					return PTR_ERR(tmp_client);
				printk(KERN_INFO "[MIRA016]: write tbd_client, i2c_addr %u, reg_addr 0x%X, reg_val 0x%X.\n",
						mira016->tbd_client_i2c_addr, (u8)(reg_addr & 0xFF), reg_val);
				ret = mira016pmic_write(tmp_client, (u8)(reg_addr & 0xFF), reg_val);
				i2c_unregister_device(tmp_client);
			}
		}
	}

	return 0;
}

static int mira016_v4l2_reg_r(struct mira016 *mira016, u32 *value) {
	struct i2c_client* const client = v4l2_get_subdevdata(&mira016->sd);
	u32 ret = 0;

	u16 reg_addr = mira016->mira016_reg_w_cached_addr;
	u8 reg_flag = mira016->mira016_reg_w_cached_flag;
	u8 reg_val = 0;

	*value = 0;

	if (reg_flag & AMS_CAMERA_CID_MIRA016_REG_FLAG_USE_BANK){
		u8 bank;
		u8 context;
		// Set conetxt bank 0 or 1
		if (reg_flag & AMS_CAMERA_CID_MIRA016_REG_FLAG_BANK) {
			bank = 1;
		} else {
			bank = 0;
		}
		// printk(KERN_INFO "[MIRA016]: %s select bank: %u.\n", __func__, bank);
		ret = mira016_write(mira016, MIRA016_BANK_SEL_REG, bank);
		if (ret) {
			dev_err(&client->dev, "Error setting BANK_SEL_REG.");
			return ret;
		}
		// Set context bank 1A or bank 1B
		if (reg_flag & AMS_CAMERA_CID_MIRA016_REG_FLAG_CONTEXT) {
			context = 1;
		} else {
			context = 0;
		}
		// printk(KERN_INFO "[MIRA016]: %s select context: %u.\n", __func__, context);
		ret = mira016_write(mira016, MIRA016_RW_CONTEXT_REG, context);
		if (ret) {
			dev_err(&client->dev, "Error setting RW_CONTEXT.");
			return ret;
		}
	}
	ret = mira016_read(mira016, reg_addr, &reg_val);
	if (ret) {
		dev_err_ratelimited(&client->dev, "Error AMS_CAMERA_CID_MIRA_REG_R reg_addr %X.\n", reg_addr);
		return -EINVAL;
	}
	// Return 32-bit value that includes flags, addr, and register value
	*value = ((u32)reg_flag << 24) | ((u32)reg_addr << 8) | (u32)reg_val;

	// printk(KERN_INFO "[MIRA016]: mira016_v4l2_reg_r() reg_flag: 0x%02X; reg_addr: 0x%04X, reg_val: 0x%02X.\n",
	// 		reg_flag, reg_addr, reg_val);

	return 0;
}


/* Write a list of v4l2 registers */
static int mira016_write_v4l2_regs(struct mira016 *mira016,
				const struct mira016_v4l2_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = mira016_v4l2_reg_w(mira016, regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "Failed to write v4l2 reg value 0x%8.8x. error = %d\n",
					    regs[i].val, ret);
			return ret;
		}
	}

	return 0;
}

// Returns the maximum exposure time in microseconds (reg value)
static u32 mira016_calculate_max_exposure_time(u32 row_length, u32 vsize,
					       u32 vblank) {
	(void)(row_length);
	(void)(vsize);
	(void)(vblank);
	/* Mira016 does not have a max exposure limit besides register bits */
	// return row_length * (vsize + vblank) - MIRA016_GLOB_NUM_CLK_CYCLES;
	return 	MIRA016_EXPOSURE_MAX_US;
}

static int mira016_write_exposure_reg(struct mira016 *mira016, u32 exposure) {
	struct i2c_client* const client = v4l2_get_subdevdata(&mira016->sd);
	const u32 min_exposure = MIRA016_EXPOSURE_MIN_US;
	u32 max_exposure = mira016->exposure->maximum * MIRA016_MIN_ROW_LENGTH_US;
	u32 ret = 0;

	if (exposure < min_exposure) {
		exposure = min_exposure;
	}
	if (exposure > max_exposure) {
		exposure = max_exposure;
	}

	/* Write Bank 1 context 0 */
	ret = mira016_write(mira016, MIRA016_RW_CONTEXT_REG, 0);
	ret = mira016_write(mira016, MIRA016_BANK_SEL_REG, 1);
	ret = mira016_write_be32(mira016, MIRA016_EXP_TIME_L_REG, exposure);
	/* Write Bank 1 context 1 */
	ret = mira016_write(mira016, MIRA016_RW_CONTEXT_REG, 1);
	ret = mira016_write_be32(mira016, MIRA016_EXP_TIME_L_REG, exposure);
	if (ret) {
		dev_err_ratelimited(&client->dev, "Error setting exposure time to %d", exposure);
		return -EINVAL;
	}

	return 0;
}

static int mira016_write_start_streaming_regs(struct mira016* mira016) {
	struct i2c_client* const client = v4l2_get_subdevdata(&mira016->sd);
	int ret = 0;

	// Set conetxt bank 0 or 1
	ret = mira016_write(mira016, MIRA016_BANK_SEL_REG, 0);
	if (ret) {
		dev_err(&client->dev, "Error setting BANK_SEL_REG.");
		return ret;
	}

	// Set context bank 1A or bank 1B
	ret = mira016_write(mira016, MIRA016_RW_CONTEXT_REG, 0);
	if (ret) {
		dev_err(&client->dev, "Error setting RW_CONTEXT.");
		return ret;
	}

	// Raising CMD_REQ_1 to 1 for REQ_EXP
	ret = mira016_write(mira016, MIRA016_CMD_REQ_1_REG,
				1);
	if (ret) {
		dev_err(&client->dev, "Error setting CMD_REQ_1 to 1 for REQ_EXP.");
		return ret;
	}
	
	usleep_range(10, 20);

	// Setting CMD_REQ_1 tp 0 for REQ_EXP
	ret = mira016_write(mira016, MIRA016_CMD_REQ_1_REG,
				0);
	if (ret) {
		dev_err(&client->dev, "Error setting CMD_REQ_1 to 0 for REQ_EXP.");
		return ret;
	}
	usleep_range(10, 20);

	return ret;
}


static int mira016_write_stop_streaming_regs(struct mira016* mira016) {
	struct i2c_client* const client = v4l2_get_subdevdata(&mira016->sd);
	int ret = 0;

	// Set conetxt bank 0 or 1
	ret = mira016_write(mira016, MIRA016_BANK_SEL_REG, 0);
	if (ret) {
		dev_err(&client->dev, "Error setting BANK_SEL_REG.");
		return ret;
	}

	// Raising CMD_HALT_BLOCK to 1 to stop streaming
	ret = mira016_write(mira016, MIRA016_CMD_HALT_BLOCK_REG,
				1);
	if (ret) {
		dev_err(&client->dev, "Error setting CMD_HALT_BLOCK to 1.");
		return ret;
	}

	usleep_range(10, 20);

	// Setting CMD_HALT_BLOCK to 0 to stop streaming
	ret = mira016_write(mira016, MIRA016_CMD_HALT_BLOCK_REG,
				0);
	if (ret) {
		dev_err(&client->dev, "Error setting CMD_HALT_BLOCK to 0.");
		return ret;
	}
	usleep_range(10, 20);

        /*
         * Wait for one frame to make sure sensor is set to
         * software standby in V-blank
         *
         * frame_time = frame length rows * Tline
         * Tline = line length / pixel clock (in MHz)
         */
	/*
	u32 frame_time;
        frame_time = MIRA016_DEFAULT_FRAME_LENGTH *
            MIRA016_DEFAULT_LINE_LENGTH / MIRA016_DEFAULT_PIXEL_CLOCK;

        usleep_range(frame_time, frame_time + 1000);
	*/

	return ret;
}

// Gets the format code if supported. Otherwise returns the default format code `codes[0]`
static u32 mira016_validate_format_code_or_default(struct mira016 *mira016, u32 code)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);
	unsigned int i;

	lockdep_assert_held(&mira016->mutex);

	for (i = 0; i < ARRAY_SIZE(codes); i++)
		if (codes[i] == code)
			break;

	if (i >= ARRAY_SIZE(codes)) {
		dev_err_ratelimited(&client->dev, "Could not set requested format code %u", code);
		dev_err_ratelimited(&client->dev, "Using default format %u", codes[0]);
		i = 0;
	}

	return codes[i];
}

static void mira016_set_default_format(struct mira016 *mira016)
{
	struct v4l2_mbus_framefmt *fmt;

	fmt = &mira016->fmt;
	fmt->code = MEDIA_BUS_FMT_SGRBG10_1X10; // MEDIA_BUS_FMT_Y10_1X10;
	mira016->bit_depth = 10;
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

static int mira016_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct mira016 *mira016 = to_mira016(sd);
	struct v4l2_mbus_framefmt *try_fmt_img =
		v4l2_subdev_get_try_format(sd, fh->state, IMAGE_PAD);
	struct v4l2_mbus_framefmt *try_fmt_meta =
		v4l2_subdev_get_try_format(sd, fh->state, METADATA_PAD);
	struct v4l2_rect *try_crop;

	mutex_lock(&mira016->mutex);

	/* Initialize try_fmt for the image pad */
	try_fmt_img->width = supported_modes[0].width;
	try_fmt_img->height = supported_modes[0].height;
	try_fmt_img->code = mira016_validate_format_code_or_default(mira016,
						   MEDIA_BUS_FMT_SGRBG10_1X10);
	try_fmt_img->field = V4L2_FIELD_NONE;

	/* TODO(jalv): Initialize try_fmt for the embedded metadata pad */
	try_fmt_meta->width = MIRA016_EMBEDDED_LINE_WIDTH;
	try_fmt_meta->height = MIRA016_NUM_EMBEDDED_LINES;
	try_fmt_meta->code = MEDIA_BUS_FMT_SENSOR_DATA;
	try_fmt_meta->field = V4L2_FIELD_NONE;



	/* Initialize try_crop rectangle. */
	try_crop = v4l2_subdev_get_try_crop(sd, fh->state, 0);
	try_crop->top = MIRA016_PIXEL_ARRAY_TOP;
	try_crop->left = MIRA016_PIXEL_ARRAY_LEFT;
	try_crop->width = MIRA016_PIXEL_ARRAY_WIDTH;
	try_crop->height = MIRA016_PIXEL_ARRAY_HEIGHT;

	mutex_unlock(&mira016->mutex);

	return 0;
}

static int mira016_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mira016 *mira016 =
		container_of(ctrl->handler, struct mira016, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);
	int ret = 0;

	// Debug print
	// printk(KERN_INFO "[MIRA016]: mira016_set_ctrl() id: 0x%X value: 0x%X.\n", ctrl->id, ctrl->val);

	if (ctrl->id == V4L2_CID_VBLANK) {
		int exposure_max, exposure_def;

		/* Update max exposure while meeting expected vblanking */
		exposure_max = mira016_calculate_max_exposure_time(MIRA016_MIN_ROW_LENGTH,
						   mira016->mode->height,
						   ctrl->val);
		exposure_def = (exposure_max < MIRA016_DEFAULT_EXPOSURE_US) ?
			exposure_max : MIRA016_DEFAULT_EXPOSURE_US;
		__v4l2_ctrl_modify_range(mira016->exposure,
					 mira016->exposure->minimum,
					 (int)(1 + exposure_max / MIRA016_MIN_ROW_LENGTH_US), mira016->exposure->step,
					 (int)(1 + exposure_def / MIRA016_MIN_ROW_LENGTH_US));
	}


	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(&client->dev) == 0) {
		dev_info(&client->dev,
                         "device in use, ctrl(id:0x%x,val:0x%x) is not handled\n",
                         ctrl->id, ctrl->val);
		return 0;
	}

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		break;
	case V4L2_CID_EXPOSURE:
		ret = mira016_write_exposure_reg(mira016, ctrl->val * MIRA016_MIN_ROW_LENGTH_US);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = mira016_write(mira016, MIRA016_BANK_SEL_REG, 0);
		// Fixed data is hard coded to 0xAB.
		ret = mira016_write(mira016, MIRA016_TRAINING_WORD_REG, 0xAB);
		// Gradient is hard coded to 45 degree.
		ret = mira016_write(mira016, MIRA016_DELTA_TEST_IMG_REG, 0x01);
		ret = mira016_write(mira016, MIRA016_TEST_PATTERN_REG,
				        mira016_test_pattern_val[ctrl->val]);
		break;
	case V4L2_CID_HFLIP:
		// TODO: HFLIP requires multiple register writes
		//ret = mira016_write(mira016, MIRA016_HFLIP_REG,
		//		        ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		// TODO: VFLIP seems not supported in Mira016
		//ret = mira016_write(mira016, MIRA016_VFLIP_REG,
		//		        ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		// TODO: check whether blanking control is supported in Mira016
		//ret = mira016_write_be16(mira016, MIRA016_VBLANK_LO_REG,
		//		        mira016->mode->height + ctrl->val);
		break;
	case V4L2_CID_HBLANK:
		break;
	default:
		dev_info(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	// TODO: FIXIT
	return ret;
}

static int mira016_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mira016 *mira016 =
		container_of(ctrl->handler, struct mira016, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);
	int ret = 0;

	// printk(KERN_INFO "[MIRA016]: mira016_s_ctrl() id: %X value: %X.\n", ctrl->id, ctrl->val);

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(&client->dev) == 0) {
		struct mira016_v4l2_reg_list *reg_list;
		reg_list = &reg_list_s_ctrl_mira016_reg_w_buf;
		if (ctrl->id == AMS_CAMERA_CID_MIRA_REG_W &&
		    reg_list->num_of_regs < AMS_CAMERA_CID_MIRA016_REG_W_BUF_SIZE) {
			int buf_idx = reg_list->num_of_regs;
			u32 value = ctrl->val;
			reg_list->regs[buf_idx].val = value;
			reg_list->num_of_regs++;
		}
		// Below is optional warning
		// dev_info(&client->dev,
                //         "device in use, ctrl(id:0x%x,val:0x%x) is not handled\n",
                //         ctrl->id, ctrl->val);
		return 0;
	}

	switch (ctrl->id) {
	case AMS_CAMERA_CID_MIRA_REG_W:
		ret = mira016_v4l2_reg_w(mira016, ctrl->val);
		break;
	default:
		dev_info(&client->dev,
			 "set ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	// TODO: FIXIT
	return ret;
}

static int mira016_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mira016 *mira016 =
		container_of(ctrl->handler, struct mira016, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);
	int ret = 0;

	// printk(KERN_INFO "[MIRA016]: mira016_g_ctrl() id: %X.\n", ctrl->id);

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(&client->dev) == 0) {
		dev_info(&client->dev,
                        "device in use, ctrl(id:0x%x) is not handled\n",
                        ctrl->id);
		return 0;
	}

	switch (ctrl->id) {
	case AMS_CAMERA_CID_MIRA_REG_R:
		ret = mira016_v4l2_reg_r(mira016, (u32 *)&ctrl->cur.val);
		ctrl->val = ctrl->cur.val;
		break;
	default:
		dev_info(&client->dev,
			 "get ctrl(id:0x%x) is not handled\n",
			 ctrl->id);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	// TODO: FIXIT
	return ret;
}


static const struct v4l2_ctrl_ops mira016_ctrl_ops = {
	.s_ctrl = mira016_set_ctrl,
};

static const struct v4l2_ctrl_ops mira016_custom_ctrl_ops = {
	.g_volatile_ctrl = mira016_g_ctrl,
	.s_ctrl = mira016_s_ctrl,
};


/* list of custom v4l2 ctls */
static struct v4l2_ctrl_config custom_ctrl_config_list[] = {
	/* Do not change the name field for the controls! */
	{
		.ops = &mira016_custom_ctrl_ops,
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
		.ops = &mira016_custom_ctrl_ops,
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
static int mira016_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct mira016 *mira016 = to_mira016(sd);

	if (code->pad >= NUM_PADS)
		return -EINVAL;

	if (code->pad == IMAGE_PAD) {
		if (code->index >= ARRAY_SIZE(codes))
			return -EINVAL;

		code->code = mira016_validate_format_code_or_default(mira016,
						    codes[code->index]);
	} else {
		if (code->index > 0)
			return -EINVAL;

		code->code = MEDIA_BUS_FMT_SENSOR_DATA;
	}

	return 0;
}

static int mira016_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct mira016 *mira016 = to_mira016(sd);

	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	if (fse->pad == IMAGE_PAD) {
		/* Two options about how many modes to be exposed:
		 * - Expose all supported_modes by ARRAY_SIZE(supported_modes).
		 * - Expose less modes by MIRA016_SUPPORTED_MODE_SIZE_PUBLIC.
		 */
		/* if (fse->index >= ARRAY_SIZE(supported_modes)) */
		if (fse->index >= MIRA016_SUPPORTED_MODE_SIZE_PUBLIC)
			return -EINVAL;

		if (fse->code != mira016_validate_format_code_or_default(mira016, fse->code))
			return -EINVAL;

		fse->min_width = supported_modes[fse->index].width;
		fse->max_width = fse->min_width;
		fse->min_height = supported_modes[fse->index].height;
		fse->max_height = fse->min_height;
	} else {
		if (fse->code != MEDIA_BUS_FMT_SENSOR_DATA || fse->index > 0)
			return -EINVAL;

		fse->min_width = MIRA016_EMBEDDED_LINE_WIDTH;
		fse->max_width = fse->min_width;
		fse->min_height = MIRA016_NUM_EMBEDDED_LINES;
		fse->max_height = fse->min_height;
	}

	return 0;
}

static void mira016_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void mira016_update_image_pad_format(struct mira016 *mira016,
					   const struct mira016_mode *mode,
					   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	mira016_reset_colorspace(&fmt->format);
}

static void mira016_update_metadata_pad_format(struct v4l2_subdev_format *fmt)
{
	fmt->format.width = MIRA016_EMBEDDED_LINE_WIDTH;
	fmt->format.height = MIRA016_NUM_EMBEDDED_LINES;
	fmt->format.code = MEDIA_BUS_FMT_SENSOR_DATA;
	fmt->format.field = V4L2_FIELD_NONE;

}

static int __mira016_get_pad_format(struct mira016 *mira016,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_format *fmt)
{
	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(&mira016->sd, sd_state, fmt->pad);

		try_fmt->code = fmt->pad == IMAGE_PAD ?
				mira016_validate_format_code_or_default(mira016, try_fmt->code) :
				MEDIA_BUS_FMT_SENSOR_DATA;
		fmt->format = *try_fmt;
	} else {
		if (fmt->pad == IMAGE_PAD) {
			mira016_update_image_pad_format(mira016, mira016->mode,
						       fmt);
			fmt->format.code = mira016_validate_format_code_or_default(mira016,
							      mira016->fmt.code);
		} else {
			mira016_update_metadata_pad_format(fmt);
		}
	}

	return 0;
}

static int mira016_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct mira016 *mira016 = to_mira016(sd);
	int ret;

	mutex_lock(&mira016->mutex);
	ret = __mira016_get_pad_format(mira016, sd_state, fmt);
	mutex_unlock(&mira016->mutex);

	return ret;
}

static int mira016_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mira016 *mira016 = to_mira016(sd);
	const struct mira016_mode *mode;
	struct v4l2_mbus_framefmt *framefmt;
	u32 max_exposure = 0, default_exp = 0;
	int rc = 0;

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&mira016->mutex);

	if (fmt->pad == IMAGE_PAD) {
		/* Validate format or use default */
		fmt->format.code = mira016_validate_format_code_or_default(mira016,
									  fmt->format.code);

		mode = v4l2_find_nearest_size(supported_modes,
					      ARRAY_SIZE(supported_modes),
					      width, height,
					      fmt->format.width,
					      fmt->format.height);
		mira016_update_image_pad_format(mira016, mode, fmt);
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else if (mira016->mode != mode ||
			mira016->fmt.code != fmt->format.code) {
			mira016->fmt = fmt->format;
			mira016->mode = mode;

			// Update controls based on new mode (range and current value).
			max_exposure = mira016_calculate_max_exposure_time(MIRA016_MIN_ROW_LENGTH,
									   mira016->mode->height,
									   mira016->mode->vblank);
			default_exp = MIRA016_DEFAULT_EXPOSURE_US > max_exposure ? max_exposure : MIRA016_DEFAULT_EXPOSURE_US;
			rc = __v4l2_ctrl_modify_range(mira016->exposure,
						     mira016->exposure->minimum,
						     (int)( 1 + max_exposure / MIRA016_MIN_ROW_LENGTH_US), mira016->exposure->step,
						     (int)( 1 + default_exp / MIRA016_MIN_ROW_LENGTH_US));
			if (rc) {
				dev_err(&client->dev, "Error setting exposure range");
			}

			// Set the current vblank value
			rc = __v4l2_ctrl_s_ctrl(mira016->vblank, mira016->mode->vblank);
			if (rc) {
				dev_err(&client->dev, "Error setting vblank value to %u",
					mira016->mode->vblank);
			}
		}
	} else {
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else {
			/* Only one embedded data mode is supported */
			mira016_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&mira016->mutex);

	return 0;
}

static int mira016_set_framefmt(struct mira016 *mira016)
{
	// TODO: There is no easy way to change frame format
	switch (mira016->fmt.code) {
	case MEDIA_BUS_FMT_SGRBG10_1X10:
		printk(KERN_INFO "[MIRA016]: mira016_set_framefmt() selects 10 bit mode.\n");
		mira016->mode = &supported_modes[0];
		mira016->bit_depth = 10;
		return 0;
	case MEDIA_BUS_FMT_SGRBG12_1X12:
		printk(KERN_INFO "[MIRA016]: mira016_set_framefmt() selects 12 bit mode.\n");
		mira016->mode = &supported_modes[1];
		mira016->bit_depth = 12;
		return 0;

	default:
		printk(KERN_ERR "Unknown format requested %d", mira016->fmt.code);
	}

	return -EINVAL;
}

static const struct v4l2_rect *
__mira016_get_pad_crop(struct mira016 *mira016, struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&mira016->sd, sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &mira016->mode->crop;
	}

	return NULL;
}

static int mira016_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct mira016 *mira016 = to_mira016(sd);

		mutex_lock(&mira016->mutex);
		sel->r = *__mira016_get_pad_crop(mira016, sd_state, sel->pad,
						sel->which);
		mutex_unlock(&mira016->mutex);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = MIRA016_NATIVE_WIDTH;
		sel->r.height = MIRA016_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = MIRA016_PIXEL_ARRAY_TOP;
		sel->r.left = MIRA016_PIXEL_ARRAY_LEFT;
		sel->r.width = MIRA016_PIXEL_ARRAY_WIDTH;
		sel->r.height = MIRA016_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

static int mira016_start_streaming(struct mira016 *mira016)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);
	const struct mira016_reg_list *reg_list;
	const struct mira016_v4l2_reg_list *reg_v4l2_list;
	u32 otp_cal_val;
	int ret;

	printk(KERN_INFO "[MIRA016]: Entering start streaming function.\n");

	ret = pm_runtime_get_sync(&client->dev);

	if (ret < 0) {
		printk(KERN_INFO "[MIRA016]: get_sync failed, but continue.\n");
		pm_runtime_put_noidle(&client->dev);
		return ret;
	}

	/* Set current mode according to frame format bit depth */
	ret = mira016_set_framefmt(mira016);
	if (ret) {
		dev_err(&client->dev, "%s failed to set frame format: %d\n",
			__func__, ret);
		goto err_rpm_put;
	}
	printk(KERN_INFO "[MIRA016]: Register sequence for %d bit mode will be used.\n", mira016->mode->bit_depth);

	if (mira016->skip_reg_upload == 0) {
		/* Apply pre soft reset default values of current mode */
		reg_list = &mira016->mode->reg_list_pre_soft_reset;
		printk(KERN_INFO "[MIRA016]: Write %d regs.\n", reg_list->num_of_regs);
		ret = mira016_write_regs(mira016, reg_list->regs, reg_list->num_of_regs);
		if (ret) {
			dev_err(&client->dev, "%s failed to set mode\n", __func__);
			goto err_rpm_put;
		}

		usleep_range(10, 50);

		/* Apply post soft reset default values of current mode */
		reg_list = &mira016->mode->reg_list_post_soft_reset;
		printk(KERN_INFO "[MIRA016]: Write %d regs.\n", reg_list->num_of_regs);
		ret = mira016_write_regs(mira016, reg_list->regs, reg_list->num_of_regs);
		if (ret) {
			dev_err(&client->dev, "%s failed to set mode\n", __func__);
			goto err_rpm_put;
		}
	} else {
		printk(KERN_INFO "[MIRA016]: Skip base register sequence upload, due to mira016->skip_reg_upload=%u.\n", mira016->skip_reg_upload);
	}

	printk(KERN_INFO "[MIRA016]: Entering v4l2 ctrl handler setup function.\n");

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(mira016->sd.ctrl_handler);
	printk(KERN_INFO "[MIRA016]: __v4l2_ctrl_handler_setup ret = %d.\n", ret);
	if (ret)
		goto err_rpm_put;


	reg_v4l2_list = &reg_list_s_ctrl_mira016_reg_w_buf;
	printk(KERN_INFO "[MIRA016]: Writing %d regs from AMS_CAMERA_CID_MIRA_REG_W.\n", reg_v4l2_list->num_of_regs);
	ret = mira016_write_v4l2_regs(mira016, reg_v4l2_list->regs, reg_v4l2_list->num_of_regs);
        if (ret) {
                dev_err(&client->dev, "%s failed to set mode\n", __func__);
                goto err_rpm_put;
        }
	reg_list_s_ctrl_mira016_reg_w_buf.num_of_regs = 0;

	/* Read OTP memory for OTP_CALIBRATION_VALUE */
	ret = mira016_otp_read(mira016, 0x01, &otp_cal_val);
	/* OTP_CALIBRATION_VALUE is little-endian, LSB at [7:0], MSB at [15:8] */
	mira016->otp_cal_val = (u16)(otp_cal_val & 0x0000FFFF);
	if (ret) {
		dev_err(&client->dev, "%s failed to read OTP addr 0x01.\n", __func__);
		/* Even if OTP reading fails, continue with the rest. */
		/* goto err_rpm_put; */
	} else {
		printk(KERN_INFO "[MIRA016]: OTP_CALIBRATION_VALUE: %u, extracted from 32-bit 0x%X.\n", mira016->otp_cal_val, otp_cal_val);
	}

	printk(KERN_INFO "[MIRA016]: Writing start streaming regs.\n");

	ret = mira016_write_start_streaming_regs(mira016);
	if (ret) {
		dev_err(&client->dev, "Could not write stream-on sequence");
		goto err_rpm_put;
	}

	/* vflip and hflip cannot change during streaming */
	printk(KERN_INFO "[MIRA016]: Entering v4l2 ctrl grab vflip grab vflip.\n");
	__v4l2_ctrl_grab(mira016->vflip, true);
	printk(KERN_INFO "[MIRA016]: Entering v4l2 ctrl grab vflip grab hflip.\n");
	__v4l2_ctrl_grab(mira016->hflip, true);

	return 0;

err_rpm_put:
	pm_runtime_put(&client->dev);
	return ret;
}

static void mira016_stop_streaming(struct mira016 *mira016)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);
	int ret = 0;

	ret = mira016_write_stop_streaming_regs(mira016);
	if (ret) {
		dev_err(&client->dev, "Could not write the stream-off sequence");
	}

	/* Unlock controls for vflip and hflip */
	__v4l2_ctrl_grab(mira016->vflip, false);
	__v4l2_ctrl_grab(mira016->hflip, false);

	pm_runtime_put(&client->dev);
}

static int mira016_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct mira016 *mira016 = to_mira016(sd);
	int ret = 0;

	mutex_lock(&mira016->mutex);
	if (mira016->streaming == enable) {
		mutex_unlock(&mira016->mutex);
		return 0;
	}

	printk(KERN_INFO "[MIRA016]: Entering mira016_set_stream enable: %d.\n", enable);

	if (enable) {
		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = mira016_start_streaming(mira016);
		if (ret)
			goto err_unlock;
	} else {
		mira016_stop_streaming(mira016);
	}

	mira016->streaming = enable;

	mutex_unlock(&mira016->mutex);

	printk(KERN_INFO "[MIRA016]: Returning mira016_set_stream with ret: %d.\n", ret);

	return ret;

err_unlock:
	mutex_unlock(&mira016->mutex);

	return ret;
}

/* Power/clock management functions */
static int mira016_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira016 *mira016 = to_mira016(sd);
	int ret = -EINVAL;

	printk(KERN_INFO "[MIRA016]: Entering power on function.\n");

	/* Alway enable regulator even if (skip_reset == 1) */
	ret = regulator_bulk_enable(MIRA016_NUM_SUPPLIES, mira016->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(mira016->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}

	usleep_range(MIRA016_XCLR_MIN_DELAY_US,
		     MIRA016_XCLR_MIN_DELAY_US + MIRA016_XCLR_DELAY_RANGE_US);

	return 0;

reg_off:
	ret = regulator_bulk_disable(MIRA016_NUM_SUPPLIES, mira016->supplies);
	return ret;
}

static int mira016_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira016 *mira016 = to_mira016(sd);

	printk(KERN_INFO "[MIRA016]: Entering power off function.\n");

	if (mira016->skip_reset == 0) {
		regulator_bulk_disable(MIRA016_NUM_SUPPLIES, mira016->supplies);
		clk_disable_unprepare(mira016->xclk);
	} else {
		printk(KERN_INFO "[MIRA016]: Skip disabling regulator at power on due to mira016->skip_reset=%u.\n", mira016->skip_reset);
	}

	return 0;
}

static int __maybe_unused mira016_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira016 *mira016 = to_mira016(sd);

	printk(KERN_INFO "[MIRA016]: Entering suspend function.\n");

	if (mira016->streaming)
		mira016_stop_streaming(mira016);

	return 0;
}

static int __maybe_unused mira016_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira016 *mira016 = to_mira016(sd);
	int ret;

	printk(KERN_INFO "[MIRA016]: Entering resume function.\n");

	if (mira016->streaming) {
		ret = mira016_start_streaming(mira016);
		if (ret)
			goto error;
	}

	return 0;

error:
	mira016_stop_streaming(mira016);
	mira016->streaming = false;

	return ret;
}

static int mira016_get_regulators(struct mira016 *mira016)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);
	unsigned int i;

	for (i = 0; i < MIRA016_NUM_SUPPLIES; i++)
		mira016->supplies[i].supply = mira016_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       MIRA016_NUM_SUPPLIES,
				       mira016->supplies);
}

/* Verify chip ID */
static int mira016_identify_module(struct mira016 *mira016)
{
	int ret;
	u8 val;

	ret = mira016_read(mira016, 0x25, &val);
	printk(KERN_INFO "[MIRA016]: Read reg 0x%4.4x, val = 0x%x.\n",
		      0x25, val);
	ret = mira016_read(mira016, 0x3, &val);
	printk(KERN_INFO "[MIRA016]: Read reg 0x%4.4x, val = 0x%x.\n",
		      0x3, val);
	ret = mira016_read(mira016, 0x4, &val);
	printk(KERN_INFO "[MIRA016]: Read reg 0x%4.4x, val = 0x%x.\n",
		      0x4, val);

	return 0;
}

static const struct v4l2_subdev_core_ops mira016_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops mira016_video_ops = {
	.s_stream = mira016_set_stream,
};

static const struct v4l2_subdev_pad_ops mira016_pad_ops = {
	.enum_mbus_code = mira016_enum_mbus_code,
	.get_fmt = mira016_get_pad_format,
	.set_fmt = mira016_set_pad_format,
	.get_selection = mira016_get_selection,
	.enum_frame_size = mira016_enum_frame_size,
};

static const struct v4l2_subdev_ops mira016_subdev_ops = {
	.core = &mira016_core_ops,
	.video = &mira016_video_ops,
	.pad = &mira016_pad_ops,
};

static const struct v4l2_subdev_internal_ops mira016_internal_ops = {
	.open = mira016_open,
};

/* Initialize control handlers */
static int mira016_init_controls(struct mira016 *mira016)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct v4l2_fwnode_device_properties props;
	int ret;
	struct v4l2_ctrl_config *mira016_reg_w;
	struct v4l2_ctrl_config *mira016_reg_r;

	ctrl_hdlr = &mira016->ctrl_handler;
	/* v4l2_ctrl_handler_init gives a hint/guess of the number of v4l2_ctrl_new */
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 16);
	if (ret)
		return ret;

	mutex_init(&mira016->mutex);
	ctrl_hdlr->lock = &mira016->mutex;

	printk(KERN_INFO "[MIRA016]: %s V4L2_CID_PIXEL_RATE %X.\n", __func__, V4L2_CID_PIXEL_RATE);

	/* By default, PIXEL_RATE is read only */
	mira016->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &mira016_ctrl_ops,
					        V4L2_CID_PIXEL_RATE,
					        MIRA016_PIXEL_RATE,
					        MIRA016_PIXEL_RATE, 1,
					        MIRA016_PIXEL_RATE);

	printk(KERN_INFO "[MIRA016]: %s V4L2_CID_VBLANK %X.\n", __func__, V4L2_CID_VBLANK);

	mira016->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &mira016_ctrl_ops,
					   V4L2_CID_VBLANK, MIRA016_MIN_VBLANK,
					   0xFFFF, 1,
					   mira016->mode->vblank);

	printk(KERN_INFO "[MIRA016]: %s V4L2_CID_HBLANK %X.\n", __func__, V4L2_CID_HBLANK);

	mira016->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &mira016_ctrl_ops,
					   V4L2_CID_HBLANK, mira016->mode->hblank,
					   mira016->mode->hblank, 1,
					   mira016->mode->hblank);

	// Make the vblank control read only. This could be changed to allow changing framerate in
	// runtime, but would require adapting other settings
	// mira016->vblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	// Exposure is indicated in number of lines here
	// Max is determined by vblank + vsize and Tglob.
	printk(KERN_INFO "[MIRA016]: %s V4L2_CID_EXPOSURE %X.\n", __func__, V4L2_CID_EXPOSURE);

	mira016->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &mira016_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     MIRA016_EXPOSURE_MIN_RT, MIRA016_EXPOSURE_MAX_RT,
					     1,
					     MIRA016_DEFAULT_EXPOSURE_RT);

	printk(KERN_INFO "[MIRA016]: %s V4L2_CID_ANALOGUE_GAIN %X.\n", __func__, V4L2_CID_ANALOGUE_GAIN);

	mira016->gain = v4l2_ctrl_new_std(ctrl_hdlr, &mira016_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  MIRA016_ANALOG_GAIN_MIN, MIRA016_ANALOG_GAIN_MAX,
			  MIRA016_ANALOG_GAIN_STEP, MIRA016_ANALOG_GAIN_DEFAULT);

	printk(KERN_INFO "[MIRA016]: %s V4L2_CID_HFLIP %X.\n", __func__, V4L2_CID_HFLIP);

	mira016->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &mira016_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 0, 1, 0);
	if (mira016->hflip)
		mira016->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	printk(KERN_INFO "[MIRA016]: %s V4L2_CID_VFLIP %X.\n", __func__, V4L2_CID_VFLIP);

	mira016->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &mira016_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 0, 1, 0);
	if (mira016->vflip)
		mira016->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	printk(KERN_INFO "[MIRA016]: %s V4L2_CID_TEST_PATTERN %X.\n", __func__, V4L2_CID_TEST_PATTERN);
	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &mira016_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(mira016_test_pattern_menu) - 1,
				     0, 0, mira016_test_pattern_menu);
	/*
	 * Custom op
	 */
	mira016_reg_w = &custom_ctrl_config_list[0];
	printk(KERN_INFO "[MIRA016]: %s AMS_CAMERA_CID_MIRA_REG_W %X.\n", __func__, AMS_CAMERA_CID_MIRA_REG_W);
	mira016->mira016_reg_w = v4l2_ctrl_new_custom(ctrl_hdlr, mira016_reg_w, NULL);

	mira016_reg_r = &custom_ctrl_config_list[1];
	printk(KERN_INFO "[MIRA016]: %s AMS_CAMERA_CID_MIRA_REG_R %X.\n", __func__, AMS_CAMERA_CID_MIRA_REG_R);
	mira016->mira016_reg_r = v4l2_ctrl_new_custom(ctrl_hdlr, mira016_reg_r, NULL);
	if (mira016->mira016_reg_r)
		mira016->mira016_reg_r->flags |= (V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &mira016_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	mira016->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&mira016->mutex);

	return ret;
}

static void mira016_free_controls(struct mira016 *mira016)
{
	v4l2_ctrl_handler_free(mira016->sd.ctrl_handler);
	mutex_destroy(&mira016->mutex);
}

static int mira016_check_hwcfg(struct device *dev)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	int ret = -EINVAL;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg)) {
		dev_err(dev, "could not parse endpoint\n");
		goto error_out;
	}

	/* Check the number of MIPI CSI2 data lanes */
	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 1) {
		dev_err(dev, "only 1 data lanes are currently supported\n");
		goto error_out;
	}

	/* Check the link frequency set in device tree */
	if (!ep_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		goto error_out;
	}

	if (ep_cfg.nr_of_link_frequencies != 1 ||
	    ep_cfg.link_frequencies[0] != MIRA016_DEFAULT_LINK_FREQ) {
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

static int mira016pmic_read(struct i2c_client *client, u8 reg, u8 *val)
{
	int ret;
	unsigned char data_w[1] = { reg & 0xff };

	ret = i2c_master_send(client, data_w, 1);
	/*
	 * A negative return code, or sending the wrong number of bytes, both
	 * count as an error.
	 */
	if (ret != 1) {
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
	if (ret == 1) {
		ret = 0;
	} else {
		dev_dbg(&client->dev, "%s: i2c read error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
	}

	return ret;
}


static int mira016pmic_init_controls(struct i2c_client *pmic_client, struct i2c_client *uc_client)
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
	ret = mira016pmic_write(uc_client, 12, 0xF7);
	ret = mira016pmic_write(uc_client, 16, 0xFF); // ldo en:1
	ret = mira016pmic_write(uc_client, 11, 0XCF);
	ret = mira016pmic_write(uc_client, 15, 0xFF);
	ret = mira016pmic_write(uc_client, 6, 1); // write

	// Disable master switch //
	ret = mira016pmic_write(pmic_client, 0x62, 0x00);

	// Set all voltages to 0

	// DCDC1=0V
	ret = mira016pmic_write(pmic_client, 0x05, 0x00);
	// DCDC4=0V
	ret = mira016pmic_write(pmic_client, 0x0E, 0x0);
	// LDO1=0V VDDLO_PLL
	ret = mira016pmic_write(pmic_client, 0x11, 0x0);
	// LDO2=0.0V
	ret = mira016pmic_write(pmic_client, 0x14, 0x00);
	// LDO3=0.0V
	ret = mira016pmic_write(pmic_client, 0x17, 0x00);
	// LDO4=0V
	ret = mira016pmic_write(pmic_client, 0x1A, 0x00);
	// LDO5=0.0V
	ret = mira016pmic_write(pmic_client, 0x1C, 0x00);
	// LDO6=0.0V
	ret = mira016pmic_write(pmic_client, 0x1D, 0x00);
	// LDO7=0V
	ret = mira016pmic_write(pmic_client, 0x1E, 0x0);
	// LDO8=0.0V
	ret = mira016pmic_write(pmic_client, 0x1F, 0x00);
	// Disable LDO9 Lock
	ret = mira016pmic_write(pmic_client, 0x24, 0x48);
	// LDO9=0V VDDHI
	ret = mira016pmic_write(pmic_client, 0x20, 0x00);
	// LDO10=0V VDDLO_ANA
	ret = mira016pmic_write(pmic_client, 0x21, 0x0);

	// Enable master switch //
	usleep_range(50,60);
	ret = mira016pmic_write(pmic_client, 0x62, 0x0D);  // enable master switch
	usleep_range(50,60);

	// start PMIC
	// Keep LDOs always on
	ret = mira016pmic_write(pmic_client, 0x27, 0xFF);
	ret = mira016pmic_write(pmic_client, 0x28, 0xFF);
	ret = mira016pmic_write(pmic_client, 0x29, 0x00);
	ret = mira016pmic_write(pmic_client, 0x2A, 0x00);
	ret = mira016pmic_write(pmic_client, 0x2B, 0x00);

	// Unused LDO off //
	usleep_range(50,60);
	// set GPIO1=0
	ret = mira016pmic_write(pmic_client, 0x41, 0x04);
	// DCDC2=0.0V SPARE_PWR1
	ret = mira016pmic_write(pmic_client, 0x01, 0x00);
	ret = mira016pmic_write(pmic_client, 0x08, 0x00);
	// DCDC3=0V SPARE_PWR1
	ret = mira016pmic_write(pmic_client, 0x02, 0x00);
	ret = mira016pmic_write(pmic_client, 0x0B, 0x00);
	// LDO2=0.0V
	ret = mira016pmic_write(pmic_client, 0x14, 0x00);
	// LDO3=0.0V
	ret = mira016pmic_write(pmic_client, 0x17, 0x00);
	// LDO5=0.0V
	ret = mira016pmic_write(pmic_client, 0x1C, 0x00);
	// LDO6=0.0V
	ret = mira016pmic_write(pmic_client, 0x1D, 0x00);
	// LDO8=0.0V
	ret = mira016pmic_write(pmic_client, 0x1F, 0x00);

	ret = mira016pmic_write(pmic_client, 0x42, 4);

	// Enable 1.80V //
	usleep_range(50,60);
	// DCDC1=1.8V VINLDO1p8 >=1P8
	ret = mira016pmic_write(pmic_client, 0x00, 0x00);
	ret = mira016pmic_write(pmic_client, 0x04, 0x34);
	ret = mira016pmic_write(pmic_client, 0x06, 0xBF);
	ret = mira016pmic_write(pmic_client, 0x05, 0xB4);
	// DCDC4=1.8V VDDIO
	ret = mira016pmic_write(pmic_client, 0x03, 0x00);
	ret = mira016pmic_write(pmic_client, 0x0D, 0x34);
	ret = mira016pmic_write(pmic_client, 0x0F, 0xBF);
	ret = mira016pmic_write(pmic_client, 0x0E, 0xB4);

	// Enable 2.85V //
	usleep_range(50,60);
	// LDO4=2.85V VDDHI alternativ
	ret = mira016pmic_write(pmic_client, 0x1A, 0xB8); // Either 0x00 or 0xB8
	// Disable LDO9 Lock
	ret = mira016pmic_write(pmic_client, 0x24, 0x48);
	// LDO9=2.85V VDDHI
	ret = mira016pmic_read(pmic_client, 0x20, &val);
	dev_err(&pmic_client->dev, "Read 0x20 with val %x\n", val);
	ret = mira016pmic_write(pmic_client, 0x20, 0xB9);
	ret = mira016pmic_read(pmic_client, 0x20, &val);
	dev_err(&pmic_client->dev, "Read 0x20 with val %x\n", val);

	// VPIXH on cob = vdd25A on interposer = LDO4 on pmic
	// VPIXH should connect to VDD28 on pcb, or enable 4th supply
	ret = mira016pmic_read(pmic_client, 0x19, &val);
	dev_err(&pmic_client->dev, "Read 0x19 with val %x\n", val);
	ret = mira016pmic_write(pmic_client, 0x19, 0x38);
	ret = mira016pmic_read(pmic_client, 0x19, &val);
	dev_err(&pmic_client->dev, "Read 0x19 with val %x\n", val);


	// Enable 1.2V //
	usleep_range(700,710);
	// LDO1=1.2V VDDLO_PLL
	ret = mira016pmic_write(pmic_client, 0x12, 0x16);
	ret = mira016pmic_write(pmic_client, 0x10, 0x16);
	ret = mira016pmic_write(pmic_client, 0x11, 0x90);
	// LDO7=1.2V VDDLO_DIG
	ret = mira016pmic_write(pmic_client, 0x1E, 0x90);
	// LDO10=1.2V VDDLO_ANA
	ret = mira016pmic_write(pmic_client, 0x21, 0x90);

	// Enable green LED //
	usleep_range(50,60);
	ret = mira016pmic_write(pmic_client, 0x42, 0x15); // gpio2
	// ret = mira016pmic_write(pmic_client, 0x43, 0x40); // leda
	// ret = mira016pmic_write(pmic_client, 0x44, 0x40); // ledb
	ret = mira016pmic_write(pmic_client, 0x45, 0x40); // ledc

	// ret = mira016pmic_write(pmic_client, 0x47, 0x02); // leda ctrl1
	// ret = mira016pmic_write(pmic_client, 0x4F, 0x02); // ledb ctrl1
	ret = mira016pmic_write(pmic_client, 0x57, 0x02); // ledc ctrl1


	// ret = mira016pmic_write(pmic_client, 0x4D, 0x01); // leda ctrl1
	// ret = mira016pmic_write(pmic_client, 0x55, 0x10); // ledb ctrl7
	ret = mira016pmic_write(pmic_client, 0x5D, 0x10); // ledc ctrl7
	ret = mira016pmic_write(pmic_client, 0x61, 0x10); // led seq -- use this to turn on leds. abc0000- 1110000 for all leds

	// uC, set atb and jtag high and ldo_en
	ret = mira016pmic_write(uc_client, 12, 0xF7);
	ret = mira016pmic_write(uc_client, 16, 0xF7); // ldo en:0
	/*
	 * In Mira016-bringup.py, write 11, 0xCF; 15: 0x30.
	 * In mira016.py, write 11, 0x8D; 15, 0xFD.
	 */
	ret = mira016pmic_write(uc_client, 11, 0X8D);
	ret = mira016pmic_write(uc_client, 15, 0xFD);
	ret = mira016pmic_write(uc_client, 6, 1); // write

	usleep_range(2000000,2001000);

	return 0;
}


static int mira016_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct mira016 *mira016;
	int ret;

	printk(KERN_INFO "[MIRA016]: probing v4l2 sensor.\n");
	printk(KERN_INFO "[MIRA016]: Driver Version 0.0.\n");

	dev_err(dev, "[MIRA016] name: %s.\n", client->name);

	mira016 = devm_kzalloc(&client->dev, sizeof(*mira016), GFP_KERNEL);
	if (!mira016)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&mira016->sd, client, &mira016_subdev_ops);

	/* Check the hardware configuration in device tree */
	if (mira016_check_hwcfg(dev))
		return -EINVAL;

        /* Parse device tree to check if dtoverlay has param skip-reg-upload=1 */
        device_property_read_u32(dev, "skip-reg-upload", &mira016->skip_reg_upload);
	printk(KERN_INFO "[MIRA016]: skip-reg-upload %d.\n", mira016->skip_reg_upload);
	/* Set default TBD I2C device address to LED I2C Address*/
	mira016->tbd_client_i2c_addr = MIRA016LED_I2C_ADDR;
	printk(KERN_INFO "[MIRA016]: User defined I2C device address defaults to LED driver I2C address 0x%X.\n", mira016->tbd_client_i2c_addr);

	/* Get system clock (xclk) */
	mira016->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(mira016->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(mira016->xclk);
	}

	mira016->xclk_freq = clk_get_rate(mira016->xclk);
	if (mira016->xclk_freq != MIRA016_SUPPORTED_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			mira016->xclk_freq);
		return -EINVAL;
	}

	ret = mira016_get_regulators(mira016);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	{
		printk(KERN_INFO "[MIRA016]: Init PMIC and uC and led driver.\n");
		mira016->pmic_client = i2c_new_dummy_device(client->adapter,
				MIRA016PMIC_I2C_ADDR);
		if (IS_ERR(mira016->pmic_client))
			return PTR_ERR(mira016->pmic_client);
		mira016->uc_client = i2c_new_dummy_device(client->adapter,
				MIRA016UC_I2C_ADDR);
		if (IS_ERR(mira016->uc_client))
			return PTR_ERR(mira016->uc_client);
		mira016->led_client = i2c_new_dummy_device(client->adapter,
				MIRA016LED_I2C_ADDR);
		if (IS_ERR(mira016->led_client))
			return PTR_ERR(mira016->led_client);
		mira016pmic_init_controls(mira016->pmic_client, mira016->uc_client);
	}

	dev_err(dev, "[MIRA016] Sleep for 1 second to let PMIC driver complete init.\n");
	usleep_range(1000000, 1000000+100);

	/*
	 * The sensor must be powered for mira016_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = mira016_power_on(dev);
	if (ret)
		return ret;

	printk(KERN_INFO "[MIRA016]: Entering identify function.\n");

	ret = mira016_identify_module(mira016);
	if (ret)
		goto error_power_off;

	printk(KERN_INFO "[MIRA016]: Setting support function.\n");

	/* Set default mode to max resolution */
	mira016->mode = &supported_modes[0];

	printk(KERN_INFO "[MIRA016]: Entering init controls function.\n");

	ret = mira016_init_controls(mira016);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	mira016->sd.internal_ops = &mira016_internal_ops;
	mira016->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	mira016->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	mira016->pad[IMAGE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	mira016->pad[METADATA_PAD].flags = MEDIA_PAD_FL_SOURCE;

	printk(KERN_INFO "[MIRA016]: Entering set default format function.\n");

	/* Initialize default format */
	mira016_set_default_format(mira016);

	printk(KERN_INFO "[MIRA016]: Entering pads init function.\n");

	ret = media_entity_pads_init(&mira016->sd.entity, NUM_PADS, mira016->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	printk(KERN_INFO "[MIRA016]: Entering subdev sensor common function.\n");

	ret = v4l2_async_register_subdev_sensor(&mira016->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	/* For debug purpose */
	// mira016_start_streaming(mira016);

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

error_media_entity:
	media_entity_cleanup(&mira016->sd.entity);

error_handler_free:
	mira016_free_controls(mira016);

error_power_off:
	mira016_power_off(dev);

	i2c_unregister_device(mira016->pmic_client);
	i2c_unregister_device(mira016->uc_client);
	i2c_unregister_device(mira016->led_client);

	return ret;
}

static void mira016_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira016 *mira016 = to_mira016(sd);

	i2c_unregister_device(mira016->pmic_client);
	i2c_unregister_device(mira016->uc_client);
	i2c_unregister_device(mira016->led_client);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	mira016_free_controls(mira016);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		mira016_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);

}

static const struct dev_pm_ops mira016_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mira016_suspend, mira016_resume)
	SET_RUNTIME_PM_OPS(mira016_power_off, mira016_power_on, NULL)
};

#endif // __MIRA016_INL__
