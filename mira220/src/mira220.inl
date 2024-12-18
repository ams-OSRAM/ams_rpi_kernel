// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for ams MIRA220 cameras.
 * Copyright (C) 2022, ams-OSRAM
 *
 * Based on Sony IMX219 camera driver
 * Copyright (C) 2019, Raspberry Pi (Trading) Ltd
 */

#ifndef __MIRA220_INL__
#define __MIRA220_INL__

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
#define AMS_CAMERA_CID_MIRA220_REG_FLAG_FOR_READ        0b00000001
/* Use bit 5 to indicate special command, bit 1,2,3,4 for command. */
#define AMS_CAMERA_CID_MIRA220_REG_FLAG_CMD_SEL         0b00010000
/* Special command for sleep. The other 3 Bytes (addr+val) is sleep values in us. */
#define AMS_CAMERA_CID_MIRA220_REG_FLAG_SLEEP_US        0b00010000
/* Special command to enable power on (/off) when stream on (/off). */
#define AMS_CAMERA_CID_MIRA220_REG_FLAG_RESET_ON        0b00010010
/* Special command to disable power on (/off) when stream on (/off). */
#define AMS_CAMERA_CID_MIRA220_REG_FLAG_RESET_OFF       0b00010100
/* Special command to enable base register sequence upload, overwrite skip-reg-upload in dtoverlay */
#define AMS_CAMERA_CID_MIRA220_REG_FLAG_REG_UP_ON       0b00010110
/* Special command to disable base register sequence upload, overwrite skip-reg-upload in dtoverlay */
#define AMS_CAMERA_CID_MIRA220_REG_FLAG_REG_UP_OFF      0b00011000
/* Special command to manually power on */
#define AMS_CAMERA_CID_MIRA220_REG_FLAG_POWER_ON        0b00011010
/* Special command to manually power off */
#define AMS_CAMERA_CID_MIRA220_REG_FLAG_POWER_OFF       0b00011100
/* Special command to turn illumination trigger on */
#define AMS_CAMERA_CID_MIRA220_REG_FLAG_ILLUM_TRIG_ON   0b00011110
/* Special command to turn illumination trigger off */
#define AMS_CAMERA_CID_MIRA220_REG_FLAG_ILLUM_TRIG_OFF  0b00010001
/* Special command to set ILLUM_WIDTH. The other 3 Bytes (addr+val) is width value. */
#define AMS_CAMERA_CID_MIRA220_REG_FLAG_ILLUM_WIDTH     0b00010011
/* Special command to set ILLUM_DELAY. The other 3 Bytes (addr+val) is width value. */
#define AMS_CAMERA_CID_MIRA220_REG_FLAG_ILLUM_DELAY     0b00010101
/* Special command to enable force_stream_ctrl */
#define AMS_CAMERA_CID_MIRA220_REG_FLAG_STREAM_CTRL_ON  0b00011011
/* Special command to disable force_stream_ctrl */
#define AMS_CAMERA_CID_MIRA220_REG_FLAG_STREAM_CTRL_OFF 0b00011101

/*
 * Bit 6&7 of flag are combined to specify I2C dev (default is Mira).
 * If bit 6&7 is 0b01, the reg_addr and reg_val are for a TBD I2C address.
 * The TBD I2C address is default to MIRA220LED_I2C_ADDR.
 * To change the TBD I2C address, set bit 6&7 to 0b10,
 * then the reg_val will become TBD I2C address.
 * The TBD I2C address is stored in mira220->tbd_client_i2c_addr.
 */
#define AMS_CAMERA_CID_MIRA220_REG_FLAG_I2C_SEL         0b01100000
#define AMS_CAMERA_CID_MIRA220_REG_FLAG_I2C_MIRA        0b00000000
#define AMS_CAMERA_CID_MIRA220_REG_FLAG_I2C_TBD         0b00100000
#define AMS_CAMERA_CID_MIRA220_REG_FLAG_I2C_SET_TBD     0b01000000

/* Pre-allocated i2c_client */
#define MIRA220PMIC_I2C_ADDR 0x2D
#define MIRA220UC_I2C_ADDR 0x0A
#define MIRA220LED_I2C_ADDR 0x53


#define MIRA220_NATIVE_WIDTH			1600U
#define MIRA220_NATIVE_HEIGHT			1400U

#define MIRA220_PIXEL_ARRAY_LEFT		0U
#define MIRA220_PIXEL_ARRAY_TOP			0U
#define MIRA220_PIXEL_ARRAY_WIDTH		1600U
#define MIRA220_PIXEL_ARRAY_HEIGHT		1400U

#define MIRA220_ANALOG_GAIN_REG			0x400A
#define MIRA220_ANALOG_GAIN_MIN			1
#define MIRA220_ANALOG_GAIN_MAX			1 /* Fixed analog gain to 1, to avoid unexpected behavior. */
#define MIRA220_ANALOG_GAIN_STEP		1
#define MIRA220_ANALOG_GAIN_DEFAULT		MIRA220_ANALOG_GAIN_MIN

#define MIRA220_BIT_DEPTH_REG			0x209E
#define MIRA220_BIT_DEPTH_12_BIT		0x02
#define MIRA220_BIT_DEPTH_10_BIT		0x04
#define MIRA220_BIT_DEPTH_8_BIT			0x06

#define MIRA220_CSI_DATA_TYPE_REG		0x208D
#define MIRA220_CSI_DATA_TYPE_12_BIT		0x04
#define MIRA220_CSI_DATA_TYPE_10_BIT		0x02
#define MIRA220_CSI_DATA_TYPE_8_BIT		0x01

#define MIRA220_IMAGER_STATE_REG		0x1003
#define MIRA220_IMAGER_STATE_STOP_AT_ROW	0x02
#define MIRA220_IMAGER_STATE_STOP_AT_FRAME	0x04
#define MIRA220_IMAGER_STATE_MASTER_CONTROL	0x10

#define MIRA220_IMAGER_RUN_REG			0x10F0
#define MIRA220_IMAGER_RUN_START		0x01
#define MIRA220_IMAGER_RUN_STOP			0x00

#define MIRA220_IMAGER_RUN_CONT_REG		0x1002
#define MIRA220_IMAGER_RUN_CONT_ENABLE		0x04
#define MIRA220_IMAGER_RUN_CONT_DISABLE		0x00

#define MIRA220_NB_OF_FRAMES_LO_REG		0x10F2
#define MIRA220_NB_OF_FRAMES_HI_REG		0x10F3

#define MIRA220_POWER_MODE_REG			0x0043
#define MIRA220_POWER_MODE_SLEEP		0x01
#define MIRA220_POWER_MODE_IDLE			0x02
#define MIRA220_POWER_MODE_ACTIVE		0x0C

// Exposure time is indicated in number of rows
#define MIRA220_EXP_TIME_LO_REG			0x100C
#define MIRA220_EXP_TIME_HI_REG			0x100D

// VBLANK is indicated in number of rows
#define MIRA220_VBLANK_LO_REG			0x1012
#define MIRA220_VBLANK_HI_REG			0x1013

#define MIRA220_EXT_EXP_PW_SEL_REG		0x1001
#define MIRA220_EXT_EXP_PW_SEL_USE_REG		1
#define MIRA220_EXT_EXP_PW_SEL_USE_EXT		0

// Exposure delay is indicated in number of rows
#define MIRA220_EXT_EXP_DELAY_LO_REG		0x10D0
#define MIRA220_EXT_EXP_DELAY_HI_REG		0x10D1

// Sets the duration of the row length in clock cycles of CLK_IN
#define MIRA220_ROW_LENGTH_LO_REG		0x102B
#define MIRA220_ROW_LENGTH_HI_REG		0x102C

#define MIRA220_VSIZE1_LO_REG			0x1087
#define MIRA220_VSIZE1_HI_REG			0x1088
#define MIRA220_VSIZE1_MASK			0x7FF

#define MIRA220_VSTART1_LO_REG			0x107D
#define MIRA220_VSTART1_HI_REG			0x107E
#define MIRA220_VSTART1_MASK			0x7FF

// HSIZE units are number of columns / 2
#define MIRA220_HSIZE_LO_REG			0x2008
#define MIRA220_HSIZE_HI_REG			0x2009
#define MIRA220_HSIZE_MASK			0x3FF

// HSTART units are number of columns / 2
#define MIRA220_HSTART_LO_REG			0x200A
#define MIRA220_HSTART_HI_REG			0x200B
#define MIRA220_HSTART_MASK			0x3FF

// MIPI_HSIZE units are number of columns (HSIZE * 2)
#define MIRA220_MIPI_HSIZE_LO_REG		0x207D
#define MIRA220_MIPI_HSIZE_HI_REG		0x207E
#define MIRA220_MIPI_HSIZE_MASK			0xFFFF

#define MIRA220_HFLIP_REG			0x209C
#define MIRA220_HFLIP_ENABLE_MIRROR		1
#define MIRA220_HFLIP_DISABLE_MIRROR		0

#define MIRA220_VFLIP_REG			0x1095
#define MIRA220_VFLIP_ENABLE_FLIP		1
#define MIRA220_VFLIP_DISABLE_FLIP		0

#define MIRA220_BIT_ORDER_REG			0x2063
#define MIRA220_BIT_ORDER_NORMAL		0
#define MIRA220_BIT_ORDER_REVERSED		1

#define MIRA220_BSP_REG				0x4006
#define MIRA220_BSP_ENABLE			0x08
#define MIRA220_BSP_DISABLE			0x0F

#define MIRA220_MIPI_SOFT_RESET_REG		0x5004
#define MIRA220_MIPI_SOFT_RESET_DPHY		0x01
#define MIRA220_MIPI_SOFT_RESET_NONE		0x00

#define MIRA220_FSYNC_EOF_MAX_CTR_LO_REG	0x2066
#define MIRA220_FSYNC_EOF_MAX_CTR_HI_REG	0x2067

#define MIRA220_FSYNC_EOF_VEND_ST_LO_REG	0x206E
#define MIRA220_FSYNC_EOF_VEND_ST_HI_REG	0x206F

#define MIRA220_FSYNC_EOF_HSTART_EMB_ST_LO_REG	0x2076
#define MIRA220_FSYNC_EOF_HSTART_EMB_ST_HI_REG	0x2077

#define MIRA220_FSYNC_EOF_DSTART_EMB_ST_LO_REG	0x2078
#define MIRA220_FSYNC_EOF_DSTART_EMB_ST_HI_REG	0x2079

#define MIRA220_FSYNC_EOF_HEND_EMB_ST_LO_REG	0x207A
#define MIRA220_FSYNC_EOF_HEND_EMB_ST_HI_REG	0x207B

#define MIRA220_GLOB_NUM_CLK_CYCLES		1928

#define MIRA220_SUPPORTED_XCLK_FREQ		24000000



// Default exposure is adjusted to mode with smallest height
#define MIRA220_DEFAULT_EXPOSURE		100 //(MIRA220_MIN_V_SIZE + MIRA220_MIN_VBLANK - MIRA220_GLOB_NUM_CLK_CYCLES / MIRA220_MIN_ROW_LENGTH) //TODO
#define MIRA220_EXPOSURE_MIN			1

// Power on function timing
#define MIRA220_XCLR_MIN_DELAY_US		100000
#define MIRA220_XCLR_DELAY_RANGE_US		30



// Outdated. See below.
// pixel_rate = link_freq * 2 * nr_of_lanes / bits_per_sample
// 1.0Gb/s * 2 * 2 / 12 = 357913941
// #define MIRA220_PIXEL_RATE		(357913941)

// Mira220 PIXEL_RATE is derived from ROW_LENGTH. See datasheet Section 9.2.
// ROW_LENGTH is set by registers: 0x102B, 0x102C. Unit is number of CLK_IN cycles.
// PIXEL_RATE = 1000000000 * WIDTH / (ROW_LENGTH * CLK_IN_PERIOD_NS)
// ROW_LENGTH_1600x1400_1000GBS=300
// ROW_LENGTH_640x480_1000GBS=450
// CLK_IN_PERIOD_NS = 1.0 s / 38.4 Mhz = 26.04 ns
// MIRA220_PIXEL_RATE = 1000000000 * 1600 / (300 * 26.04) = 204813108
// MIRA220_PIXEL_RATE = 1000000000 * 640 / (450 * 26.04) = 54616828
#define MIRA220_PIXEL_RATE   384000000 //384M (x10)
// Row time in microseconds. Not used in driver, but used in libcamera cam_helper.
// ROW_TIME_US = ROW_LENGTH * CLK_IN_PERIOD_NS / 1000
// MIRA220_ROW_TIME_1600x1400_1000GBS_US=(300*26.04/1000)=7.8us
// MIRA220_ROW_TIME_640x480_1000GBS_US=(450*26.04/1000)=11.7us

/* Should match device tree link freq */
#define MIRA220_DEFAULT_LINK_FREQ	456000000

/* Trick the libcamera with achievable fps via hblank */

/* Formular in libcamera to derive TARGET_FPS:
 * TARGET_FPS=1/((1/MIRA220_PIXEL_RATE)*(WIDTH+HBLANK)*(HEIGHT+MIRA220_MIN_VBLANK))
 * Example: 640x480 with HBLANK=0 and MIRA220_MIN_VBLANK=13
 * TARGET_FPS=1/((1/54616828)*640*(480+13))=173
 * Example: 1600x1400 with HBLANK=0 and MIRA220_MIN_VBLANK=13
 * TARGET_FPS=1/((1/204813108)*1600*(1400+13))=91
 * 
 * Inverse the above formula to derive HBLANK from TARGET_FPS:
 * HBLANK=1/((1/MIRA220_PIXEL_RATE)*TARGET_FPS*(HEIGHT+MIRA220_MIN_VBLANK))-WIDTH
 * Example with TARGET_FPS of 120 fps for 640x480
 * HBLANK=1/((1/54616828)*120*(480+13))-640=283
 * Example with TARGET_FPS of 30 fps for 1600x1400
 * HBLANK=1/((1/204813108)*30*(1400+13))-1600=3232
 */

#define MIRA220_HBLANK_640x480_120FPS		3860
#define MIRA220_HBLANK_1600x1400_30FPS		2900
#define MIRA220_HBLANK_1600x1400_1500		1400

#define MIRA220_HBLANK_400x400_304			2640
#define MIRA220_HBLANK_640x480_304			2400
#define MIRA220_HBLANK_1600x1400_304		1440


#define MIRA220_REG_TEST_PATTERN	0x2091
#define	MIRA220_TEST_PATTERN_DISABLE	0x00
#define	MIRA220_TEST_PATTERN_VERTICAL_GRADIENT	0x01

/* Embedded metadata stream structure */
#define MIRA220_EMBEDDED_LINE_WIDTH 16384
#define MIRA220_NUM_EMBEDDED_LINES 1

/* From Jetson driver */
#define MIRA220_DEFAULT_LINE_LENGTH    (0xA80)
#define MIRA220_DEFAULT_PIXEL_CLOCK    (160)
#define MIRA220_DEFAULT_FRAME_LENGTH    (0x07C0) //TODO REMOVE THESE

/* Illumination trigger */
#define MIRA220_EN_TRIG_ILLUM_REG     0x10D7
#define MIRA220_ILLUM_WIDTH_REG       0x10D5
#define MIRA220_ILLUM_DELAY_REG       0x10D2
#define MIRA220_ILLUM_DELAY_SIGN_REG  0x10D4
#define MIRA220_ILLUM_WIDTH_DEFAULT   (0)
#define MIRA220_ILLUM_DELAY_DEFAULT   (0)

enum pad_types {
	IMAGE_PAD,
	METADATA_PAD,
	NUM_PADS
};

struct mira220_reg {
	u16 address;
	u8 val;
};

struct mira220_reg_list {
	unsigned int num_of_regs;
	const struct mira220_reg *regs;
};

struct mira220_v4l2_reg {
	u32 val;
};

/* Mode : resolution and related config&values */
struct mira220_mode {
	/* Frame width */
	unsigned int width;
	/* Frame height */
	unsigned int height;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* Default register values */
	struct mira220_reg_list reg_list;
	u32 row_length;

	u32 pixel_rate;
	u32 min_vblank;
	u32 max_vblank;
	u32 hblank;
	u32 code;
};

// 1600_1400_30fps_12b_2lanes
static const struct mira220_reg full_1600_1400_1500_12b_2lanes_reg[] = {

	{0x1003,0x2}, //,Initial Upload
	{0x6006,0x0}, //,Initial Upload
	{0x6012,0x1}, //,Initial Upload
	{0x6013,0x0}, //,Initial Upload
	{0x6006,0x1}, //,Initial Upload
	{0x205D,0x0}, //,Initial Upload
	{0x2063,0x0}, //,Initial Upload
	{0x24DC,0x13}, //,Initial Upload
	{0x24DD,0x3}, //,Initial Upload
	{0x24DE,0x3}, //,Initial Upload
	{0x24DF,0x0}, //,Initial Upload
	{0x4006,0x8}, //,Initial Upload
	{0x401C,0x6F}, //,Initial Upload
	{0x204B,0x3}, //,Initial Upload
	{0x205B,0x64}, //,Initial Upload
	{0x205C,0x0}, //,Initial Upload
	{0x4018,0x3F}, //,Initial Upload
	{0x403B,0xB}, //,Initial Upload
	{0x403E,0xE}, //,Initial Upload
	{0x402B,0x6}, //,Initial Upload
	{0x401E,0x2}, //,Initial Upload
	{0x4038,0x3B}, //,Initial Upload
	{0x1077,0x0}, //,Initial Upload
	{0x1078,0x0}, //,Initial Upload
	{0x1009,0x8}, //,Initial Upload
	{0x100A,0x0}, //,Initial Upload
	{0x110F,0x8}, //,Initial Upload
	{0x1110,0x0}, //,Initial Upload
	{0x1006,0x2}, //,Initial Upload
	{0x402C,0x64}, //,Initial Upload
	{0x3064,0x0}, //,Initial Upload
	{0x3065,0xF0}, //,Initial Upload
	{0x4013,0x13}, //,Initial Upload
	{0x401F,0x9}, //,Initial Upload
	{0x4020,0x13}, //,Initial Upload
	{0x4044,0x75}, //,Initial Upload
	{0x4027,0x0}, //,Initial Upload
	{0x3215,0x69}, //,Initial Upload
	{0x3216,0xF}, //,Initial Upload
	{0x322B,0x69}, //,Initial Upload
	{0x322C,0xF}, //,Initial Upload
	{0x4051,0x80}, //,Initial Upload
	{0x4052,0x10}, //,Initial Upload
	{0x4057,0x80}, //,Initial Upload
	{0x4058,0x10}, //,Initial Upload
	{0x3212,0x59}, //,Initial Upload
	{0x4047,0x8F}, //,Initial Upload
	{0x4026,0x10}, //,Initial Upload
	{0x4032,0x53}, //,Initial Upload
	{0x4036,0x17}, //,Initial Upload
	{0x50B8,0xF4}, //,Initial Upload
	{0x3016,0x0}, //,Initial Upload
	{0x3017,0x2C}, //,Initial Upload
	{0x3018,0x8C}, //,Initial Upload
	{0x3019,0x45}, //,Initial Upload
	{0x301A,0x5}, //,Initial Upload
	{0x3013,0xA}, //,Initial Upload
	{0x301B,0x0}, //,Initial Upload
	{0x301C,0x4}, //,Initial Upload
	{0x301D,0x88}, //,Initial Upload
	{0x301E,0x45}, //,Initial Upload
	{0x301F,0x5}, //,Initial Upload
	{0x3020,0x0}, //,Initial Upload
	{0x3021,0x4}, //,Initial Upload
	{0x3022,0x88}, //,Initial Upload
	{0x3023,0x45}, //,Initial Upload
	{0x3024,0x5}, //,Initial Upload
	{0x3025,0x0}, //,Initial Upload
	{0x3026,0x4}, //,Initial Upload
	{0x3027,0x88}, //,Initial Upload
	{0x3028,0x45}, //,Initial Upload
	{0x3029,0x5}, //,Initial Upload
	{0x302F,0x0}, //,Initial Upload
	{0x3056,0x0}, //,Initial Upload
	{0x3057,0x0}, //,Initial Upload
	{0x3300,0x1}, //,Initial Upload
	{0x3301,0x0}, //,Initial Upload
	{0x3302,0xB0}, //,Initial Upload
	{0x3303,0xB0}, //,Initial Upload
	{0x3304,0x16}, //,Initial Upload
	{0x3305,0x15}, //,Initial Upload
	{0x3306,0x1}, //,Initial Upload
	{0x3307,0x0}, //,Initial Upload
	{0x3308,0x30}, //,Initial Upload
	{0x3309,0xA0}, //,Initial Upload
	{0x330A,0x16}, //,Initial Upload
	{0x330B,0x15}, //,Initial Upload
	{0x330C,0x1}, //,Initial Upload
	{0x330D,0x0}, //,Initial Upload
	{0x330E,0x30}, //,Initial Upload
	{0x330F,0xA0}, //,Initial Upload
	{0x3310,0x16}, //,Initial Upload
	{0x3311,0x15}, //,Initial Upload
	{0x3312,0x1}, //,Initial Upload
	{0x3313,0x0}, //,Initial Upload
	{0x3314,0x30}, //,Initial Upload
	{0x3315,0xA0}, //,Initial Upload
	{0x3316,0x16}, //,Initial Upload
	{0x3317,0x15}, //,Initial Upload
	{0x3318,0x1}, //,Initial Upload
	{0x3319,0x0}, //,Initial Upload
	{0x331A,0x30}, //,Initial Upload
	{0x331B,0xA0}, //,Initial Upload
	{0x331C,0x16}, //,Initial Upload
	{0x331D,0x15}, //,Initial Upload
	{0x331E,0x1}, //,Initial Upload
	{0x331F,0x0}, //,Initial Upload
	{0x3320,0x30}, //,Initial Upload
	{0x3321,0xA0}, //,Initial Upload
	{0x3322,0x16}, //,Initial Upload
	{0x3323,0x15}, //,Initial Upload
	{0x3324,0x1}, //,Initial Upload
	{0x3325,0x0}, //,Initial Upload
	{0x3326,0x30}, //,Initial Upload
	{0x3327,0xA0}, //,Initial Upload
	{0x3328,0x16}, //,Initial Upload
	{0x3329,0x15}, //,Initial Upload
	{0x332A,0x2B}, //,Initial Upload
	{0x332B,0x0}, //,Initial Upload
	{0x332C,0x30}, //,Initial Upload
	{0x332D,0xA0}, //,Initial Upload
	{0x332E,0x16}, //,Initial Upload
	{0x332F,0x15}, //,Initial Upload
	{0x3330,0x1}, //,Initial Upload
	{0x3331,0x0}, //,Initial Upload
	{0x3332,0x10}, //,Initial Upload
	{0x3333,0xA0}, //,Initial Upload
	{0x3334,0x16}, //,Initial Upload
	{0x3335,0x15}, //,Initial Upload
	{0x3058,0x8}, //,Initial Upload
	{0x3059,0x0}, //,Initial Upload
	{0x305A,0x9}, //,Initial Upload
	{0x305B,0x0}, //,Initial Upload
	{0x3336,0x1}, //,Initial Upload
	{0x3337,0x0}, //,Initial Upload
	{0x3338,0x90}, //,Initial Upload
	{0x3339,0xB0}, //,Initial Upload
	{0x333A,0x16}, //,Initial Upload
	{0x333B,0x15}, //,Initial Upload
	{0x333C,0x1F}, //,Initial Upload
	{0x333D,0x0}, //,Initial Upload
	{0x333E,0x10}, //,Initial Upload
	{0x333F,0xA0}, //,Initial Upload
	{0x3340,0x16}, //,Initial Upload
	{0x3341,0x15}, //,Initial Upload
	{0x3342,0x52}, //,Initial Upload
	{0x3343,0x0}, //,Initial Upload
	{0x3344,0x10}, //,Initial Upload
	{0x3345,0x80}, //,Initial Upload
	{0x3346,0x16}, //,Initial Upload
	{0x3347,0x15}, //,Initial Upload
	{0x3348,0x1}, //,Initial Upload
	{0x3349,0x0}, //,Initial Upload
	{0x334A,0x10}, //,Initial Upload
	{0x334B,0x80}, //,Initial Upload
	{0x334C,0x16}, //,Initial Upload
	{0x334D,0x1D}, //,Initial Upload
	{0x334E,0x1}, //,Initial Upload
	{0x334F,0x0}, //,Initial Upload
	{0x3350,0x50}, //,Initial Upload
	{0x3351,0x84}, //,Initial Upload
	{0x3352,0x16}, //,Initial Upload
	{0x3353,0x1D}, //,Initial Upload
	{0x3354,0x18}, //,Initial Upload
	{0x3355,0x0}, //,Initial Upload
	{0x3356,0x10}, //,Initial Upload
	{0x3357,0x84}, //,Initial Upload
	{0x3358,0x16}, //,Initial Upload
	{0x3359,0x1D}, //,Initial Upload
	{0x335A,0x80}, //,Initial Upload
	{0x335B,0x2}, //,Initial Upload
	{0x335C,0x10}, //,Initial Upload
	{0x335D,0xC4}, //,Initial Upload
	{0x335E,0x14}, //,Initial Upload
	{0x335F,0x1D}, //,Initial Upload
	{0x3360,0xA5}, //,Initial Upload
	{0x3361,0x0}, //,Initial Upload
	{0x3362,0x10}, //,Initial Upload
	{0x3363,0x84}, //,Initial Upload
	{0x3364,0x16}, //,Initial Upload
	{0x3365,0x1D}, //,Initial Upload
	{0x3366,0x1}, //,Initial Upload
	{0x3367,0x0}, //,Initial Upload
	{0x3368,0x90}, //,Initial Upload
	{0x3369,0x84}, //,Initial Upload
	{0x336A,0x16}, //,Initial Upload
	{0x336B,0x1D}, //,Initial Upload
	{0x336C,0x12}, //,Initial Upload
	{0x336D,0x0}, //,Initial Upload
	{0x336E,0x10}, //,Initial Upload
	{0x336F,0x84}, //,Initial Upload
	{0x3370,0x16}, //,Initial Upload
	{0x3371,0x15}, //,Initial Upload
	{0x3372,0x32}, //,Initial Upload
	{0x3373,0x0}, //,Initial Upload
	{0x3374,0x30}, //,Initial Upload
	{0x3375,0x84}, //,Initial Upload
	{0x3376,0x16}, //,Initial Upload
	{0x3377,0x15}, //,Initial Upload
	{0x3378,0x26}, //,Initial Upload
	{0x3379,0x0}, //,Initial Upload
	{0x337A,0x10}, //,Initial Upload
	{0x337B,0x84}, //,Initial Upload
	{0x337C,0x16}, //,Initial Upload
	{0x337D,0x15}, //,Initial Upload
	{0x337E,0x80}, //,Initial Upload
	{0x337F,0x2}, //,Initial Upload
	{0x3380,0x10}, //,Initial Upload
	{0x3381,0xC4}, //,Initial Upload
	{0x3382,0x14}, //,Initial Upload
	{0x3383,0x15}, //,Initial Upload
	{0x3384,0xA9}, //,Initial Upload
	{0x3385,0x0}, //,Initial Upload
	{0x3386,0x10}, //,Initial Upload
	{0x3387,0x84}, //,Initial Upload
	{0x3388,0x16}, //,Initial Upload
	{0x3389,0x15}, //,Initial Upload
	{0x338A,0x41}, //,Initial Upload
	{0x338B,0x0}, //,Initial Upload
	{0x338C,0x10}, //,Initial Upload
	{0x338D,0x80}, //,Initial Upload
	{0x338E,0x16}, //,Initial Upload
	{0x338F,0x15}, //,Initial Upload
	{0x3390,0x2}, //,Initial Upload
	{0x3391,0x0}, //,Initial Upload
	{0x3392,0x10}, //,Initial Upload
	{0x3393,0xA0}, //,Initial Upload
	{0x3394,0x16}, //,Initial Upload
	{0x3395,0x15}, //,Initial Upload
	{0x305C,0x18}, //,Initial Upload
	{0x305D,0x0}, //,Initial Upload
	{0x305E,0x19}, //,Initial Upload
	{0x305F,0x0}, //,Initial Upload
	{0x3396,0x1}, //,Initial Upload
	{0x3397,0x0}, //,Initial Upload
	{0x3398,0x90}, //,Initial Upload
	{0x3399,0x30}, //,Initial Upload
	{0x339A,0x56}, //,Initial Upload
	{0x339B,0x57}, //,Initial Upload
	{0x339C,0x1}, //,Initial Upload
	{0x339D,0x0}, //,Initial Upload
	{0x339E,0x10}, //,Initial Upload
	{0x339F,0x20}, //,Initial Upload
	{0x33A0,0xD6}, //,Initial Upload
	{0x33A1,0x17}, //,Initial Upload
	{0x33A2,0x1}, //,Initial Upload
	{0x33A3,0x0}, //,Initial Upload
	{0x33A4,0x10}, //,Initial Upload
	{0x33A5,0x28}, //,Initial Upload
	{0x33A6,0xD6}, //,Initial Upload
	{0x33A7,0x17}, //,Initial Upload
	{0x33A8,0x3}, //,Initial Upload
	{0x33A9,0x0}, //,Initial Upload
	{0x33AA,0x10}, //,Initial Upload
	{0x33AB,0x20}, //,Initial Upload
	{0x33AC,0xD6}, //,Initial Upload
	{0x33AD,0x17}, //,Initial Upload
	{0x33AE,0x61}, //,Initial Upload
	{0x33AF,0x0}, //,Initial Upload
	{0x33B0,0x10}, //,Initial Upload
	{0x33B1,0x20}, //,Initial Upload
	{0x33B2,0xD6}, //,Initial Upload
	{0x33B3,0x15}, //,Initial Upload
	{0x33B4,0x1}, //,Initial Upload
	{0x33B5,0x0}, //,Initial Upload
	{0x33B6,0x10}, //,Initial Upload
	{0x33B7,0x20}, //,Initial Upload
	{0x33B8,0xD6}, //,Initial Upload
	{0x33B9,0x1D}, //,Initial Upload
	{0x33BA,0x1}, //,Initial Upload
	{0x33BB,0x0}, //,Initial Upload
	{0x33BC,0x50}, //,Initial Upload
	{0x33BD,0x20}, //,Initial Upload
	{0x33BE,0xD6}, //,Initial Upload
	{0x33BF,0x1D}, //,Initial Upload
	{0x33C0,0x2C}, //,Initial Upload
	{0x33C1,0x0}, //,Initial Upload
	{0x33C2,0x10}, //,Initial Upload
	{0x33C3,0x20}, //,Initial Upload
	{0x33C4,0xD6}, //,Initial Upload
	{0x33C5,0x1D}, //,Initial Upload
	{0x33C6,0x1}, //,Initial Upload
	{0x33C7,0x0}, //,Initial Upload
	{0x33C8,0x90}, //,Initial Upload
	{0x33C9,0x20}, //,Initial Upload
	{0x33CA,0xD6}, //,Initial Upload
	{0x33CB,0x1D}, //,Initial Upload
	{0x33CC,0x83}, //,Initial Upload
	{0x33CD,0x0}, //,Initial Upload
	{0x33CE,0x10}, //,Initial Upload
	{0x33CF,0x20}, //,Initial Upload
	{0x33D0,0xD6}, //,Initial Upload
	{0x33D1,0x15}, //,Initial Upload
	{0x33D2,0x1}, //,Initial Upload
	{0x33D3,0x0}, //,Initial Upload
	{0x33D4,0x10}, //,Initial Upload
	{0x33D5,0x30}, //,Initial Upload
	{0x33D6,0xD6}, //,Initial Upload
	{0x33D7,0x15}, //,Initial Upload
	{0x33D8,0x1}, //,Initial Upload
	{0x33D9,0x0}, //,Initial Upload
	{0x33DA,0x10}, //,Initial Upload
	{0x33DB,0x20}, //,Initial Upload
	{0x33DC,0xD6}, //,Initial Upload
	{0x33DD,0x15}, //,Initial Upload
	{0x33DE,0x1}, //,Initial Upload
	{0x33DF,0x0}, //,Initial Upload
	{0x33E0,0x10}, //,Initial Upload
	{0x33E1,0x20}, //,Initial Upload
	{0x33E2,0x56}, //,Initial Upload
	{0x33E3,0x15}, //,Initial Upload
	{0x33E4,0x7}, //,Initial Upload
	{0x33E5,0x0}, //,Initial Upload
	{0x33E6,0x10}, //,Initial Upload
	{0x33E7,0x20}, //,Initial Upload
	{0x33E8,0x16}, //,Initial Upload
	{0x33E9,0x15}, //,Initial Upload
	{0x3060,0x26}, //,Initial Upload
	{0x3061,0x0}, //,Initial Upload
	{0x302A,0xFF}, //,Initial Upload
	{0x302B,0xFF}, //,Initial Upload
	{0x302C,0xFF}, //,Initial Upload
	{0x302D,0xFF}, //,Initial Upload
	{0x302E,0x3F}, //,Initial Upload
	{0x3013,0xB}, //,Initial Upload
	{0x102B,0x2C}, //,Initial Upload
	{0x102C,0x1}, //,Initial Upload
	{0x1035,0x54}, //,Initial Upload
	{0x1036,0x0}, //,Initial Upload
	{0x3090,0x2A}, //,Initial Upload
	{0x3091,0x1}, //,Initial Upload
	{0x30C6,0x5}, //,Initial Upload
	{0x30C7,0x0}, //,Initial Upload
	{0x30C8,0x0}, //,Initial Upload
	{0x30C9,0x0}, //,Initial Upload
	{0x30CA,0x0}, //,Initial Upload
	{0x30CB,0x0}, //,Initial Upload
	{0x30CC,0x0}, //,Initial Upload
	{0x30CD,0x0}, //,Initial Upload
	{0x30CE,0x0}, //,Initial Upload
	{0x30CF,0x5}, //,Initial Upload
	{0x30D0,0x0}, //,Initial Upload
	{0x30D1,0x0}, //,Initial Upload
	{0x30D2,0x0}, //,Initial Upload
	{0x30D3,0x0}, //,Initial Upload
	{0x30D4,0x0}, //,Initial Upload
	{0x30D5,0x0}, //,Initial Upload
	{0x30D6,0x0}, //,Initial Upload
	{0x30D7,0x0}, //,Initial Upload
	{0x30F3,0x5}, //,Initial Upload
	{0x30F4,0x0}, //,Initial Upload
	{0x30F5,0x0}, //,Initial Upload
	{0x30F6,0x0}, //,Initial Upload
	{0x30F7,0x0}, //,Initial Upload
	{0x30F8,0x0}, //,Initial Upload
	{0x30F9,0x0}, //,Initial Upload
	{0x30FA,0x0}, //,Initial Upload
	{0x30FB,0x0}, //,Initial Upload
	{0x30D8,0x5}, //,Initial Upload
	{0x30D9,0x0}, //,Initial Upload
	{0x30DA,0x0}, //,Initial Upload
	{0x30DB,0x0}, //,Initial Upload
	{0x30DC,0x0}, //,Initial Upload
	{0x30DD,0x0}, //,Initial Upload
	{0x30DE,0x0}, //,Initial Upload
	{0x30DF,0x0}, //,Initial Upload
	{0x30E0,0x0}, //,Initial Upload
	{0x30E1,0x5}, //,Initial Upload
	{0x30E2,0x0}, //,Initial Upload
	{0x30E3,0x0}, //,Initial Upload
	{0x30E4,0x0}, //,Initial Upload
	{0x30E5,0x0}, //,Initial Upload
	{0x30E6,0x0}, //,Initial Upload
	{0x30E7,0x0}, //,Initial Upload
	{0x30E8,0x0}, //,Initial Upload
	{0x30E9,0x0}, //,Initial Upload
	{0x30F3,0x5}, //,Initial Upload
	{0x30F4,0x2}, //,Initial Upload
	{0x30F5,0x0}, //,Initial Upload
	{0x30F6,0x17}, //,Initial Upload
	{0x30F7,0x1}, //,Initial Upload
	{0x30F8,0x0}, //,Initial Upload
	{0x30F9,0x0}, //,Initial Upload
	{0x30FA,0x0}, //,Initial Upload
	{0x30FB,0x0}, //,Initial Upload
	{0x30D8,0x3}, //,Initial Upload
	{0x30D9,0x1}, //,Initial Upload
	{0x30DA,0x0}, //,Initial Upload
	{0x30DB,0x19}, //,Initial Upload
	{0x30DC,0x1}, //,Initial Upload
	{0x30DD,0x0}, //,Initial Upload
	{0x30DE,0x0}, //,Initial Upload
	{0x30DF,0x0}, //,Initial Upload
	{0x30E0,0x0}, //,Initial Upload
	{0x30A2,0x5}, //,Initial Upload
	{0x30A3,0x2}, //,Initial Upload
	{0x30A4,0x0}, //,Initial Upload
	{0x30A5,0x22}, //,Initial Upload
	{0x30A6,0x0}, //,Initial Upload
	{0x30A7,0x0}, //,Initial Upload
	{0x30A8,0x0}, //,Initial Upload
	{0x30A9,0x0}, //,Initial Upload
	{0x30AA,0x0}, //,Initial Upload
	{0x30AB,0x5}, //,Initial Upload
	{0x30AC,0x2}, //,Initial Upload
	{0x30AD,0x0}, //,Initial Upload
	{0x30AE,0x22}, //,Initial Upload
	{0x30AF,0x0}, //,Initial Upload
	{0x30B0,0x0}, //,Initial Upload
	{0x30B1,0x0}, //,Initial Upload
	{0x30B2,0x0}, //,Initial Upload
	{0x30B3,0x0}, //,Initial Upload
	{0x30BD,0x5}, //,Initial Upload
	{0x30BE,0x9F}, //,Initial Upload
	{0x30BF,0x0}, //,Initial Upload
	{0x30C0,0x7D}, //,Initial Upload
	{0x30C1,0x0}, //,Initial Upload
	{0x30C2,0x0}, //,Initial Upload
	{0x30C3,0x0}, //,Initial Upload
	{0x30C4,0x0}, //,Initial Upload
	{0x30C5,0x0}, //,Initial Upload
	{0x30B4,0x4}, //,Initial Upload
	{0x30B5,0x9C}, //,Initial Upload
	{0x30B6,0x0}, //,Initial Upload
	{0x30B7,0x7D}, //,Initial Upload
	{0x30B8,0x0}, //,Initial Upload
	{0x30B9,0x0}, //,Initial Upload
	{0x30BA,0x0}, //,Initial Upload
	{0x30BB,0x0}, //,Initial Upload
	{0x30BC,0x0}, //,Initial Upload
	{0x30FC,0x5}, //,Initial Upload
	{0x30FD,0x0}, //,Initial Upload
	{0x30FE,0x0}, //,Initial Upload
	{0x30FF,0x0}, //,Initial Upload
	{0x3100,0x0}, //,Initial Upload
	{0x3101,0x0}, //,Initial Upload
	{0x3102,0x0}, //,Initial Upload
	{0x3103,0x0}, //,Initial Upload
	{0x3104,0x0}, //,Initial Upload
	{0x3105,0x5}, //,Initial Upload
	{0x3106,0x0}, //,Initial Upload
	{0x3107,0x0}, //,Initial Upload
	{0x3108,0x0}, //,Initial Upload
	{0x3109,0x0}, //,Initial Upload
	{0x310A,0x0}, //,Initial Upload
	{0x310B,0x0}, //,Initial Upload
	{0x310C,0x0}, //,Initial Upload
	{0x310D,0x0}, //,Initial Upload
	{0x3099,0x5}, //,Initial Upload
	{0x309A,0x96}, //,Initial Upload
	{0x309B,0x0}, //,Initial Upload
	{0x309C,0x6}, //,Initial Upload
	{0x309D,0x0}, //,Initial Upload
	{0x309E,0x0}, //,Initial Upload
	{0x309F,0x0}, //,Initial Upload
	{0x30A0,0x0}, //,Initial Upload
	{0x30A1,0x0}, //,Initial Upload
	{0x310E,0x5}, //,Initial Upload
	{0x310F,0x2}, //,Initial Upload
	{0x3110,0x0}, //,Initial Upload
	{0x3111,0x2B}, //,Initial Upload
	{0x3112,0x0}, //,Initial Upload
	{0x3113,0x0}, //,Initial Upload
	{0x3114,0x0}, //,Initial Upload
	{0x3115,0x0}, //,Initial Upload
	{0x3116,0x0}, //,Initial Upload
	{0x3117,0x5}, //,Initial Upload
	{0x3118,0x2}, //,Initial Upload
	{0x3119,0x0}, //,Initial Upload
	{0x311A,0x2C}, //,Initial Upload
	{0x311B,0x0}, //,Initial Upload
	{0x311C,0x0}, //,Initial Upload
	{0x311D,0x0}, //,Initial Upload
	{0x311E,0x0}, //,Initial Upload
	{0x311F,0x0}, //,Initial Upload
	{0x30EA,0x0}, //,Initial Upload
	{0x30EB,0x0}, //,Initial Upload
	{0x30EC,0x0}, //,Initial Upload
	{0x30ED,0x0}, //,Initial Upload
	{0x30EE,0x0}, //,Initial Upload
	{0x30EF,0x0}, //,Initial Upload
	{0x30F0,0x0}, //,Initial Upload
	{0x30F1,0x0}, //,Initial Upload
	{0x30F2,0x0}, //,Initial Upload
	{0x313B,0x3}, //,Initial Upload
	{0x313C,0x31}, //,Initial Upload
	{0x313D,0x0}, //,Initial Upload
	{0x313E,0x7}, //,Initial Upload
	{0x313F,0x0}, //,Initial Upload
	{0x3140,0x68}, //,Initial Upload
	{0x3141,0x0}, //,Initial Upload
	{0x3142,0x34}, //,Initial Upload
	{0x3143,0x0}, //,Initial Upload
	{0x31A0,0x3}, //,Initial Upload
	{0x31A1,0x16}, //,Initial Upload
	{0x31A2,0x0}, //,Initial Upload
	{0x31A3,0x8}, //,Initial Upload
	{0x31A4,0x0}, //,Initial Upload
	{0x31A5,0x7E}, //,Initial Upload
	{0x31A6,0x0}, //,Initial Upload
	{0x31A7,0x8}, //,Initial Upload
	{0x31A8,0x0}, //,Initial Upload
	{0x31A9,0x3}, //,Initial Upload
	{0x31AA,0x16}, //,Initial Upload
	{0x31AB,0x0}, //,Initial Upload
	{0x31AC,0x8}, //,Initial Upload
	{0x31AD,0x0}, //,Initial Upload
	{0x31AE,0x7E}, //,Initial Upload
	{0x31AF,0x0}, //,Initial Upload
	{0x31B0,0x8}, //,Initial Upload
	{0x31B1,0x0}, //,Initial Upload
	{0x31B2,0x3}, //,Initial Upload
	{0x31B3,0x16}, //,Initial Upload
	{0x31B4,0x0}, //,Initial Upload
	{0x31B5,0x8}, //,Initial Upload
	{0x31B6,0x0}, //,Initial Upload
	{0x31B7,0x7E}, //,Initial Upload
	{0x31B8,0x0}, //,Initial Upload
	{0x31B9,0x8}, //,Initial Upload
	{0x31BA,0x0}, //,Initial Upload
	{0x3120,0x5}, //,Initial Upload
	{0x3121,0x45}, //,Initial Upload
	{0x3122,0x0}, //,Initial Upload
	{0x3123,0x1D}, //,Initial Upload
	{0x3124,0x0}, //,Initial Upload
	{0x3125,0xA9}, //,Initial Upload
	{0x3126,0x0}, //,Initial Upload
	{0x3127,0x6D}, //,Initial Upload
	{0x3128,0x0}, //,Initial Upload
	{0x3129,0x5}, //,Initial Upload
	{0x312A,0x15}, //,Initial Upload
	{0x312B,0x0}, //,Initial Upload
	{0x312C,0xA}, //,Initial Upload
	{0x312D,0x0}, //,Initial Upload
	{0x312E,0x45}, //,Initial Upload
	{0x312F,0x0}, //,Initial Upload
	{0x3130,0x1D}, //,Initial Upload
	{0x3131,0x0}, //,Initial Upload
	{0x3132,0x5}, //,Initial Upload
	{0x3133,0x7D}, //,Initial Upload
	{0x3134,0x0}, //,Initial Upload
	{0x3135,0xA}, //,Initial Upload
	{0x3136,0x0}, //,Initial Upload
	{0x3137,0xA9}, //,Initial Upload
	{0x3138,0x0}, //,Initial Upload
	{0x3139,0x6D}, //,Initial Upload
	{0x313A,0x0}, //,Initial Upload
	{0x3144,0x5}, //,Initial Upload
	{0x3145,0x0}, //,Initial Upload
	{0x3146,0x0}, //,Initial Upload
	{0x3147,0x30}, //,Initial Upload
	{0x3148,0x0}, //,Initial Upload
	{0x3149,0x0}, //,Initial Upload
	{0x314A,0x0}, //,Initial Upload
	{0x314B,0x0}, //,Initial Upload
	{0x314C,0x0}, //,Initial Upload
	{0x314D,0x3}, //,Initial Upload
	{0x314E,0x0}, //,Initial Upload
	{0x314F,0x0}, //,Initial Upload
	{0x3150,0x31}, //,Initial Upload
	{0x3151,0x0}, //,Initial Upload
	{0x3152,0x0}, //,Initial Upload
	{0x3153,0x0}, //,Initial Upload
	{0x3154,0x0}, //,Initial Upload
	{0x3155,0x0}, //,Initial Upload
	{0x31D8,0x5}, //,Initial Upload
	{0x31D9,0x3A}, //,Initial Upload
	{0x31DA,0x0}, //,Initial Upload
	{0x31DB,0x2E}, //,Initial Upload
	{0x31DC,0x0}, //,Initial Upload
	{0x31DD,0x9E}, //,Initial Upload
	{0x31DE,0x0}, //,Initial Upload
	{0x31DF,0x7E}, //,Initial Upload
	{0x31E0,0x0}, //,Initial Upload
	{0x31E1,0x5}, //,Initial Upload
	{0x31E2,0x4}, //,Initial Upload
	{0x31E3,0x0}, //,Initial Upload
	{0x31E4,0x4}, //,Initial Upload
	{0x31E5,0x0}, //,Initial Upload
	{0x31E6,0x73}, //,Initial Upload
	{0x31E7,0x0}, //,Initial Upload
	{0x31E8,0x4}, //,Initial Upload
	{0x31E9,0x0}, //,Initial Upload
	{0x31EA,0x5}, //,Initial Upload
	{0x31EB,0x0}, //,Initial Upload
	{0x31EC,0x0}, //,Initial Upload
	{0x31ED,0x0}, //,Initial Upload
	{0x31EE,0x0}, //,Initial Upload
	{0x31EF,0x0}, //,Initial Upload
	{0x31F0,0x0}, //,Initial Upload
	{0x31F1,0x0}, //,Initial Upload
	{0x31F2,0x0}, //,Initial Upload
	{0x31F3,0x0}, //,Initial Upload
	{0x31F4,0x0}, //,Initial Upload
	{0x31F5,0x0}, //,Initial Upload
	{0x31F6,0x0}, //,Initial Upload
	{0x31F7,0x0}, //,Initial Upload
	{0x31F8,0x0}, //,Initial Upload
	{0x31F9,0x0}, //,Initial Upload
	{0x31FA,0x0}, //,Initial Upload
	{0x31FB,0x5}, //,Initial Upload
	{0x31FC,0x0}, //,Initial Upload
	{0x31FD,0x0}, //,Initial Upload
	{0x31FE,0x0}, //,Initial Upload
	{0x31FF,0x0}, //,Initial Upload
	{0x3200,0x0}, //,Initial Upload
	{0x3201,0x0}, //,Initial Upload
	{0x3202,0x0}, //,Initial Upload
	{0x3203,0x0}, //,Initial Upload
	{0x3204,0x0}, //,Initial Upload
	{0x3205,0x0}, //,Initial Upload
	{0x3206,0x0}, //,Initial Upload
	{0x3207,0x0}, //,Initial Upload
	{0x3208,0x0}, //,Initial Upload
	{0x3209,0x0}, //,Initial Upload
	{0x320A,0x0}, //,Initial Upload
	{0x320B,0x0}, //,Initial Upload
	{0x3164,0x5}, //,Initial Upload
	{0x3165,0x14}, //,Initial Upload
	{0x3166,0x0}, //,Initial Upload
	{0x3167,0xC}, //,Initial Upload
	{0x3168,0x0}, //,Initial Upload
	{0x3169,0x44}, //,Initial Upload
	{0x316A,0x0}, //,Initial Upload
	{0x316B,0x1F}, //,Initial Upload
	{0x316C,0x0}, //,Initial Upload
	{0x316D,0x5}, //,Initial Upload
	{0x316E,0x7C}, //,Initial Upload
	{0x316F,0x0}, //,Initial Upload
	{0x3170,0xC}, //,Initial Upload
	{0x3171,0x0}, //,Initial Upload
	{0x3172,0xA8}, //,Initial Upload
	{0x3173,0x0}, //,Initial Upload
	{0x3174,0x6F}, //,Initial Upload
	{0x3175,0x0}, //,Initial Upload
	{0x31C4,0x5}, //,Initial Upload
	{0x31C5,0x24}, //,Initial Upload
	{0x31C6,0x1}, //,Initial Upload
	{0x31C7,0x4}, //,Initial Upload
	{0x31C8,0x0}, //,Initial Upload
	{0x31C9,0x5}, //,Initial Upload
	{0x31CA,0x24}, //,Initial Upload
	{0x31CB,0x1}, //,Initial Upload
	{0x31CC,0x4}, //,Initial Upload
	{0x31CD,0x0}, //,Initial Upload
	{0x31CE,0x5}, //,Initial Upload
	{0x31CF,0x24}, //,Initial Upload
	{0x31D0,0x1}, //,Initial Upload
	{0x31D1,0x4}, //,Initial Upload
	{0x31D2,0x0}, //,Initial Upload
	{0x31D3,0x5}, //,Initial Upload
	{0x31D4,0x73}, //,Initial Upload
	{0x31D5,0x0}, //,Initial Upload
	{0x31D6,0xB1}, //,Initial Upload
	{0x31D7,0x0}, //,Initial Upload
	{0x3176,0x5}, //,Initial Upload
	{0x3177,0x10}, //,Initial Upload
	{0x3178,0x0}, //,Initial Upload
	{0x3179,0x56}, //,Initial Upload
	{0x317A,0x0}, //,Initial Upload
	{0x317B,0x0}, //,Initial Upload
	{0x317C,0x0}, //,Initial Upload
	{0x317D,0x0}, //,Initial Upload
	{0x317E,0x0}, //,Initial Upload
	{0x317F,0x5}, //,Initial Upload
	{0x3180,0x6A}, //,Initial Upload
	{0x3181,0x0}, //,Initial Upload
	{0x3182,0xAD}, //,Initial Upload
	{0x3183,0x0}, //,Initial Upload
	{0x3184,0x0}, //,Initial Upload
	{0x3185,0x0}, //,Initial Upload
	{0x3186,0x0}, //,Initial Upload
	{0x3187,0x0}, //,Initial Upload
	{0x100C,0x7E}, //,Initial Upload
	{0x100D,0x0}, //,Initial Upload
	{0x1012,0xDF}, //,Initial Upload
	{0x1013,0x2B}, //,Initial Upload
	{0x1002,0x4}, //,Initial Upload
	// {0x1003,0x10}, //,Sensor Control Mode.IMAGER_STATE(0)
	{0x0043,0x0}, //,Sensor Control Mode.SLEEP_POWER_MODE(0)
	{0x0043,0x0}, //,Sensor Control Mode.IDLE_POWER_MODE(0)
	{0x0043,0x4}, //,Sensor Control Mode.SYSTEM_CLOCK_ENABLE(0)
	{0x0043,0xC}, //,Sensor Control Mode.SRAM_CLOCK_ENABLE(0)
	{0x1002,0x4}, //,Sensor Control Mode.IMAGER_RUN_CONT(0)
	{0x1001,0x41}, //,Sensor Control Mode.EXT_EVENT_SEL(0)
	{0x10F2,0x1}, //,Sensor Control Mode.NB_OF_FRAMES_A(0)
	{0x10F3,0x0}, //,Sensor Control Mode.NB_OF_FRAMES_A(1)
	{0x1111,0x1}, //,Sensor Control Mode.NB_OF_FRAMES_B(0)
	{0x1112,0x0}, //,Sensor Control Mode.NB_OF_FRAMES_B(1)
	{0x0012,0x0}, //,IO Drive Strength.DIG_DRIVE_STRENGTH(0)
	{0x0012,0x0}, //,IO Drive Strength.CCI_DRIVE_STRENGTH(0)
	{0x1001,0x41}, //,Readout && Exposure.EXT_EXP_PW_SEL(0)
	{0x10D0,0x0}, //,Readout && Exposure.EXT_EXP_PW_DELAY(0)
	{0x10D1,0x0}, //,Readout && Exposure.EXT_EXP_PW_DELAY(1)
	{0x1012,0x91}, //,Readout && Exposure.VBLANK_A(0)
	{0x1013,0xD}, //,Readout && Exposure.VBLANK_A(1)
	{0x1103,0x91}, //,Readout && Exposure.VBLANK_B(0)
	{0x1104,0xD}, //,Readout && Exposure.VBLANK_B(1)
	{0x100C,0x80}, //,Readout && Exposure.EXP_TIME_A(0)
	{0x100D,0x0}, //,Readout && Exposure.EXP_TIME_A(1)
	{0x1115,0x80}, //,Readout && Exposure.EXP_TIME_B(0)
	{0x1116,0x0}, //,Readout && Exposure.EXP_TIME_B(1)
	{0x102B,0x30}, //,Readout && Exposure.ROW_LENGTH_A(0)
	{0x102C,0x1}, //,Readout && Exposure.ROW_LENGTH_A(1)
	{0x1113,0x30}, //,Readout && Exposure.ROW_LENGTH_B(0)
	{0x1114,0x1}, //,Readout && Exposure.ROW_LENGTH_B(1)
	{0x2008,0x20}, //,Horizontal ROI.HSIZE_A(0)
	{0x2009,0x3}, //,Horizontal ROI.HSIZE_A(1)
	{0x2098,0x20}, //,Horizontal ROI.HSIZE_B(0)
	{0x2099,0x3}, //,Horizontal ROI.HSIZE_B(1)
	{0x200A,0x0}, //,Horizontal ROI.HSTART_A(0)
	{0x200B,0x0}, //,Horizontal ROI.HSTART_A(1)
	{0x209A,0x0}, //,Horizontal ROI.HSTART_B(0)
	{0x209B,0x0}, //,Horizontal ROI.HSTART_B(1)
	{0x207D,0x40}, //,Horizontal ROI.MIPI_HSIZE(0)
	{0x207E,0x6}, //,Horizontal ROI.MIPI_HSIZE(1)
	{0x107D,0x0}, //,Vertical ROI.VSTART0_A(0)
	{0x107E,0x0}, //,Vertical ROI.VSTART0_A(1)
	{0x1087,0x78}, //,Vertical ROI.VSIZE0_A(0)
	{0x1088,0x5}, //,Vertical ROI.VSIZE0_A(1)
	{0x1105,0x0}, //,Vertical ROI.VSTART0_B(0)
	{0x1106,0x0}, //,Vertical ROI.VSTART0_B(1)
	{0x110A,0x78}, //,Vertical ROI.VSIZE0_B(0)
	{0x110B,0x5}, //,Vertical ROI.VSIZE0_B(1)
	{0x107D,0x0}, //,Vertical ROI.VSTART1_A(0)
	{0x107E,0x0}, //,Vertical ROI.VSTART1_A(1)
	{0x107F,0x0}, //,Vertical ROI.VSTART1_A(2)
	{0x1087,0x78}, //,Vertical ROI.VSIZE1_A(0)
	{0x1088,0x5}, //,Vertical ROI.VSIZE1_A(1)
	{0x1089,0x0}, //,Vertical ROI.VSIZE1_A(2)
	{0x1105,0x0}, //,Vertical ROI.VSTART1_B(0)
	{0x1106,0x0}, //,Vertical ROI.VSTART1_B(1)
	{0x1107,0x0}, //,Vertical ROI.VSTART1_B(2)
	{0x110A,0x78}, //,Vertical ROI.VSIZE1_B(0)
	{0x110B,0x5}, //,Vertical ROI.VSIZE1_B(1)
	{0x110C,0x0}, //,Vertical ROI.VSIZE1_B(2)
	{0x107D,0x0}, //,Vertical ROI.VSTART2_A(0)
	{0x107E,0x0}, //,Vertical ROI.VSTART2_A(1)
	{0x107F,0x0}, //,Vertical ROI.VSTART2_A(2)
	{0x1080,0x0}, //,Vertical ROI.VSTART2_A(3)
	{0x1081,0x0}, //,Vertical ROI.VSTART2_A(4)
	{0x1087,0x78}, //,Vertical ROI.VSIZE2_A(0)
	{0x1088,0x5}, //,Vertical ROI.VSIZE2_A(1)
	{0x1089,0x0}, //,Vertical ROI.VSIZE2_A(2)
	{0x108A,0x0}, //,Vertical ROI.VSIZE2_A(3)
	{0x108B,0x0}, //,Vertical ROI.VSIZE2_A(4)
	{0x1105,0x0}, //,Vertical ROI.VSTART2_B(0)
	{0x1106,0x0}, //,Vertical ROI.VSTART2_B(1)
	{0x1107,0x0}, //,Vertical ROI.VSTART2_B(2)
	{0x1108,0x0}, //,Vertical ROI.VSTART2_B(3)
	{0x1109,0x0}, //,Vertical ROI.VSTART2_B(4)
	{0x110A,0x78}, //,Vertical ROI.VSIZE2_B(0)
	{0x110B,0x5}, //,Vertical ROI.VSIZE2_B(1)
	{0x110C,0x0}, //,Vertical ROI.VSIZE2_B(2)
	{0x110D,0x0}, //,Vertical ROI.VSIZE2_B(3)
	{0x110E,0x0}, //,Vertical ROI.VSIZE2_B(4)
	{0x209C,0x0}, //,Mirroring && Flipping.HFLIP_A(0)
	{0x209D,0x0}, //,Mirroring && Flipping.HFLIP_B(0)
	{0x1095,0x0}, //,Mirroring && Flipping.VFLIP(0)
	{0x2063,0x0}, //,Mirroring && Flipping.BIT_ORDER(0)
	{0x6006,0x0}, //,MIPI.TX_CTRL_EN(0)
	{0x5004,0x1}, //,MIPI.datarate
	{0x5086,0x2}, //,MIPI.datarate
	{0x5087,0x4E}, //,MIPI.datarate
	{0x5088,0x0}, //,MIPI.datarate
	{0x5090,0x0}, //,MIPI.datarate
	{0x5091,0x8}, //,MIPI.datarate
	{0x5092,0x14}, //,MIPI.datarate
	{0x5093,0xF}, //,MIPI.datarate
	{0x5094,0x6}, //,MIPI.datarate
	{0x5095,0x32}, //,MIPI.datarate
	{0x5096,0xE}, //,MIPI.datarate
	{0x5097,0x0}, //,MIPI.datarate
	{0x5098,0x11}, //,MIPI.datarate
	{0x5004,0x0}, //,MIPI.datarate
	{0x2066,0x6C}, //,MIPI.datarate
	{0x2067,0x7}, //,MIPI.datarate
	{0x206E,0x7E}, //,MIPI.datarate
	{0x206F,0x6}, //,MIPI.datarate
	{0x20AC,0x7E}, //,MIPI.datarate
	{0x20AD,0x6}, //,MIPI.datarate
	{0x2076,0xC8}, //,MIPI.datarate
	{0x2077,0x0}, //,MIPI.datarate
	{0x20B4,0xC8}, //,MIPI.datarate
	{0x20B5,0x0}, //,MIPI.datarate
	{0x2078,0x1E}, //,MIPI.datarate
	{0x2079,0x4}, //,MIPI.datarate
	{0x20B6,0x1E}, //,MIPI.datarate
	{0x20B7,0x4}, //,MIPI.datarate
	{0x207A,0xD4}, //,MIPI.datarate
	{0x207B,0x4}, //,MIPI.datarate
	{0x20B8,0xD4}, //,MIPI.datarate
	{0x20B9,0x4}, //,MIPI.datarate
	{0x208D,0x4}, //,MIPI.CSI2_DTYPE(0)
	{0x208E,0x0}, //,MIPI.CSI2_DTYPE(1)
	{0x207C,0x0}, //,MIPI.VC_ID(0)
	{0x6001,0x7}, //,MIPI.TINIT(0)
	{0x6002,0xD8}, //,MIPI.TINIT(1)
	{0x6010,0x0}, //,MIPI.FRAME_MODE(0)
	{0x6010,0x0}, //,MIPI.EMBEDDED_FRAME_MODE(0)
	{0x6011,0x0}, //,MIPI.DATA_ENABLE_POLARITY(0)
	{0x6011,0x0}, //,MIPI.HSYNC_POLARITY(0)
	{0x6011,0x0}, //,MIPI.VSYNC_POLARITY(0)
	{0x6012,0x1}, //,MIPI.LANE(0)
	{0x6013,0x0}, //,MIPI.CLK_MODE(0)
	{0x6016,0x0}, //,MIPI.FRAME_COUNTER(0)
	{0x6017,0x0}, //,MIPI.FRAME_COUNTER(1)
	{0x6037,0x1}, //,MIPI.LINE_COUNT_RAW8(0)
	{0x6037,0x3}, //,MIPI.LINE_COUNT_RAW10(0)
	{0x6037,0x7}, //,MIPI.LINE_COUNT_RAW12(0)
	{0x6039,0x1}, //,MIPI.LINE_COUNT_EMB(0)
	{0x6018,0x0}, //,MIPI.CCI_READ_INTERRUPT_EN(0)
	{0x6018,0x0}, //,MIPI.CCI_WRITE_INTERRUPT_EN(0)
	{0x6065,0x0}, //,MIPI.TWAKE_TIMER(0)
	{0x6066,0x0}, //,MIPI.TWAKE_TIMER(1)
	{0x601C,0x0}, //,MIPI.SKEW_CAL_EN(0)
	{0x601D,0x0}, //,MIPI.SKEW_COUNT(0)
	{0x601E,0x22}, //,MIPI.SKEW_COUNT(1)
	{0x601F,0x0}, //,MIPI.SCRAMBLING_EN(0)
	{0x6003,0x1}, //,MIPI.INIT_SKEW_EN(0)
	{0x6004,0x7A}, //,MIPI.INIT_SKEW(0)
	{0x6005,0x12}, //,MIPI.INIT_SKEW(1)
	{0x6006,0x1}, //,MIPI.TX_CTRL_EN(0)
	{0x4006,0x8}, //,Processing.BSP(0)
	{0x209E,0x2}, //,Processing.BIT_DEPTH(0)
	{0x2045,0x1}, //,Processing.CDS_RNC(0)
	{0x2048,0x1}, //,Processing.CDS_IMG(0)
	{0x204B,0x3}, //,Processing.RNC_EN(0)
	{0x205B,0x64}, //,Processing.RNC_DARK_TARGET(0)
	{0x205C,0x0}, //,Processing.RNC_DARK_TARGET(1)
	{0x24DC,0x12}, //,Defect Pixel Correction.DC_ENABLE(0)
	{0x24DC,0x10}, //,Defect Pixel Correction.DC_MODE(0)
	{0x24DC,0x0}, //,Defect Pixel Correction.DC_REPLACEMENT_VALUE(0)
	{0x24DD,0x0}, //,Defect Pixel Correction.DC_LIMIT_LOW(0)
	{0x24DE,0x0}, //,Defect Pixel Correction.DC_LIMIT_HIGH(0)
	{0x24DF,0x0}, //,Defect Pixel Correction.DC_LIMIT_HIGH_MODE(0)
	{0x10D7,0x1}, //,Illumination Trigger.ILLUM_EN(0)
	{0x10D8,0x2}, //,Illumination Trigger.ILLUM_POL(0)
	{0x205D,0x0}, //,Histogram.HIST_EN(0)
	{0x205E,0x0}, //,Histogram.HIST_USAGE_RATIO(0)
	{0x2063,0x0}, //,Histogram.PIXEL_DATA_SUPP(0)
	{0x2063,0x0}, //,Histogram.PIXEL_TRANSMISSION(0)
	{0x2091,0x0}, //,Test Pattern Generator.TPG_EN(0)
	{0x2091,0x0}, //,Test Pattern Generator.TPG_CONFIG(0)

};



// vga_640_480_120fps_12b_2lanes
static const struct mira220_reg vga_640_480_120fps_12b_2lanes_reg[] = {
	{0x1003,0x2}, //,Initial Upload
	{0x6006,0x0}, //,Initial Upload
	{0x6012,0x1}, //,Initial Upload
	{0x6013,0x0}, //,Initial Upload
	{0x6006,0x1}, //,Initial Upload
	{0x205D,0x0}, //,Initial Upload
	{0x2063,0x0}, //,Initial Upload
	{0x24DC,0x13}, //,Initial Upload
	{0x24DD,0x3}, //,Initial Upload
	{0x24DE,0x3}, //,Initial Upload
	{0x24DF,0x0}, //,Initial Upload
	{0x4006,0x8}, //,Initial Upload
	{0x401C,0x6F}, //,Initial Upload
	{0x204B,0x3}, //,Initial Upload
	{0x205B,0x64}, //,Initial Upload
	{0x205C,0x0}, //,Initial Upload
	{0x4018,0x3F}, //,Initial Upload
	{0x403B,0xB}, //,Initial Upload
	{0x403E,0xE}, //,Initial Upload
	{0x402B,0x6}, //,Initial Upload
	{0x401E,0x2}, //,Initial Upload
	{0x4038,0x3B}, //,Initial Upload
	{0x1077,0x0}, //,Initial Upload
	{0x1078,0x0}, //,Initial Upload
	{0x1009,0x8}, //,Initial Upload
	{0x100A,0x0}, //,Initial Upload
	{0x110F,0x8}, //,Initial Upload
	{0x1110,0x0}, //,Initial Upload
	{0x1006,0x2}, //,Initial Upload
	{0x402C,0x64}, //,Initial Upload
	{0x3064,0x0}, //,Initial Upload
	{0x3065,0xF0}, //,Initial Upload
	{0x4013,0x13}, //,Initial Upload
	{0x401F,0x9}, //,Initial Upload
	{0x4020,0x13}, //,Initial Upload
	{0x4044,0x75}, //,Initial Upload
	{0x4027,0x0}, //,Initial Upload
	{0x3215,0x69}, //,Initial Upload
	{0x3216,0xF}, //,Initial Upload
	{0x322B,0x69}, //,Initial Upload
	{0x322C,0xF}, //,Initial Upload
	{0x4051,0x80}, //,Initial Upload
	{0x4052,0x10}, //,Initial Upload
	{0x4057,0x80}, //,Initial Upload
	{0x4058,0x10}, //,Initial Upload
	{0x3212,0x59}, //,Initial Upload
	{0x4047,0x8F}, //,Initial Upload
	{0x4026,0x10}, //,Initial Upload
	{0x4032,0x53}, //,Initial Upload
	{0x4036,0x17}, //,Initial Upload
	{0x50B8,0xF4}, //,Initial Upload
	{0x3016,0x0}, //,Initial Upload
	{0x3017,0x2C}, //,Initial Upload
	{0x3018,0x8C}, //,Initial Upload
	{0x3019,0x45}, //,Initial Upload
	{0x301A,0x5}, //,Initial Upload
	{0x3013,0xA}, //,Initial Upload
	{0x301B,0x0}, //,Initial Upload
	{0x301C,0x4}, //,Initial Upload
	{0x301D,0x88}, //,Initial Upload
	{0x301E,0x45}, //,Initial Upload
	{0x301F,0x5}, //,Initial Upload
	{0x3020,0x0}, //,Initial Upload
	{0x3021,0x4}, //,Initial Upload
	{0x3022,0x88}, //,Initial Upload
	{0x3023,0x45}, //,Initial Upload
	{0x3024,0x5}, //,Initial Upload
	{0x3025,0x0}, //,Initial Upload
	{0x3026,0x4}, //,Initial Upload
	{0x3027,0x88}, //,Initial Upload
	{0x3028,0x45}, //,Initial Upload
	{0x3029,0x5}, //,Initial Upload
	{0x302F,0x0}, //,Initial Upload
	{0x3056,0x0}, //,Initial Upload
	{0x3057,0x0}, //,Initial Upload
	{0x3300,0x1}, //,Initial Upload
	{0x3301,0x0}, //,Initial Upload
	{0x3302,0xB0}, //,Initial Upload
	{0x3303,0xB0}, //,Initial Upload
	{0x3304,0x16}, //,Initial Upload
	{0x3305,0x15}, //,Initial Upload
	{0x3306,0x1}, //,Initial Upload
	{0x3307,0x0}, //,Initial Upload
	{0x3308,0x30}, //,Initial Upload
	{0x3309,0xA0}, //,Initial Upload
	{0x330A,0x16}, //,Initial Upload
	{0x330B,0x15}, //,Initial Upload
	{0x330C,0x1}, //,Initial Upload
	{0x330D,0x0}, //,Initial Upload
	{0x330E,0x30}, //,Initial Upload
	{0x330F,0xA0}, //,Initial Upload
	{0x3310,0x16}, //,Initial Upload
	{0x3311,0x15}, //,Initial Upload
	{0x3312,0x1}, //,Initial Upload
	{0x3313,0x0}, //,Initial Upload
	{0x3314,0x30}, //,Initial Upload
	{0x3315,0xA0}, //,Initial Upload
	{0x3316,0x16}, //,Initial Upload
	{0x3317,0x15}, //,Initial Upload
	{0x3318,0x1}, //,Initial Upload
	{0x3319,0x0}, //,Initial Upload
	{0x331A,0x30}, //,Initial Upload
	{0x331B,0xA0}, //,Initial Upload
	{0x331C,0x16}, //,Initial Upload
	{0x331D,0x15}, //,Initial Upload
	{0x331E,0x1}, //,Initial Upload
	{0x331F,0x0}, //,Initial Upload
	{0x3320,0x30}, //,Initial Upload
	{0x3321,0xA0}, //,Initial Upload
	{0x3322,0x16}, //,Initial Upload
	{0x3323,0x15}, //,Initial Upload
	{0x3324,0x1}, //,Initial Upload
	{0x3325,0x0}, //,Initial Upload
	{0x3326,0x30}, //,Initial Upload
	{0x3327,0xA0}, //,Initial Upload
	{0x3328,0x16}, //,Initial Upload
	{0x3329,0x15}, //,Initial Upload
	{0x332A,0x2B}, //,Initial Upload
	{0x332B,0x0}, //,Initial Upload
	{0x332C,0x30}, //,Initial Upload
	{0x332D,0xA0}, //,Initial Upload
	{0x332E,0x16}, //,Initial Upload
	{0x332F,0x15}, //,Initial Upload
	{0x3330,0x1}, //,Initial Upload
	{0x3331,0x0}, //,Initial Upload
	{0x3332,0x10}, //,Initial Upload
	{0x3333,0xA0}, //,Initial Upload
	{0x3334,0x16}, //,Initial Upload
	{0x3335,0x15}, //,Initial Upload
	{0x3058,0x8}, //,Initial Upload
	{0x3059,0x0}, //,Initial Upload
	{0x305A,0x9}, //,Initial Upload
	{0x305B,0x0}, //,Initial Upload
	{0x3336,0x1}, //,Initial Upload
	{0x3337,0x0}, //,Initial Upload
	{0x3338,0x90}, //,Initial Upload
	{0x3339,0xB0}, //,Initial Upload
	{0x333A,0x16}, //,Initial Upload
	{0x333B,0x15}, //,Initial Upload
	{0x333C,0x1F}, //,Initial Upload
	{0x333D,0x0}, //,Initial Upload
	{0x333E,0x10}, //,Initial Upload
	{0x333F,0xA0}, //,Initial Upload
	{0x3340,0x16}, //,Initial Upload
	{0x3341,0x15}, //,Initial Upload
	{0x3342,0x52}, //,Initial Upload
	{0x3343,0x0}, //,Initial Upload
	{0x3344,0x10}, //,Initial Upload
	{0x3345,0x80}, //,Initial Upload
	{0x3346,0x16}, //,Initial Upload
	{0x3347,0x15}, //,Initial Upload
	{0x3348,0x1}, //,Initial Upload
	{0x3349,0x0}, //,Initial Upload
	{0x334A,0x10}, //,Initial Upload
	{0x334B,0x80}, //,Initial Upload
	{0x334C,0x16}, //,Initial Upload
	{0x334D,0x1D}, //,Initial Upload
	{0x334E,0x1}, //,Initial Upload
	{0x334F,0x0}, //,Initial Upload
	{0x3350,0x50}, //,Initial Upload
	{0x3351,0x84}, //,Initial Upload
	{0x3352,0x16}, //,Initial Upload
	{0x3353,0x1D}, //,Initial Upload
	{0x3354,0x18}, //,Initial Upload
	{0x3355,0x0}, //,Initial Upload
	{0x3356,0x10}, //,Initial Upload
	{0x3357,0x84}, //,Initial Upload
	{0x3358,0x16}, //,Initial Upload
	{0x3359,0x1D}, //,Initial Upload
	{0x335A,0x80}, //,Initial Upload
	{0x335B,0x2}, //,Initial Upload
	{0x335C,0x10}, //,Initial Upload
	{0x335D,0xC4}, //,Initial Upload
	{0x335E,0x14}, //,Initial Upload
	{0x335F,0x1D}, //,Initial Upload
	{0x3360,0xA5}, //,Initial Upload
	{0x3361,0x0}, //,Initial Upload
	{0x3362,0x10}, //,Initial Upload
	{0x3363,0x84}, //,Initial Upload
	{0x3364,0x16}, //,Initial Upload
	{0x3365,0x1D}, //,Initial Upload
	{0x3366,0x1}, //,Initial Upload
	{0x3367,0x0}, //,Initial Upload
	{0x3368,0x90}, //,Initial Upload
	{0x3369,0x84}, //,Initial Upload
	{0x336A,0x16}, //,Initial Upload
	{0x336B,0x1D}, //,Initial Upload
	{0x336C,0x12}, //,Initial Upload
	{0x336D,0x0}, //,Initial Upload
	{0x336E,0x10}, //,Initial Upload
	{0x336F,0x84}, //,Initial Upload
	{0x3370,0x16}, //,Initial Upload
	{0x3371,0x15}, //,Initial Upload
	{0x3372,0x32}, //,Initial Upload
	{0x3373,0x0}, //,Initial Upload
	{0x3374,0x30}, //,Initial Upload
	{0x3375,0x84}, //,Initial Upload
	{0x3376,0x16}, //,Initial Upload
	{0x3377,0x15}, //,Initial Upload
	{0x3378,0x26}, //,Initial Upload
	{0x3379,0x0}, //,Initial Upload
	{0x337A,0x10}, //,Initial Upload
	{0x337B,0x84}, //,Initial Upload
	{0x337C,0x16}, //,Initial Upload
	{0x337D,0x15}, //,Initial Upload
	{0x337E,0x80}, //,Initial Upload
	{0x337F,0x2}, //,Initial Upload
	{0x3380,0x10}, //,Initial Upload
	{0x3381,0xC4}, //,Initial Upload
	{0x3382,0x14}, //,Initial Upload
	{0x3383,0x15}, //,Initial Upload
	{0x3384,0xA9}, //,Initial Upload
	{0x3385,0x0}, //,Initial Upload
	{0x3386,0x10}, //,Initial Upload
	{0x3387,0x84}, //,Initial Upload
	{0x3388,0x16}, //,Initial Upload
	{0x3389,0x15}, //,Initial Upload
	{0x338A,0x41}, //,Initial Upload
	{0x338B,0x0}, //,Initial Upload
	{0x338C,0x10}, //,Initial Upload
	{0x338D,0x80}, //,Initial Upload
	{0x338E,0x16}, //,Initial Upload
	{0x338F,0x15}, //,Initial Upload
	{0x3390,0x2}, //,Initial Upload
	{0x3391,0x0}, //,Initial Upload
	{0x3392,0x10}, //,Initial Upload
	{0x3393,0xA0}, //,Initial Upload
	{0x3394,0x16}, //,Initial Upload
	{0x3395,0x15}, //,Initial Upload
	{0x305C,0x18}, //,Initial Upload
	{0x305D,0x0}, //,Initial Upload
	{0x305E,0x19}, //,Initial Upload
	{0x305F,0x0}, //,Initial Upload
	{0x3396,0x1}, //,Initial Upload
	{0x3397,0x0}, //,Initial Upload
	{0x3398,0x90}, //,Initial Upload
	{0x3399,0x30}, //,Initial Upload
	{0x339A,0x56}, //,Initial Upload
	{0x339B,0x57}, //,Initial Upload
	{0x339C,0x1}, //,Initial Upload
	{0x339D,0x0}, //,Initial Upload
	{0x339E,0x10}, //,Initial Upload
	{0x339F,0x20}, //,Initial Upload
	{0x33A0,0xD6}, //,Initial Upload
	{0x33A1,0x17}, //,Initial Upload
	{0x33A2,0x1}, //,Initial Upload
	{0x33A3,0x0}, //,Initial Upload
	{0x33A4,0x10}, //,Initial Upload
	{0x33A5,0x28}, //,Initial Upload
	{0x33A6,0xD6}, //,Initial Upload
	{0x33A7,0x17}, //,Initial Upload
	{0x33A8,0x3}, //,Initial Upload
	{0x33A9,0x0}, //,Initial Upload
	{0x33AA,0x10}, //,Initial Upload
	{0x33AB,0x20}, //,Initial Upload
	{0x33AC,0xD6}, //,Initial Upload
	{0x33AD,0x17}, //,Initial Upload
	{0x33AE,0x61}, //,Initial Upload
	{0x33AF,0x0}, //,Initial Upload
	{0x33B0,0x10}, //,Initial Upload
	{0x33B1,0x20}, //,Initial Upload
	{0x33B2,0xD6}, //,Initial Upload
	{0x33B3,0x15}, //,Initial Upload
	{0x33B4,0x1}, //,Initial Upload
	{0x33B5,0x0}, //,Initial Upload
	{0x33B6,0x10}, //,Initial Upload
	{0x33B7,0x20}, //,Initial Upload
	{0x33B8,0xD6}, //,Initial Upload
	{0x33B9,0x1D}, //,Initial Upload
	{0x33BA,0x1}, //,Initial Upload
	{0x33BB,0x0}, //,Initial Upload
	{0x33BC,0x50}, //,Initial Upload
	{0x33BD,0x20}, //,Initial Upload
	{0x33BE,0xD6}, //,Initial Upload
	{0x33BF,0x1D}, //,Initial Upload
	{0x33C0,0x2C}, //,Initial Upload
	{0x33C1,0x0}, //,Initial Upload
	{0x33C2,0x10}, //,Initial Upload
	{0x33C3,0x20}, //,Initial Upload
	{0x33C4,0xD6}, //,Initial Upload
	{0x33C5,0x1D}, //,Initial Upload
	{0x33C6,0x1}, //,Initial Upload
	{0x33C7,0x0}, //,Initial Upload
	{0x33C8,0x90}, //,Initial Upload
	{0x33C9,0x20}, //,Initial Upload
	{0x33CA,0xD6}, //,Initial Upload
	{0x33CB,0x1D}, //,Initial Upload
	{0x33CC,0x83}, //,Initial Upload
	{0x33CD,0x0}, //,Initial Upload
	{0x33CE,0x10}, //,Initial Upload
	{0x33CF,0x20}, //,Initial Upload
	{0x33D0,0xD6}, //,Initial Upload
	{0x33D1,0x15}, //,Initial Upload
	{0x33D2,0x1}, //,Initial Upload
	{0x33D3,0x0}, //,Initial Upload
	{0x33D4,0x10}, //,Initial Upload
	{0x33D5,0x30}, //,Initial Upload
	{0x33D6,0xD6}, //,Initial Upload
	{0x33D7,0x15}, //,Initial Upload
	{0x33D8,0x1}, //,Initial Upload
	{0x33D9,0x0}, //,Initial Upload
	{0x33DA,0x10}, //,Initial Upload
	{0x33DB,0x20}, //,Initial Upload
	{0x33DC,0xD6}, //,Initial Upload
	{0x33DD,0x15}, //,Initial Upload
	{0x33DE,0x1}, //,Initial Upload
	{0x33DF,0x0}, //,Initial Upload
	{0x33E0,0x10}, //,Initial Upload
	{0x33E1,0x20}, //,Initial Upload
	{0x33E2,0x56}, //,Initial Upload
	{0x33E3,0x15}, //,Initial Upload
	{0x33E4,0x7}, //,Initial Upload
	{0x33E5,0x0}, //,Initial Upload
	{0x33E6,0x10}, //,Initial Upload
	{0x33E7,0x20}, //,Initial Upload
	{0x33E8,0x16}, //,Initial Upload
	{0x33E9,0x15}, //,Initial Upload
	{0x3060,0x26}, //,Initial Upload
	{0x3061,0x0}, //,Initial Upload
	{0x302A,0xFF}, //,Initial Upload
	{0x302B,0xFF}, //,Initial Upload
	{0x302C,0xFF}, //,Initial Upload
	{0x302D,0xFF}, //,Initial Upload
	{0x302E,0x3F}, //,Initial Upload
	{0x3013,0xB}, //,Initial Upload
	{0x102B,0x2C}, //,Initial Upload
	{0x102C,0x1}, //,Initial Upload
	{0x1035,0x54}, //,Initial Upload
	{0x1036,0x0}, //,Initial Upload
	{0x3090,0x2A}, //,Initial Upload
	{0x3091,0x1}, //,Initial Upload
	{0x30C6,0x5}, //,Initial Upload
	{0x30C7,0x0}, //,Initial Upload
	{0x30C8,0x0}, //,Initial Upload
	{0x30C9,0x0}, //,Initial Upload
	{0x30CA,0x0}, //,Initial Upload
	{0x30CB,0x0}, //,Initial Upload
	{0x30CC,0x0}, //,Initial Upload
	{0x30CD,0x0}, //,Initial Upload
	{0x30CE,0x0}, //,Initial Upload
	{0x30CF,0x5}, //,Initial Upload
	{0x30D0,0x0}, //,Initial Upload
	{0x30D1,0x0}, //,Initial Upload
	{0x30D2,0x0}, //,Initial Upload
	{0x30D3,0x0}, //,Initial Upload
	{0x30D4,0x0}, //,Initial Upload
	{0x30D5,0x0}, //,Initial Upload
	{0x30D6,0x0}, //,Initial Upload
	{0x30D7,0x0}, //,Initial Upload
	{0x30F3,0x5}, //,Initial Upload
	{0x30F4,0x0}, //,Initial Upload
	{0x30F5,0x0}, //,Initial Upload
	{0x30F6,0x0}, //,Initial Upload
	{0x30F7,0x0}, //,Initial Upload
	{0x30F8,0x0}, //,Initial Upload
	{0x30F9,0x0}, //,Initial Upload
	{0x30FA,0x0}, //,Initial Upload
	{0x30FB,0x0}, //,Initial Upload
	{0x30D8,0x5}, //,Initial Upload
	{0x30D9,0x0}, //,Initial Upload
	{0x30DA,0x0}, //,Initial Upload
	{0x30DB,0x0}, //,Initial Upload
	{0x30DC,0x0}, //,Initial Upload
	{0x30DD,0x0}, //,Initial Upload
	{0x30DE,0x0}, //,Initial Upload
	{0x30DF,0x0}, //,Initial Upload
	{0x30E0,0x0}, //,Initial Upload
	{0x30E1,0x5}, //,Initial Upload
	{0x30E2,0x0}, //,Initial Upload
	{0x30E3,0x0}, //,Initial Upload
	{0x30E4,0x0}, //,Initial Upload
	{0x30E5,0x0}, //,Initial Upload
	{0x30E6,0x0}, //,Initial Upload
	{0x30E7,0x0}, //,Initial Upload
	{0x30E8,0x0}, //,Initial Upload
	{0x30E9,0x0}, //,Initial Upload
	{0x30F3,0x5}, //,Initial Upload
	{0x30F4,0x2}, //,Initial Upload
	{0x30F5,0x0}, //,Initial Upload
	{0x30F6,0x17}, //,Initial Upload
	{0x30F7,0x1}, //,Initial Upload
	{0x30F8,0x0}, //,Initial Upload
	{0x30F9,0x0}, //,Initial Upload
	{0x30FA,0x0}, //,Initial Upload
	{0x30FB,0x0}, //,Initial Upload
	{0x30D8,0x3}, //,Initial Upload
	{0x30D9,0x1}, //,Initial Upload
	{0x30DA,0x0}, //,Initial Upload
	{0x30DB,0x19}, //,Initial Upload
	{0x30DC,0x1}, //,Initial Upload
	{0x30DD,0x0}, //,Initial Upload
	{0x30DE,0x0}, //,Initial Upload
	{0x30DF,0x0}, //,Initial Upload
	{0x30E0,0x0}, //,Initial Upload
	{0x30A2,0x5}, //,Initial Upload
	{0x30A3,0x2}, //,Initial Upload
	{0x30A4,0x0}, //,Initial Upload
	{0x30A5,0x22}, //,Initial Upload
	{0x30A6,0x0}, //,Initial Upload
	{0x30A7,0x0}, //,Initial Upload
	{0x30A8,0x0}, //,Initial Upload
	{0x30A9,0x0}, //,Initial Upload
	{0x30AA,0x0}, //,Initial Upload
	{0x30AB,0x5}, //,Initial Upload
	{0x30AC,0x2}, //,Initial Upload
	{0x30AD,0x0}, //,Initial Upload
	{0x30AE,0x22}, //,Initial Upload
	{0x30AF,0x0}, //,Initial Upload
	{0x30B0,0x0}, //,Initial Upload
	{0x30B1,0x0}, //,Initial Upload
	{0x30B2,0x0}, //,Initial Upload
	{0x30B3,0x0}, //,Initial Upload
	{0x30BD,0x5}, //,Initial Upload
	{0x30BE,0x9F}, //,Initial Upload
	{0x30BF,0x0}, //,Initial Upload
	{0x30C0,0x7D}, //,Initial Upload
	{0x30C1,0x0}, //,Initial Upload
	{0x30C2,0x0}, //,Initial Upload
	{0x30C3,0x0}, //,Initial Upload
	{0x30C4,0x0}, //,Initial Upload
	{0x30C5,0x0}, //,Initial Upload
	{0x30B4,0x4}, //,Initial Upload
	{0x30B5,0x9C}, //,Initial Upload
	{0x30B6,0x0}, //,Initial Upload
	{0x30B7,0x7D}, //,Initial Upload
	{0x30B8,0x0}, //,Initial Upload
	{0x30B9,0x0}, //,Initial Upload
	{0x30BA,0x0}, //,Initial Upload
	{0x30BB,0x0}, //,Initial Upload
	{0x30BC,0x0}, //,Initial Upload
	{0x30FC,0x5}, //,Initial Upload
	{0x30FD,0x0}, //,Initial Upload
	{0x30FE,0x0}, //,Initial Upload
	{0x30FF,0x0}, //,Initial Upload
	{0x3100,0x0}, //,Initial Upload
	{0x3101,0x0}, //,Initial Upload
	{0x3102,0x0}, //,Initial Upload
	{0x3103,0x0}, //,Initial Upload
	{0x3104,0x0}, //,Initial Upload
	{0x3105,0x5}, //,Initial Upload
	{0x3106,0x0}, //,Initial Upload
	{0x3107,0x0}, //,Initial Upload
	{0x3108,0x0}, //,Initial Upload
	{0x3109,0x0}, //,Initial Upload
	{0x310A,0x0}, //,Initial Upload
	{0x310B,0x0}, //,Initial Upload
	{0x310C,0x0}, //,Initial Upload
	{0x310D,0x0}, //,Initial Upload
	{0x3099,0x5}, //,Initial Upload
	{0x309A,0x96}, //,Initial Upload
	{0x309B,0x0}, //,Initial Upload
	{0x309C,0x6}, //,Initial Upload
	{0x309D,0x0}, //,Initial Upload
	{0x309E,0x0}, //,Initial Upload
	{0x309F,0x0}, //,Initial Upload
	{0x30A0,0x0}, //,Initial Upload
	{0x30A1,0x0}, //,Initial Upload
	{0x310E,0x5}, //,Initial Upload
	{0x310F,0x2}, //,Initial Upload
	{0x3110,0x0}, //,Initial Upload
	{0x3111,0x2B}, //,Initial Upload
	{0x3112,0x0}, //,Initial Upload
	{0x3113,0x0}, //,Initial Upload
	{0x3114,0x0}, //,Initial Upload
	{0x3115,0x0}, //,Initial Upload
	{0x3116,0x0}, //,Initial Upload
	{0x3117,0x5}, //,Initial Upload
	{0x3118,0x2}, //,Initial Upload
	{0x3119,0x0}, //,Initial Upload
	{0x311A,0x2C}, //,Initial Upload
	{0x311B,0x0}, //,Initial Upload
	{0x311C,0x0}, //,Initial Upload
	{0x311D,0x0}, //,Initial Upload
	{0x311E,0x0}, //,Initial Upload
	{0x311F,0x0}, //,Initial Upload
	{0x30EA,0x0}, //,Initial Upload
	{0x30EB,0x0}, //,Initial Upload
	{0x30EC,0x0}, //,Initial Upload
	{0x30ED,0x0}, //,Initial Upload
	{0x30EE,0x0}, //,Initial Upload
	{0x30EF,0x0}, //,Initial Upload
	{0x30F0,0x0}, //,Initial Upload
	{0x30F1,0x0}, //,Initial Upload
	{0x30F2,0x0}, //,Initial Upload
	{0x313B,0x3}, //,Initial Upload
	{0x313C,0x31}, //,Initial Upload
	{0x313D,0x0}, //,Initial Upload
	{0x313E,0x7}, //,Initial Upload
	{0x313F,0x0}, //,Initial Upload
	{0x3140,0x68}, //,Initial Upload
	{0x3141,0x0}, //,Initial Upload
	{0x3142,0x34}, //,Initial Upload
	{0x3143,0x0}, //,Initial Upload
	{0x31A0,0x3}, //,Initial Upload
	{0x31A1,0x16}, //,Initial Upload
	{0x31A2,0x0}, //,Initial Upload
	{0x31A3,0x8}, //,Initial Upload
	{0x31A4,0x0}, //,Initial Upload
	{0x31A5,0x7E}, //,Initial Upload
	{0x31A6,0x0}, //,Initial Upload
	{0x31A7,0x8}, //,Initial Upload
	{0x31A8,0x0}, //,Initial Upload
	{0x31A9,0x3}, //,Initial Upload
	{0x31AA,0x16}, //,Initial Upload
	{0x31AB,0x0}, //,Initial Upload
	{0x31AC,0x8}, //,Initial Upload
	{0x31AD,0x0}, //,Initial Upload
	{0x31AE,0x7E}, //,Initial Upload
	{0x31AF,0x0}, //,Initial Upload
	{0x31B0,0x8}, //,Initial Upload
	{0x31B1,0x0}, //,Initial Upload
	{0x31B2,0x3}, //,Initial Upload
	{0x31B3,0x16}, //,Initial Upload
	{0x31B4,0x0}, //,Initial Upload
	{0x31B5,0x8}, //,Initial Upload
	{0x31B6,0x0}, //,Initial Upload
	{0x31B7,0x7E}, //,Initial Upload
	{0x31B8,0x0}, //,Initial Upload
	{0x31B9,0x8}, //,Initial Upload
	{0x31BA,0x0}, //,Initial Upload
	{0x3120,0x5}, //,Initial Upload
	{0x3121,0x45}, //,Initial Upload
	{0x3122,0x0}, //,Initial Upload
	{0x3123,0x1D}, //,Initial Upload
	{0x3124,0x0}, //,Initial Upload
	{0x3125,0xA9}, //,Initial Upload
	{0x3126,0x0}, //,Initial Upload
	{0x3127,0x6D}, //,Initial Upload
	{0x3128,0x0}, //,Initial Upload
	{0x3129,0x5}, //,Initial Upload
	{0x312A,0x15}, //,Initial Upload
	{0x312B,0x0}, //,Initial Upload
	{0x312C,0xA}, //,Initial Upload
	{0x312D,0x0}, //,Initial Upload
	{0x312E,0x45}, //,Initial Upload
	{0x312F,0x0}, //,Initial Upload
	{0x3130,0x1D}, //,Initial Upload
	{0x3131,0x0}, //,Initial Upload
	{0x3132,0x5}, //,Initial Upload
	{0x3133,0x7D}, //,Initial Upload
	{0x3134,0x0}, //,Initial Upload
	{0x3135,0xA}, //,Initial Upload
	{0x3136,0x0}, //,Initial Upload
	{0x3137,0xA9}, //,Initial Upload
	{0x3138,0x0}, //,Initial Upload
	{0x3139,0x6D}, //,Initial Upload
	{0x313A,0x0}, //,Initial Upload
	{0x3144,0x5}, //,Initial Upload
	{0x3145,0x0}, //,Initial Upload
	{0x3146,0x0}, //,Initial Upload
	{0x3147,0x30}, //,Initial Upload
	{0x3148,0x0}, //,Initial Upload
	{0x3149,0x0}, //,Initial Upload
	{0x314A,0x0}, //,Initial Upload
	{0x314B,0x0}, //,Initial Upload
	{0x314C,0x0}, //,Initial Upload
	{0x314D,0x3}, //,Initial Upload
	{0x314E,0x0}, //,Initial Upload
	{0x314F,0x0}, //,Initial Upload
	{0x3150,0x31}, //,Initial Upload
	{0x3151,0x0}, //,Initial Upload
	{0x3152,0x0}, //,Initial Upload
	{0x3153,0x0}, //,Initial Upload
	{0x3154,0x0}, //,Initial Upload
	{0x3155,0x0}, //,Initial Upload
	{0x31D8,0x5}, //,Initial Upload
	{0x31D9,0x3A}, //,Initial Upload
	{0x31DA,0x0}, //,Initial Upload
	{0x31DB,0x2E}, //,Initial Upload
	{0x31DC,0x0}, //,Initial Upload
	{0x31DD,0x9E}, //,Initial Upload
	{0x31DE,0x0}, //,Initial Upload
	{0x31DF,0x7E}, //,Initial Upload
	{0x31E0,0x0}, //,Initial Upload
	{0x31E1,0x5}, //,Initial Upload
	{0x31E2,0x4}, //,Initial Upload
	{0x31E3,0x0}, //,Initial Upload
	{0x31E4,0x4}, //,Initial Upload
	{0x31E5,0x0}, //,Initial Upload
	{0x31E6,0x73}, //,Initial Upload
	{0x31E7,0x0}, //,Initial Upload
	{0x31E8,0x4}, //,Initial Upload
	{0x31E9,0x0}, //,Initial Upload
	{0x31EA,0x5}, //,Initial Upload
	{0x31EB,0x0}, //,Initial Upload
	{0x31EC,0x0}, //,Initial Upload
	{0x31ED,0x0}, //,Initial Upload
	{0x31EE,0x0}, //,Initial Upload
	{0x31EF,0x0}, //,Initial Upload
	{0x31F0,0x0}, //,Initial Upload
	{0x31F1,0x0}, //,Initial Upload
	{0x31F2,0x0}, //,Initial Upload
	{0x31F3,0x0}, //,Initial Upload
	{0x31F4,0x0}, //,Initial Upload
	{0x31F5,0x0}, //,Initial Upload
	{0x31F6,0x0}, //,Initial Upload
	{0x31F7,0x0}, //,Initial Upload
	{0x31F8,0x0}, //,Initial Upload
	{0x31F9,0x0}, //,Initial Upload
	{0x31FA,0x0}, //,Initial Upload
	{0x31FB,0x5}, //,Initial Upload
	{0x31FC,0x0}, //,Initial Upload
	{0x31FD,0x0}, //,Initial Upload
	{0x31FE,0x0}, //,Initial Upload
	{0x31FF,0x0}, //,Initial Upload
	{0x3200,0x0}, //,Initial Upload
	{0x3201,0x0}, //,Initial Upload
	{0x3202,0x0}, //,Initial Upload
	{0x3203,0x0}, //,Initial Upload
	{0x3204,0x0}, //,Initial Upload
	{0x3205,0x0}, //,Initial Upload
	{0x3206,0x0}, //,Initial Upload
	{0x3207,0x0}, //,Initial Upload
	{0x3208,0x0}, //,Initial Upload
	{0x3209,0x0}, //,Initial Upload
	{0x320A,0x0}, //,Initial Upload
	{0x320B,0x0}, //,Initial Upload
	{0x3164,0x5}, //,Initial Upload
	{0x3165,0x14}, //,Initial Upload
	{0x3166,0x0}, //,Initial Upload
	{0x3167,0xC}, //,Initial Upload
	{0x3168,0x0}, //,Initial Upload
	{0x3169,0x44}, //,Initial Upload
	{0x316A,0x0}, //,Initial Upload
	{0x316B,0x1F}, //,Initial Upload
	{0x316C,0x0}, //,Initial Upload
	{0x316D,0x5}, //,Initial Upload
	{0x316E,0x7C}, //,Initial Upload
	{0x316F,0x0}, //,Initial Upload
	{0x3170,0xC}, //,Initial Upload
	{0x3171,0x0}, //,Initial Upload
	{0x3172,0xA8}, //,Initial Upload
	{0x3173,0x0}, //,Initial Upload
	{0x3174,0x6F}, //,Initial Upload
	{0x3175,0x0}, //,Initial Upload
	{0x31C4,0x5}, //,Initial Upload
	{0x31C5,0x24}, //,Initial Upload
	{0x31C6,0x1}, //,Initial Upload
	{0x31C7,0x4}, //,Initial Upload
	{0x31C8,0x0}, //,Initial Upload
	{0x31C9,0x5}, //,Initial Upload
	{0x31CA,0x24}, //,Initial Upload
	{0x31CB,0x1}, //,Initial Upload
	{0x31CC,0x4}, //,Initial Upload
	{0x31CD,0x0}, //,Initial Upload
	{0x31CE,0x5}, //,Initial Upload
	{0x31CF,0x24}, //,Initial Upload
	{0x31D0,0x1}, //,Initial Upload
	{0x31D1,0x4}, //,Initial Upload
	{0x31D2,0x0}, //,Initial Upload
	{0x31D3,0x5}, //,Initial Upload
	{0x31D4,0x73}, //,Initial Upload
	{0x31D5,0x0}, //,Initial Upload
	{0x31D6,0xB1}, //,Initial Upload
	{0x31D7,0x0}, //,Initial Upload
	{0x3176,0x5}, //,Initial Upload
	{0x3177,0x10}, //,Initial Upload
	{0x3178,0x0}, //,Initial Upload
	{0x3179,0x56}, //,Initial Upload
	{0x317A,0x0}, //,Initial Upload
	{0x317B,0x0}, //,Initial Upload
	{0x317C,0x0}, //,Initial Upload
	{0x317D,0x0}, //,Initial Upload
	{0x317E,0x0}, //,Initial Upload
	{0x317F,0x5}, //,Initial Upload
	{0x3180,0x6A}, //,Initial Upload
	{0x3181,0x0}, //,Initial Upload
	{0x3182,0xAD}, //,Initial Upload
	{0x3183,0x0}, //,Initial Upload
	{0x3184,0x0}, //,Initial Upload
	{0x3185,0x0}, //,Initial Upload
	{0x3186,0x0}, //,Initial Upload
	{0x3187,0x0}, //,Initial Upload
	{0x100C,0x7E}, //,Initial Upload
	{0x100D,0x0}, //,Initial Upload
	{0x1012,0xDF}, //,Initial Upload
	{0x1013,0x2B}, //,Initial Upload
	{0x1002,0x4}, //,Initial Upload
	// {0x1003,0x10}, //,Sensor Control Mode.IMAGER_STATE(0)
	{0x0043,0x0}, //,Sensor Control Mode.SLEEP_POWER_MODE(0)
	{0x0043,0x0}, //,Sensor Control Mode.IDLE_POWER_MODE(0)
	{0x0043,0x4}, //,Sensor Control Mode.SYSTEM_CLOCK_ENABLE(0)
	{0x0043,0xC}, //,Sensor Control Mode.SRAM_CLOCK_ENABLE(0)
	{0x1002,0x4}, //,Sensor Control Mode.IMAGER_RUN_CONT(0)
	{0x1001,0x41}, //,Sensor Control Mode.EXT_EVENT_SEL(0)
	{0x10F2,0x1}, //,Sensor Control Mode.NB_OF_FRAMES_A(0)
	{0x10F3,0x0}, //,Sensor Control Mode.NB_OF_FRAMES_A(1)
	{0x1111,0x1}, //,Sensor Control Mode.NB_OF_FRAMES_B(0)
	{0x1112,0x0}, //,Sensor Control Mode.NB_OF_FRAMES_B(1)
	{0x0012,0x0}, //,IO Drive Strength.DIG_DRIVE_STRENGTH(0)
	{0x0012,0x0}, //,IO Drive Strength.CCI_DRIVE_STRENGTH(0)
	{0x1001,0x41}, //,Readout && Exposure.EXT_EXP_PW_SEL(0)
	{0x10D0,0x0}, //,Readout && Exposure.EXT_EXP_PW_DELAY(0)
	{0x10D1,0x0}, //,Readout && Exposure.EXT_EXP_PW_DELAY(1)
	{0x1012,0x14}, //,Readout && Exposure.VBLANK_A(0)
	{0x1013,0x0}, //,Readout && Exposure.VBLANK_A(1)
	{0x1103,0x91}, //,Readout && Exposure.VBLANK_B(0)
	{0x1104,0xD}, //,Readout && Exposure.VBLANK_B(1)
	{0x100C,0x80}, //,Readout && Exposure.EXP_TIME_A(0)
	{0x100D,0x0}, //,Readout && Exposure.EXP_TIME_A(1)
	{0x1115,0x80}, //,Readout && Exposure.EXP_TIME_B(0)
	{0x1116,0x0}, //,Readout && Exposure.EXP_TIME_B(1)
	{0x102B,0x30}, //,Readout && Exposure.ROW_LENGTH_A(0)
	{0x102C,0x1}, //,Readout && Exposure.ROW_LENGTH_A(1)
	{0x1113,0x30}, //,Readout && Exposure.ROW_LENGTH_B(0)
	{0x1114,0x1}, //,Readout && Exposure.ROW_LENGTH_B(1)
	{0x2008,0x40}, //,Horizontal ROI.HSIZE_A(0)
	{0x2009,0x1}, //,Horizontal ROI.HSIZE_A(1)
	{0x2098,0x20}, //,Horizontal ROI.HSIZE_B(0)
	{0x2099,0x3}, //,Horizontal ROI.HSIZE_B(1)
	{0x200A,0xCC}, //,Horizontal ROI.HSTART_A(0)
	{0x200B,0x1}, //,Horizontal ROI.HSTART_A(1)
	{0x209A,0x0}, //,Horizontal ROI.HSTART_B(0)
	{0x209B,0x0}, //,Horizontal ROI.HSTART_B(1)
	{0x207D,0x80}, //,Horizontal ROI.MIPI_HSIZE(0)
	{0x207E,0x2}, //,Horizontal ROI.MIPI_HSIZE(1)
	{0x107D,0xF0}, //,Vertical ROI.VSTART0_A(0)
	{0x107E,0x0}, //,Vertical ROI.VSTART0_A(1)
	{0x1087,0xE0}, //,Vertical ROI.VSIZE0_A(0)
	{0x1088,0x1}, //,Vertical ROI.VSIZE0_A(1)
	{0x1105,0x0}, //,Vertical ROI.VSTART0_B(0)
	{0x1106,0x0}, //,Vertical ROI.VSTART0_B(1)
	{0x110A,0x78}, //,Vertical ROI.VSIZE0_B(0)
	{0x110B,0x5}, //,Vertical ROI.VSIZE0_B(1)
	{0x107D,0xF0}, //,Vertical ROI.VSTART1_A(0)
	{0x107E,0x0}, //,Vertical ROI.VSTART1_A(1)
	{0x107F,0x0}, //,Vertical ROI.VSTART1_A(2)
	{0x1087,0xE0}, //,Vertical ROI.VSIZE1_A(0)
	{0x1088,0x1}, //,Vertical ROI.VSIZE1_A(1)
	{0x1089,0x0}, //,Vertical ROI.VSIZE1_A(2)
	{0x1105,0x0}, //,Vertical ROI.VSTART1_B(0)
	{0x1106,0x0}, //,Vertical ROI.VSTART1_B(1)
	{0x1107,0x0}, //,Vertical ROI.VSTART1_B(2)
	{0x110A,0x78}, //,Vertical ROI.VSIZE1_B(0)
	{0x110B,0x5}, //,Vertical ROI.VSIZE1_B(1)
	{0x110C,0x0}, //,Vertical ROI.VSIZE1_B(2)
	{0x107D,0xF0}, //,Vertical ROI.VSTART2_A(0)
	{0x107E,0x0}, //,Vertical ROI.VSTART2_A(1)
	{0x107F,0x0}, //,Vertical ROI.VSTART2_A(2)
	{0x1080,0x0}, //,Vertical ROI.VSTART2_A(3)
	{0x1081,0x0}, //,Vertical ROI.VSTART2_A(4)
	{0x1087,0xE0}, //,Vertical ROI.VSIZE2_A(0)
	{0x1088,0x1}, //,Vertical ROI.VSIZE2_A(1)
	{0x1089,0x0}, //,Vertical ROI.VSIZE2_A(2)
	{0x108A,0x0}, //,Vertical ROI.VSIZE2_A(3)
	{0x108B,0x0}, //,Vertical ROI.VSIZE2_A(4)
	{0x1105,0x0}, //,Vertical ROI.VSTART2_B(0)
	{0x1106,0x0}, //,Vertical ROI.VSTART2_B(1)
	{0x1107,0x0}, //,Vertical ROI.VSTART2_B(2)
	{0x1108,0x0}, //,Vertical ROI.VSTART2_B(3)
	{0x1109,0x0}, //,Vertical ROI.VSTART2_B(4)
	{0x110A,0x78}, //,Vertical ROI.VSIZE2_B(0)
	{0x110B,0x5}, //,Vertical ROI.VSIZE2_B(1)
	{0x110C,0x0}, //,Vertical ROI.VSIZE2_B(2)
	{0x110D,0x0}, //,Vertical ROI.VSIZE2_B(3)
	{0x110E,0x0}, //,Vertical ROI.VSIZE2_B(4)
	{0x209C,0x0}, //,Mirroring && Flipping.HFLIP_A(0)
	{0x209D,0x0}, //,Mirroring && Flipping.HFLIP_B(0)
	{0x1095,0x0}, //,Mirroring && Flipping.VFLIP(0)
	{0x2063,0x0}, //,Mirroring && Flipping.BIT_ORDER(0)
	{0x6006,0x0}, //,MIPI.TX_CTRL_EN(0)
	{0x5004,0x1}, //,MIPI.datarate
	{0x5086,0x2}, //,MIPI.datarate
	{0x5087,0x4E}, //,MIPI.datarate
	{0x5088,0x0}, //,MIPI.datarate
	{0x5090,0x0}, //,MIPI.datarate
	{0x5091,0x8}, //,MIPI.datarate
	{0x5092,0x14}, //,MIPI.datarate
	{0x5093,0xF}, //,MIPI.datarate
	{0x5094,0x6}, //,MIPI.datarate
	{0x5095,0x32}, //,MIPI.datarate
	{0x5096,0xE}, //,MIPI.datarate
	{0x5097,0x0}, //,MIPI.datarate
	{0x5098,0x11}, //,MIPI.datarate
	{0x5004,0x0}, //,MIPI.datarate
	{0x2066,0x6C}, //,MIPI.datarate
	{0x2067,0x7}, //,MIPI.datarate
	{0x206E,0x7E}, //,MIPI.datarate
	{0x206F,0x6}, //,MIPI.datarate
	{0x20AC,0x7E}, //,MIPI.datarate
	{0x20AD,0x6}, //,MIPI.datarate
	{0x2076,0xC8}, //,MIPI.datarate
	{0x2077,0x0}, //,MIPI.datarate
	{0x20B4,0xC8}, //,MIPI.datarate
	{0x20B5,0x0}, //,MIPI.datarate
	{0x2078,0x1E}, //,MIPI.datarate
	{0x2079,0x4}, //,MIPI.datarate
	{0x20B6,0x1E}, //,MIPI.datarate
	{0x20B7,0x4}, //,MIPI.datarate
	{0x207A,0xD4}, //,MIPI.datarate
	{0x207B,0x4}, //,MIPI.datarate
	{0x20B8,0xD4}, //,MIPI.datarate
	{0x20B9,0x4}, //,MIPI.datarate
	{0x208D,0x4}, //,MIPI.CSI2_DTYPE(0)
	{0x208E,0x0}, //,MIPI.CSI2_DTYPE(1)
	{0x207C,0x0}, //,MIPI.VC_ID(0)
	{0x6001,0x7}, //,MIPI.TINIT(0)
	{0x6002,0xD8}, //,MIPI.TINIT(1)
	{0x6010,0x0}, //,MIPI.FRAME_MODE(0)
	{0x6010,0x0}, //,MIPI.EMBEDDED_FRAME_MODE(0)
	{0x6011,0x0}, //,MIPI.DATA_ENABLE_POLARITY(0)
	{0x6011,0x0}, //,MIPI.HSYNC_POLARITY(0)
	{0x6011,0x0}, //,MIPI.VSYNC_POLARITY(0)
	{0x6012,0x1}, //,MIPI.LANE(0)
	{0x6013,0x0}, //,MIPI.CLK_MODE(0)
	{0x6016,0x0}, //,MIPI.FRAME_COUNTER(0)
	{0x6017,0x0}, //,MIPI.FRAME_COUNTER(1)
	{0x6037,0x1}, //,MIPI.LINE_COUNT_RAW8(0)
	{0x6037,0x3}, //,MIPI.LINE_COUNT_RAW10(0)
	{0x6037,0x7}, //,MIPI.LINE_COUNT_RAW12(0)
	{0x6039,0x1}, //,MIPI.LINE_COUNT_EMB(0)
	{0x6018,0x0}, //,MIPI.CCI_READ_INTERRUPT_EN(0)
	{0x6018,0x0}, //,MIPI.CCI_WRITE_INTERRUPT_EN(0)
	{0x6065,0x0}, //,MIPI.TWAKE_TIMER(0)
	{0x6066,0x0}, //,MIPI.TWAKE_TIMER(1)
	{0x601C,0x0}, //,MIPI.SKEW_CAL_EN(0)
	{0x601D,0x0}, //,MIPI.SKEW_COUNT(0)
	{0x601E,0x22}, //,MIPI.SKEW_COUNT(1)
	{0x601F,0x0}, //,MIPI.SCRAMBLING_EN(0)
	{0x6003,0x1}, //,MIPI.INIT_SKEW_EN(0)
	{0x6004,0x7A}, //,MIPI.INIT_SKEW(0)
	{0x6005,0x12}, //,MIPI.INIT_SKEW(1)
	{0x6006,0x1}, //,MIPI.TX_CTRL_EN(0)
	{0x4006,0x8}, //,Processing.BSP(0)
	{0x209E,0x2}, //,Processing.BIT_DEPTH(0)
	{0x2045,0x1}, //,Processing.CDS_RNC(0)
	{0x2048,0x1}, //,Processing.CDS_IMG(0)
	{0x204B,0x3}, //,Processing.RNC_EN(0)
	{0x205B,0x64}, //,Processing.RNC_DARK_TARGET(0)
	{0x205C,0x0}, //,Processing.RNC_DARK_TARGET(1)
	{0x24DC,0x12}, //,Defect Pixel Correction.DC_ENABLE(0)
	{0x24DC,0x10}, //,Defect Pixel Correction.DC_MODE(0)
	{0x24DC,0x0}, //,Defect Pixel Correction.DC_REPLACEMENT_VALUE(0)
	{0x24DD,0x0}, //,Defect Pixel Correction.DC_LIMIT_LOW(0)
	{0x24DE,0x0}, //,Defect Pixel Correction.DC_LIMIT_HIGH(0)
	{0x24DF,0x0}, //,Defect Pixel Correction.DC_LIMIT_HIGH_MODE(0)
	{0x10D7,0x1}, //,Illumination Trigger.ILLUM_EN(0)
	{0x10D8,0x2}, //,Illumination Trigger.ILLUM_POL(0)
	{0x205D,0x0}, //,Histogram.HIST_EN(0)
	{0x205E,0x0}, //,Histogram.HIST_USAGE_RATIO(0)
	{0x2063,0x0}, //,Histogram.PIXEL_DATA_SUPP(0)
	{0x2063,0x0}, //,Histogram.PIXEL_TRANSMISSION(0)
	{0x2091,0x0}, //,Test Pattern Generator.TPG_EN(0)
	{0x2091,0x0}, //,Test Pattern Generator.TPG_CONFIG(0)

};

static const struct mira220_reg full_400_400_250fps_12b_2lanes_reg[] = {
	{0x1003,0x2}, //,Initial Upload
	{0x6006,0x0}, //,Initial Upload
	{0x6012,0x1}, //,Initial Upload
	{0x6013,0x0}, //,Initial Upload
	{0x6006,0x1}, //,Initial Upload
	{0x205D,0x0}, //,Initial Upload
	{0x2063,0x0}, //,Initial Upload
	{0x24DC,0x13}, //,Initial Upload
	{0x24DD,0x3}, //,Initial Upload
	{0x24DE,0x3}, //,Initial Upload
	{0x24DF,0x0}, //,Initial Upload
	{0x4006,0x8}, //,Initial Upload
	{0x401C,0x6F}, //,Initial Upload
	{0x204B,0x3}, //,Initial Upload
	{0x205B,0x64}, //,Initial Upload
	{0x205C,0x0}, //,Initial Upload
	{0x4018,0x3F}, //,Initial Upload
	{0x403B,0xB}, //,Initial Upload
	{0x403E,0xE}, //,Initial Upload
	{0x402B,0x6}, //,Initial Upload
	{0x401E,0x2}, //,Initial Upload
	{0x4038,0x3B}, //,Initial Upload
	{0x1077,0x0}, //,Initial Upload
	{0x1078,0x0}, //,Initial Upload
	{0x1009,0x8}, //,Initial Upload
	{0x100A,0x0}, //,Initial Upload
	{0x110F,0x8}, //,Initial Upload
	{0x1110,0x0}, //,Initial Upload
	{0x1006,0x2}, //,Initial Upload
	{0x402C,0x64}, //,Initial Upload
	{0x3064,0x0}, //,Initial Upload
	{0x3065,0xF0}, //,Initial Upload
	{0x4013,0x13}, //,Initial Upload
	{0x401F,0x9}, //,Initial Upload
	{0x4020,0x13}, //,Initial Upload
	{0x4044,0x75}, //,Initial Upload
	{0x4027,0x0}, //,Initial Upload
	{0x3215,0x69}, //,Initial Upload
	{0x3216,0xF}, //,Initial Upload
	{0x322B,0x69}, //,Initial Upload
	{0x322C,0xF}, //,Initial Upload
	{0x4051,0x80}, //,Initial Upload
	{0x4052,0x10}, //,Initial Upload
	{0x4057,0x80}, //,Initial Upload
	{0x4058,0x10}, //,Initial Upload
	{0x3212,0x59}, //,Initial Upload
	{0x4047,0x8F}, //,Initial Upload
	{0x4026,0x10}, //,Initial Upload
	{0x4032,0x53}, //,Initial Upload
	{0x4036,0x17}, //,Initial Upload
	{0x50B8,0xF4}, //,Initial Upload
	{0x3016,0x0}, //,Initial Upload
	{0x3017,0x2C}, //,Initial Upload
	{0x3018,0x8C}, //,Initial Upload
	{0x3019,0x45}, //,Initial Upload
	{0x301A,0x5}, //,Initial Upload
	{0x3013,0xA}, //,Initial Upload
	{0x301B,0x0}, //,Initial Upload
	{0x301C,0x4}, //,Initial Upload
	{0x301D,0x88}, //,Initial Upload
	{0x301E,0x45}, //,Initial Upload
	{0x301F,0x5}, //,Initial Upload
	{0x3020,0x0}, //,Initial Upload
	{0x3021,0x4}, //,Initial Upload
	{0x3022,0x88}, //,Initial Upload
	{0x3023,0x45}, //,Initial Upload
	{0x3024,0x5}, //,Initial Upload
	{0x3025,0x0}, //,Initial Upload
	{0x3026,0x4}, //,Initial Upload
	{0x3027,0x88}, //,Initial Upload
	{0x3028,0x45}, //,Initial Upload
	{0x3029,0x5}, //,Initial Upload
	{0x302F,0x0}, //,Initial Upload
	{0x3056,0x0}, //,Initial Upload
	{0x3057,0x0}, //,Initial Upload
	{0x3300,0x1}, //,Initial Upload
	{0x3301,0x0}, //,Initial Upload
	{0x3302,0xB0}, //,Initial Upload
	{0x3303,0xB0}, //,Initial Upload
	{0x3304,0x16}, //,Initial Upload
	{0x3305,0x15}, //,Initial Upload
	{0x3306,0x1}, //,Initial Upload
	{0x3307,0x0}, //,Initial Upload
	{0x3308,0x30}, //,Initial Upload
	{0x3309,0xA0}, //,Initial Upload
	{0x330A,0x16}, //,Initial Upload
	{0x330B,0x15}, //,Initial Upload
	{0x330C,0x1}, //,Initial Upload
	{0x330D,0x0}, //,Initial Upload
	{0x330E,0x30}, //,Initial Upload
	{0x330F,0xA0}, //,Initial Upload
	{0x3310,0x16}, //,Initial Upload
	{0x3311,0x15}, //,Initial Upload
	{0x3312,0x1}, //,Initial Upload
	{0x3313,0x0}, //,Initial Upload
	{0x3314,0x30}, //,Initial Upload
	{0x3315,0xA0}, //,Initial Upload
	{0x3316,0x16}, //,Initial Upload
	{0x3317,0x15}, //,Initial Upload
	{0x3318,0x1}, //,Initial Upload
	{0x3319,0x0}, //,Initial Upload
	{0x331A,0x30}, //,Initial Upload
	{0x331B,0xA0}, //,Initial Upload
	{0x331C,0x16}, //,Initial Upload
	{0x331D,0x15}, //,Initial Upload
	{0x331E,0x1}, //,Initial Upload
	{0x331F,0x0}, //,Initial Upload
	{0x3320,0x30}, //,Initial Upload
	{0x3321,0xA0}, //,Initial Upload
	{0x3322,0x16}, //,Initial Upload
	{0x3323,0x15}, //,Initial Upload
	{0x3324,0x1}, //,Initial Upload
	{0x3325,0x0}, //,Initial Upload
	{0x3326,0x30}, //,Initial Upload
	{0x3327,0xA0}, //,Initial Upload
	{0x3328,0x16}, //,Initial Upload
	{0x3329,0x15}, //,Initial Upload
	{0x332A,0x2B}, //,Initial Upload
	{0x332B,0x0}, //,Initial Upload
	{0x332C,0x30}, //,Initial Upload
	{0x332D,0xA0}, //,Initial Upload
	{0x332E,0x16}, //,Initial Upload
	{0x332F,0x15}, //,Initial Upload
	{0x3330,0x1}, //,Initial Upload
	{0x3331,0x0}, //,Initial Upload
	{0x3332,0x10}, //,Initial Upload
	{0x3333,0xA0}, //,Initial Upload
	{0x3334,0x16}, //,Initial Upload
	{0x3335,0x15}, //,Initial Upload
	{0x3058,0x8}, //,Initial Upload
	{0x3059,0x0}, //,Initial Upload
	{0x305A,0x9}, //,Initial Upload
	{0x305B,0x0}, //,Initial Upload
	{0x3336,0x1}, //,Initial Upload
	{0x3337,0x0}, //,Initial Upload
	{0x3338,0x90}, //,Initial Upload
	{0x3339,0xB0}, //,Initial Upload
	{0x333A,0x16}, //,Initial Upload
	{0x333B,0x15}, //,Initial Upload
	{0x333C,0x1F}, //,Initial Upload
	{0x333D,0x0}, //,Initial Upload
	{0x333E,0x10}, //,Initial Upload
	{0x333F,0xA0}, //,Initial Upload
	{0x3340,0x16}, //,Initial Upload
	{0x3341,0x15}, //,Initial Upload
	{0x3342,0x52}, //,Initial Upload
	{0x3343,0x0}, //,Initial Upload
	{0x3344,0x10}, //,Initial Upload
	{0x3345,0x80}, //,Initial Upload
	{0x3346,0x16}, //,Initial Upload
	{0x3347,0x15}, //,Initial Upload
	{0x3348,0x1}, //,Initial Upload
	{0x3349,0x0}, //,Initial Upload
	{0x334A,0x10}, //,Initial Upload
	{0x334B,0x80}, //,Initial Upload
	{0x334C,0x16}, //,Initial Upload
	{0x334D,0x1D}, //,Initial Upload
	{0x334E,0x1}, //,Initial Upload
	{0x334F,0x0}, //,Initial Upload
	{0x3350,0x50}, //,Initial Upload
	{0x3351,0x84}, //,Initial Upload
	{0x3352,0x16}, //,Initial Upload
	{0x3353,0x1D}, //,Initial Upload
	{0x3354,0x18}, //,Initial Upload
	{0x3355,0x0}, //,Initial Upload
	{0x3356,0x10}, //,Initial Upload
	{0x3357,0x84}, //,Initial Upload
	{0x3358,0x16}, //,Initial Upload
	{0x3359,0x1D}, //,Initial Upload
	{0x335A,0x80}, //,Initial Upload
	{0x335B,0x2}, //,Initial Upload
	{0x335C,0x10}, //,Initial Upload
	{0x335D,0xC4}, //,Initial Upload
	{0x335E,0x14}, //,Initial Upload
	{0x335F,0x1D}, //,Initial Upload
	{0x3360,0xA5}, //,Initial Upload
	{0x3361,0x0}, //,Initial Upload
	{0x3362,0x10}, //,Initial Upload
	{0x3363,0x84}, //,Initial Upload
	{0x3364,0x16}, //,Initial Upload
	{0x3365,0x1D}, //,Initial Upload
	{0x3366,0x1}, //,Initial Upload
	{0x3367,0x0}, //,Initial Upload
	{0x3368,0x90}, //,Initial Upload
	{0x3369,0x84}, //,Initial Upload
	{0x336A,0x16}, //,Initial Upload
	{0x336B,0x1D}, //,Initial Upload
	{0x336C,0x12}, //,Initial Upload
	{0x336D,0x0}, //,Initial Upload
	{0x336E,0x10}, //,Initial Upload
	{0x336F,0x84}, //,Initial Upload
	{0x3370,0x16}, //,Initial Upload
	{0x3371,0x15}, //,Initial Upload
	{0x3372,0x32}, //,Initial Upload
	{0x3373,0x0}, //,Initial Upload
	{0x3374,0x30}, //,Initial Upload
	{0x3375,0x84}, //,Initial Upload
	{0x3376,0x16}, //,Initial Upload
	{0x3377,0x15}, //,Initial Upload
	{0x3378,0x26}, //,Initial Upload
	{0x3379,0x0}, //,Initial Upload
	{0x337A,0x10}, //,Initial Upload
	{0x337B,0x84}, //,Initial Upload
	{0x337C,0x16}, //,Initial Upload
	{0x337D,0x15}, //,Initial Upload
	{0x337E,0x80}, //,Initial Upload
	{0x337F,0x2}, //,Initial Upload
	{0x3380,0x10}, //,Initial Upload
	{0x3381,0xC4}, //,Initial Upload
	{0x3382,0x14}, //,Initial Upload
	{0x3383,0x15}, //,Initial Upload
	{0x3384,0xA9}, //,Initial Upload
	{0x3385,0x0}, //,Initial Upload
	{0x3386,0x10}, //,Initial Upload
	{0x3387,0x84}, //,Initial Upload
	{0x3388,0x16}, //,Initial Upload
	{0x3389,0x15}, //,Initial Upload
	{0x338A,0x41}, //,Initial Upload
	{0x338B,0x0}, //,Initial Upload
	{0x338C,0x10}, //,Initial Upload
	{0x338D,0x80}, //,Initial Upload
	{0x338E,0x16}, //,Initial Upload
	{0x338F,0x15}, //,Initial Upload
	{0x3390,0x2}, //,Initial Upload
	{0x3391,0x0}, //,Initial Upload
	{0x3392,0x10}, //,Initial Upload
	{0x3393,0xA0}, //,Initial Upload
	{0x3394,0x16}, //,Initial Upload
	{0x3395,0x15}, //,Initial Upload
	{0x305C,0x18}, //,Initial Upload
	{0x305D,0x0}, //,Initial Upload
	{0x305E,0x19}, //,Initial Upload
	{0x305F,0x0}, //,Initial Upload
	{0x3396,0x1}, //,Initial Upload
	{0x3397,0x0}, //,Initial Upload
	{0x3398,0x90}, //,Initial Upload
	{0x3399,0x30}, //,Initial Upload
	{0x339A,0x56}, //,Initial Upload
	{0x339B,0x57}, //,Initial Upload
	{0x339C,0x1}, //,Initial Upload
	{0x339D,0x0}, //,Initial Upload
	{0x339E,0x10}, //,Initial Upload
	{0x339F,0x20}, //,Initial Upload
	{0x33A0,0xD6}, //,Initial Upload
	{0x33A1,0x17}, //,Initial Upload
	{0x33A2,0x1}, //,Initial Upload
	{0x33A3,0x0}, //,Initial Upload
	{0x33A4,0x10}, //,Initial Upload
	{0x33A5,0x28}, //,Initial Upload
	{0x33A6,0xD6}, //,Initial Upload
	{0x33A7,0x17}, //,Initial Upload
	{0x33A8,0x3}, //,Initial Upload
	{0x33A9,0x0}, //,Initial Upload
	{0x33AA,0x10}, //,Initial Upload
	{0x33AB,0x20}, //,Initial Upload
	{0x33AC,0xD6}, //,Initial Upload
	{0x33AD,0x17}, //,Initial Upload
	{0x33AE,0x61}, //,Initial Upload
	{0x33AF,0x0}, //,Initial Upload
	{0x33B0,0x10}, //,Initial Upload
	{0x33B1,0x20}, //,Initial Upload
	{0x33B2,0xD6}, //,Initial Upload
	{0x33B3,0x15}, //,Initial Upload
	{0x33B4,0x1}, //,Initial Upload
	{0x33B5,0x0}, //,Initial Upload
	{0x33B6,0x10}, //,Initial Upload
	{0x33B7,0x20}, //,Initial Upload
	{0x33B8,0xD6}, //,Initial Upload
	{0x33B9,0x1D}, //,Initial Upload
	{0x33BA,0x1}, //,Initial Upload
	{0x33BB,0x0}, //,Initial Upload
	{0x33BC,0x50}, //,Initial Upload
	{0x33BD,0x20}, //,Initial Upload
	{0x33BE,0xD6}, //,Initial Upload
	{0x33BF,0x1D}, //,Initial Upload
	{0x33C0,0x2C}, //,Initial Upload
	{0x33C1,0x0}, //,Initial Upload
	{0x33C2,0x10}, //,Initial Upload
	{0x33C3,0x20}, //,Initial Upload
	{0x33C4,0xD6}, //,Initial Upload
	{0x33C5,0x1D}, //,Initial Upload
	{0x33C6,0x1}, //,Initial Upload
	{0x33C7,0x0}, //,Initial Upload
	{0x33C8,0x90}, //,Initial Upload
	{0x33C9,0x20}, //,Initial Upload
	{0x33CA,0xD6}, //,Initial Upload
	{0x33CB,0x1D}, //,Initial Upload
	{0x33CC,0x83}, //,Initial Upload
	{0x33CD,0x0}, //,Initial Upload
	{0x33CE,0x10}, //,Initial Upload
	{0x33CF,0x20}, //,Initial Upload
	{0x33D0,0xD6}, //,Initial Upload
	{0x33D1,0x15}, //,Initial Upload
	{0x33D2,0x1}, //,Initial Upload
	{0x33D3,0x0}, //,Initial Upload
	{0x33D4,0x10}, //,Initial Upload
	{0x33D5,0x30}, //,Initial Upload
	{0x33D6,0xD6}, //,Initial Upload
	{0x33D7,0x15}, //,Initial Upload
	{0x33D8,0x1}, //,Initial Upload
	{0x33D9,0x0}, //,Initial Upload
	{0x33DA,0x10}, //,Initial Upload
	{0x33DB,0x20}, //,Initial Upload
	{0x33DC,0xD6}, //,Initial Upload
	{0x33DD,0x15}, //,Initial Upload
	{0x33DE,0x1}, //,Initial Upload
	{0x33DF,0x0}, //,Initial Upload
	{0x33E0,0x10}, //,Initial Upload
	{0x33E1,0x20}, //,Initial Upload
	{0x33E2,0x56}, //,Initial Upload
	{0x33E3,0x15}, //,Initial Upload
	{0x33E4,0x7}, //,Initial Upload
	{0x33E5,0x0}, //,Initial Upload
	{0x33E6,0x10}, //,Initial Upload
	{0x33E7,0x20}, //,Initial Upload
	{0x33E8,0x16}, //,Initial Upload
	{0x33E9,0x15}, //,Initial Upload
	{0x3060,0x26}, //,Initial Upload
	{0x3061,0x0}, //,Initial Upload
	{0x302A,0xFF}, //,Initial Upload
	{0x302B,0xFF}, //,Initial Upload
	{0x302C,0xFF}, //,Initial Upload
	{0x302D,0xFF}, //,Initial Upload
	{0x302E,0x3F}, //,Initial Upload
	{0x3013,0xB}, //,Initial Upload
	{0x102B,0x2C}, //,Initial Upload
	{0x102C,0x1}, //,Initial Upload
	{0x1035,0x54}, //,Initial Upload
	{0x1036,0x0}, //,Initial Upload
	{0x3090,0x2A}, //,Initial Upload
	{0x3091,0x1}, //,Initial Upload
	{0x30C6,0x5}, //,Initial Upload
	{0x30C7,0x0}, //,Initial Upload
	{0x30C8,0x0}, //,Initial Upload
	{0x30C9,0x0}, //,Initial Upload
	{0x30CA,0x0}, //,Initial Upload
	{0x30CB,0x0}, //,Initial Upload
	{0x30CC,0x0}, //,Initial Upload
	{0x30CD,0x0}, //,Initial Upload
	{0x30CE,0x0}, //,Initial Upload
	{0x30CF,0x5}, //,Initial Upload
	{0x30D0,0x0}, //,Initial Upload
	{0x30D1,0x0}, //,Initial Upload
	{0x30D2,0x0}, //,Initial Upload
	{0x30D3,0x0}, //,Initial Upload
	{0x30D4,0x0}, //,Initial Upload
	{0x30D5,0x0}, //,Initial Upload
	{0x30D6,0x0}, //,Initial Upload
	{0x30D7,0x0}, //,Initial Upload
	{0x30F3,0x5}, //,Initial Upload
	{0x30F4,0x0}, //,Initial Upload
	{0x30F5,0x0}, //,Initial Upload
	{0x30F6,0x0}, //,Initial Upload
	{0x30F7,0x0}, //,Initial Upload
	{0x30F8,0x0}, //,Initial Upload
	{0x30F9,0x0}, //,Initial Upload
	{0x30FA,0x0}, //,Initial Upload
	{0x30FB,0x0}, //,Initial Upload
	{0x30D8,0x5}, //,Initial Upload
	{0x30D9,0x0}, //,Initial Upload
	{0x30DA,0x0}, //,Initial Upload
	{0x30DB,0x0}, //,Initial Upload
	{0x30DC,0x0}, //,Initial Upload
	{0x30DD,0x0}, //,Initial Upload
	{0x30DE,0x0}, //,Initial Upload
	{0x30DF,0x0}, //,Initial Upload
	{0x30E0,0x0}, //,Initial Upload
	{0x30E1,0x5}, //,Initial Upload
	{0x30E2,0x0}, //,Initial Upload
	{0x30E3,0x0}, //,Initial Upload
	{0x30E4,0x0}, //,Initial Upload
	{0x30E5,0x0}, //,Initial Upload
	{0x30E6,0x0}, //,Initial Upload
	{0x30E7,0x0}, //,Initial Upload
	{0x30E8,0x0}, //,Initial Upload
	{0x30E9,0x0}, //,Initial Upload
	{0x30F3,0x5}, //,Initial Upload
	{0x30F4,0x2}, //,Initial Upload
	{0x30F5,0x0}, //,Initial Upload
	{0x30F6,0x17}, //,Initial Upload
	{0x30F7,0x1}, //,Initial Upload
	{0x30F8,0x0}, //,Initial Upload
	{0x30F9,0x0}, //,Initial Upload
	{0x30FA,0x0}, //,Initial Upload
	{0x30FB,0x0}, //,Initial Upload
	{0x30D8,0x3}, //,Initial Upload
	{0x30D9,0x1}, //,Initial Upload
	{0x30DA,0x0}, //,Initial Upload
	{0x30DB,0x19}, //,Initial Upload
	{0x30DC,0x1}, //,Initial Upload
	{0x30DD,0x0}, //,Initial Upload
	{0x30DE,0x0}, //,Initial Upload
	{0x30DF,0x0}, //,Initial Upload
	{0x30E0,0x0}, //,Initial Upload
	{0x30A2,0x5}, //,Initial Upload
	{0x30A3,0x2}, //,Initial Upload
	{0x30A4,0x0}, //,Initial Upload
	{0x30A5,0x22}, //,Initial Upload
	{0x30A6,0x0}, //,Initial Upload
	{0x30A7,0x0}, //,Initial Upload
	{0x30A8,0x0}, //,Initial Upload
	{0x30A9,0x0}, //,Initial Upload
	{0x30AA,0x0}, //,Initial Upload
	{0x30AB,0x5}, //,Initial Upload
	{0x30AC,0x2}, //,Initial Upload
	{0x30AD,0x0}, //,Initial Upload
	{0x30AE,0x22}, //,Initial Upload
	{0x30AF,0x0}, //,Initial Upload
	{0x30B0,0x0}, //,Initial Upload
	{0x30B1,0x0}, //,Initial Upload
	{0x30B2,0x0}, //,Initial Upload
	{0x30B3,0x0}, //,Initial Upload
	{0x30BD,0x5}, //,Initial Upload
	{0x30BE,0x9F}, //,Initial Upload
	{0x30BF,0x0}, //,Initial Upload
	{0x30C0,0x7D}, //,Initial Upload
	{0x30C1,0x0}, //,Initial Upload
	{0x30C2,0x0}, //,Initial Upload
	{0x30C3,0x0}, //,Initial Upload
	{0x30C4,0x0}, //,Initial Upload
	{0x30C5,0x0}, //,Initial Upload
	{0x30B4,0x4}, //,Initial Upload
	{0x30B5,0x9C}, //,Initial Upload
	{0x30B6,0x0}, //,Initial Upload
	{0x30B7,0x7D}, //,Initial Upload
	{0x30B8,0x0}, //,Initial Upload
	{0x30B9,0x0}, //,Initial Upload
	{0x30BA,0x0}, //,Initial Upload
	{0x30BB,0x0}, //,Initial Upload
	{0x30BC,0x0}, //,Initial Upload
	{0x30FC,0x5}, //,Initial Upload
	{0x30FD,0x0}, //,Initial Upload
	{0x30FE,0x0}, //,Initial Upload
	{0x30FF,0x0}, //,Initial Upload
	{0x3100,0x0}, //,Initial Upload
	{0x3101,0x0}, //,Initial Upload
	{0x3102,0x0}, //,Initial Upload
	{0x3103,0x0}, //,Initial Upload
	{0x3104,0x0}, //,Initial Upload
	{0x3105,0x5}, //,Initial Upload
	{0x3106,0x0}, //,Initial Upload
	{0x3107,0x0}, //,Initial Upload
	{0x3108,0x0}, //,Initial Upload
	{0x3109,0x0}, //,Initial Upload
	{0x310A,0x0}, //,Initial Upload
	{0x310B,0x0}, //,Initial Upload
	{0x310C,0x0}, //,Initial Upload
	{0x310D,0x0}, //,Initial Upload
	{0x3099,0x5}, //,Initial Upload
	{0x309A,0x96}, //,Initial Upload
	{0x309B,0x0}, //,Initial Upload
	{0x309C,0x6}, //,Initial Upload
	{0x309D,0x0}, //,Initial Upload
	{0x309E,0x0}, //,Initial Upload
	{0x309F,0x0}, //,Initial Upload
	{0x30A0,0x0}, //,Initial Upload
	{0x30A1,0x0}, //,Initial Upload
	{0x310E,0x5}, //,Initial Upload
	{0x310F,0x2}, //,Initial Upload
	{0x3110,0x0}, //,Initial Upload
	{0x3111,0x2B}, //,Initial Upload
	{0x3112,0x0}, //,Initial Upload
	{0x3113,0x0}, //,Initial Upload
	{0x3114,0x0}, //,Initial Upload
	{0x3115,0x0}, //,Initial Upload
	{0x3116,0x0}, //,Initial Upload
	{0x3117,0x5}, //,Initial Upload
	{0x3118,0x2}, //,Initial Upload
	{0x3119,0x0}, //,Initial Upload
	{0x311A,0x2C}, //,Initial Upload
	{0x311B,0x0}, //,Initial Upload
	{0x311C,0x0}, //,Initial Upload
	{0x311D,0x0}, //,Initial Upload
	{0x311E,0x0}, //,Initial Upload
	{0x311F,0x0}, //,Initial Upload
	{0x30EA,0x0}, //,Initial Upload
	{0x30EB,0x0}, //,Initial Upload
	{0x30EC,0x0}, //,Initial Upload
	{0x30ED,0x0}, //,Initial Upload
	{0x30EE,0x0}, //,Initial Upload
	{0x30EF,0x0}, //,Initial Upload
	{0x30F0,0x0}, //,Initial Upload
	{0x30F1,0x0}, //,Initial Upload
	{0x30F2,0x0}, //,Initial Upload
	{0x313B,0x3}, //,Initial Upload
	{0x313C,0x31}, //,Initial Upload
	{0x313D,0x0}, //,Initial Upload
	{0x313E,0x7}, //,Initial Upload
	{0x313F,0x0}, //,Initial Upload
	{0x3140,0x68}, //,Initial Upload
	{0x3141,0x0}, //,Initial Upload
	{0x3142,0x34}, //,Initial Upload
	{0x3143,0x0}, //,Initial Upload
	{0x31A0,0x3}, //,Initial Upload
	{0x31A1,0x16}, //,Initial Upload
	{0x31A2,0x0}, //,Initial Upload
	{0x31A3,0x8}, //,Initial Upload
	{0x31A4,0x0}, //,Initial Upload
	{0x31A5,0x7E}, //,Initial Upload
	{0x31A6,0x0}, //,Initial Upload
	{0x31A7,0x8}, //,Initial Upload
	{0x31A8,0x0}, //,Initial Upload
	{0x31A9,0x3}, //,Initial Upload
	{0x31AA,0x16}, //,Initial Upload
	{0x31AB,0x0}, //,Initial Upload
	{0x31AC,0x8}, //,Initial Upload
	{0x31AD,0x0}, //,Initial Upload
	{0x31AE,0x7E}, //,Initial Upload
	{0x31AF,0x0}, //,Initial Upload
	{0x31B0,0x8}, //,Initial Upload
	{0x31B1,0x0}, //,Initial Upload
	{0x31B2,0x3}, //,Initial Upload
	{0x31B3,0x16}, //,Initial Upload
	{0x31B4,0x0}, //,Initial Upload
	{0x31B5,0x8}, //,Initial Upload
	{0x31B6,0x0}, //,Initial Upload
	{0x31B7,0x7E}, //,Initial Upload
	{0x31B8,0x0}, //,Initial Upload
	{0x31B9,0x8}, //,Initial Upload
	{0x31BA,0x0}, //,Initial Upload
	{0x3120,0x5}, //,Initial Upload
	{0x3121,0x45}, //,Initial Upload
	{0x3122,0x0}, //,Initial Upload
	{0x3123,0x1D}, //,Initial Upload
	{0x3124,0x0}, //,Initial Upload
	{0x3125,0xA9}, //,Initial Upload
	{0x3126,0x0}, //,Initial Upload
	{0x3127,0x6D}, //,Initial Upload
	{0x3128,0x0}, //,Initial Upload
	{0x3129,0x5}, //,Initial Upload
	{0x312A,0x15}, //,Initial Upload
	{0x312B,0x0}, //,Initial Upload
	{0x312C,0xA}, //,Initial Upload
	{0x312D,0x0}, //,Initial Upload
	{0x312E,0x45}, //,Initial Upload
	{0x312F,0x0}, //,Initial Upload
	{0x3130,0x1D}, //,Initial Upload
	{0x3131,0x0}, //,Initial Upload
	{0x3132,0x5}, //,Initial Upload
	{0x3133,0x7D}, //,Initial Upload
	{0x3134,0x0}, //,Initial Upload
	{0x3135,0xA}, //,Initial Upload
	{0x3136,0x0}, //,Initial Upload
	{0x3137,0xA9}, //,Initial Upload
	{0x3138,0x0}, //,Initial Upload
	{0x3139,0x6D}, //,Initial Upload
	{0x313A,0x0}, //,Initial Upload
	{0x3144,0x5}, //,Initial Upload
	{0x3145,0x0}, //,Initial Upload
	{0x3146,0x0}, //,Initial Upload
	{0x3147,0x30}, //,Initial Upload
	{0x3148,0x0}, //,Initial Upload
	{0x3149,0x0}, //,Initial Upload
	{0x314A,0x0}, //,Initial Upload
	{0x314B,0x0}, //,Initial Upload
	{0x314C,0x0}, //,Initial Upload
	{0x314D,0x3}, //,Initial Upload
	{0x314E,0x0}, //,Initial Upload
	{0x314F,0x0}, //,Initial Upload
	{0x3150,0x31}, //,Initial Upload
	{0x3151,0x0}, //,Initial Upload
	{0x3152,0x0}, //,Initial Upload
	{0x3153,0x0}, //,Initial Upload
	{0x3154,0x0}, //,Initial Upload
	{0x3155,0x0}, //,Initial Upload
	{0x31D8,0x5}, //,Initial Upload
	{0x31D9,0x3A}, //,Initial Upload
	{0x31DA,0x0}, //,Initial Upload
	{0x31DB,0x2E}, //,Initial Upload
	{0x31DC,0x0}, //,Initial Upload
	{0x31DD,0x9E}, //,Initial Upload
	{0x31DE,0x0}, //,Initial Upload
	{0x31DF,0x7E}, //,Initial Upload
	{0x31E0,0x0}, //,Initial Upload
	{0x31E1,0x5}, //,Initial Upload
	{0x31E2,0x4}, //,Initial Upload
	{0x31E3,0x0}, //,Initial Upload
	{0x31E4,0x4}, //,Initial Upload
	{0x31E5,0x0}, //,Initial Upload
	{0x31E6,0x73}, //,Initial Upload
	{0x31E7,0x0}, //,Initial Upload
	{0x31E8,0x4}, //,Initial Upload
	{0x31E9,0x0}, //,Initial Upload
	{0x31EA,0x5}, //,Initial Upload
	{0x31EB,0x0}, //,Initial Upload
	{0x31EC,0x0}, //,Initial Upload
	{0x31ED,0x0}, //,Initial Upload
	{0x31EE,0x0}, //,Initial Upload
	{0x31EF,0x0}, //,Initial Upload
	{0x31F0,0x0}, //,Initial Upload
	{0x31F1,0x0}, //,Initial Upload
	{0x31F2,0x0}, //,Initial Upload
	{0x31F3,0x0}, //,Initial Upload
	{0x31F4,0x0}, //,Initial Upload
	{0x31F5,0x0}, //,Initial Upload
	{0x31F6,0x0}, //,Initial Upload
	{0x31F7,0x0}, //,Initial Upload
	{0x31F8,0x0}, //,Initial Upload
	{0x31F9,0x0}, //,Initial Upload
	{0x31FA,0x0}, //,Initial Upload
	{0x31FB,0x5}, //,Initial Upload
	{0x31FC,0x0}, //,Initial Upload
	{0x31FD,0x0}, //,Initial Upload
	{0x31FE,0x0}, //,Initial Upload
	{0x31FF,0x0}, //,Initial Upload
	{0x3200,0x0}, //,Initial Upload
	{0x3201,0x0}, //,Initial Upload
	{0x3202,0x0}, //,Initial Upload
	{0x3203,0x0}, //,Initial Upload
	{0x3204,0x0}, //,Initial Upload
	{0x3205,0x0}, //,Initial Upload
	{0x3206,0x0}, //,Initial Upload
	{0x3207,0x0}, //,Initial Upload
	{0x3208,0x0}, //,Initial Upload
	{0x3209,0x0}, //,Initial Upload
	{0x320A,0x0}, //,Initial Upload
	{0x320B,0x0}, //,Initial Upload
	{0x3164,0x5}, //,Initial Upload
	{0x3165,0x14}, //,Initial Upload
	{0x3166,0x0}, //,Initial Upload
	{0x3167,0xC}, //,Initial Upload
	{0x3168,0x0}, //,Initial Upload
	{0x3169,0x44}, //,Initial Upload
	{0x316A,0x0}, //,Initial Upload
	{0x316B,0x1F}, //,Initial Upload
	{0x316C,0x0}, //,Initial Upload
	{0x316D,0x5}, //,Initial Upload
	{0x316E,0x7C}, //,Initial Upload
	{0x316F,0x0}, //,Initial Upload
	{0x3170,0xC}, //,Initial Upload
	{0x3171,0x0}, //,Initial Upload
	{0x3172,0xA8}, //,Initial Upload
	{0x3173,0x0}, //,Initial Upload
	{0x3174,0x6F}, //,Initial Upload
	{0x3175,0x0}, //,Initial Upload
	{0x31C4,0x5}, //,Initial Upload
	{0x31C5,0x24}, //,Initial Upload
	{0x31C6,0x1}, //,Initial Upload
	{0x31C7,0x4}, //,Initial Upload
	{0x31C8,0x0}, //,Initial Upload
	{0x31C9,0x5}, //,Initial Upload
	{0x31CA,0x24}, //,Initial Upload
	{0x31CB,0x1}, //,Initial Upload
	{0x31CC,0x4}, //,Initial Upload
	{0x31CD,0x0}, //,Initial Upload
	{0x31CE,0x5}, //,Initial Upload
	{0x31CF,0x24}, //,Initial Upload
	{0x31D0,0x1}, //,Initial Upload
	{0x31D1,0x4}, //,Initial Upload
	{0x31D2,0x0}, //,Initial Upload
	{0x31D3,0x5}, //,Initial Upload
	{0x31D4,0x73}, //,Initial Upload
	{0x31D5,0x0}, //,Initial Upload
	{0x31D6,0xB1}, //,Initial Upload
	{0x31D7,0x0}, //,Initial Upload
	{0x3176,0x5}, //,Initial Upload
	{0x3177,0x10}, //,Initial Upload
	{0x3178,0x0}, //,Initial Upload
	{0x3179,0x56}, //,Initial Upload
	{0x317A,0x0}, //,Initial Upload
	{0x317B,0x0}, //,Initial Upload
	{0x317C,0x0}, //,Initial Upload
	{0x317D,0x0}, //,Initial Upload
	{0x317E,0x0}, //,Initial Upload
	{0x317F,0x5}, //,Initial Upload
	{0x3180,0x6A}, //,Initial Upload
	{0x3181,0x0}, //,Initial Upload
	{0x3182,0xAD}, //,Initial Upload
	{0x3183,0x0}, //,Initial Upload
	{0x3184,0x0}, //,Initial Upload
	{0x3185,0x0}, //,Initial Upload
	{0x3186,0x0}, //,Initial Upload
	{0x3187,0x0}, //,Initial Upload
	{0x100C,0x7E}, //,Initial Upload
	{0x100D,0x0}, //,Initial Upload
	{0x1012,0xDF}, //,Initial Upload
	{0x1013,0x2B}, //,Initial Upload
	{0x1002,0x4}, //,Initial Upload
	// {0x1003,0x10}, //,Sensor Control Mode.IMAGER_STATE(0)
	{0x0043,0x0}, //,Sensor Control Mode.SLEEP_POWER_MODE(0)
	{0x0043,0x0}, //,Sensor Control Mode.IDLE_POWER_MODE(0)
	{0x0043,0x4}, //,Sensor Control Mode.SYSTEM_CLOCK_ENABLE(0)
	{0x0043,0xC}, //,Sensor Control Mode.SRAM_CLOCK_ENABLE(0)
	{0x1002,0x4}, //,Sensor Control Mode.IMAGER_RUN_CONT(0)
	{0x1001,0x41}, //,Sensor Control Mode.EXT_EVENT_SEL(0)
	{0x10F2,0x1}, //,Sensor Control Mode.NB_OF_FRAMES_A(0)
	{0x10F3,0x0}, //,Sensor Control Mode.NB_OF_FRAMES_A(1)
	{0x1111,0x1}, //,Sensor Control Mode.NB_OF_FRAMES_B(0)
	{0x1112,0x0}, //,Sensor Control Mode.NB_OF_FRAMES_B(1)
	{0x0012,0x0}, //,IO Drive Strength.DIG_DRIVE_STRENGTH(0)
	{0x0012,0x0}, //,IO Drive Strength.CCI_DRIVE_STRENGTH(0)
	{0x1001,0x41}, //,Readout && Exposure.EXT_EXP_PW_SEL(0)
	{0x10D0,0x0}, //,Readout && Exposure.EXT_EXP_PW_DELAY(0)
	{0x10D1,0x0}, //,Readout && Exposure.EXT_EXP_PW_DELAY(1)
	{0x1012,0x14}, //,Readout && Exposure.VBLANK_A(0)
	{0x1013,0x0}, //,Readout && Exposure.VBLANK_A(1)
	{0x1103,0x91}, //,Readout && Exposure.VBLANK_B(0)
	{0x1104,0xD}, //,Readout && Exposure.VBLANK_B(1)
	{0x100C,0x80}, //,Readout && Exposure.EXP_TIME_A(0)
	{0x100D,0x0}, //,Readout && Exposure.EXP_TIME_A(1)
	{0x1115,0x80}, //,Readout && Exposure.EXP_TIME_B(0)
	{0x1116,0x0}, //,Readout && Exposure.EXP_TIME_B(1)
	{0x102B,0x30}, //,Readout && Exposure.ROW_LENGTH_A(0)
	{0x102C,0x1}, //,Readout && Exposure.ROW_LENGTH_A(1)
	{0x1113,0x30}, //,Readout && Exposure.ROW_LENGTH_B(0)
	{0x1114,0x1}, //,Readout && Exposure.ROW_LENGTH_B(1)
	{0x2008,0xC8}, //,Horizontal ROI.HSIZE_A(0)
	{0x2009,0x0}, //,Horizontal ROI.HSIZE_A(1)
	{0x2098,0xC8}, //,Horizontal ROI.HSIZE_B(0)
	{0x2099,0x0}, //,Horizontal ROI.HSIZE_B(1)
	{0x200A,0x58}, //,Horizontal ROI.HSTART_A(0)
	{0x200B,0x2}, //,Horizontal ROI.HSTART_A(1)
	{0x209A,0x58}, //,Horizontal ROI.HSTART_B(0)
	{0x209B,0x2}, //,Horizontal ROI.HSTART_B(1)

	{0x107D,0x2C}, //,Vertical ROI.VSTART0_A(0)
	{0x107E,0x1}, //,Vertical ROI.VSTART0_A(1)
	{0x1087,0x90}, //,Vertical ROI.VSIZE0_A(0)
	{0x1088,0x1}, //,Vertical ROI.VSIZE0_A(1)

	{0x1105,0x2C}, //,Vertical ROI.VSTART0_B(0)
	{0x1106,0x1}, //,Vertical ROI.VSTART0_B(1)
	{0x110A,0x90}, //,Vertical ROI.VSIZE0_B(0)
	{0x110B,0x1}, //,Vertical ROI.VSIZE0_B(1)

	{0x107D,0x2C}, //,Vertical ROI.VSTART1_A(0)
	{0x107E,0x1}, //,Vertical ROI.VSTART1_A(1)
	{0x107F,0x0}, //,Vertical ROI.VSTART1_A(2)
	{0x1087,0x90}, //,Vertical ROI.VSIZE1_A(0)
	{0x1088,0x2C}, //,Vertical ROI.VSIZE1_A(1)
	{0x1089,0x1},  //,Vertical ROI.VSIZE1_A(2)

	{0x1105,0x2C},  //,Vertical ROI.VSTART1_B(0)
	{0x1106,0x1}, //,Vertical ROI.VSTART1_B(1)
	{0x1107,0x0}, //,Vertical ROI.VSTART1_B(2)
	{0x110A,0x90}, //,Vertical ROI.VSIZE1_B(0)
	{0x110B,0x2C}, //,Vertical ROI.VSIZE1_B(1)
	{0x110C,0x1}, //,Vertical ROI.VSIZE1_B(2)

	{0x107D,0x2C}, //,Vertical ROI.VSTART2_A(0)
	{0x107E,0x1}, //,Vertical ROI.VSTART2_A(1)
	{0x107F,0x0}, //,Vertical ROI.VSTART2_A(2)
	{0x1080,0x0}, //,Vertical ROI.VSTART2_A(3)
	{0x1081,0x0}, //,Vertical ROI.VSTART2_A(4)
	{0x1087,0x90}, //,Vertical ROI.VSIZE2_A(0)
	{0x1088,0x1}, //,Vertical ROI.VSIZE2_A(1)
	{0x1089,0x0}, //,Vertical ROI.VSIZE2_A(2)
	{0x108A,0x0}, //,Vertical ROI.VSIZE2_A(3)
	{0x108B,0x0}, //,Vertical ROI.VSIZE2_A(4)

	{0x1105,0x2C}, //,Vertical ROI.VSTART2_B(0)
	{0x1106,0x1}, //,Vertical ROI.VSTART2_B(1)
	{0x1107,0x0}, //,Vertical ROI.VSTART2_B(2)
	{0x1108,0x0}, //,Vertical ROI.VSTART2_B(3)
	{0x1109,0x0}, //,Vertical ROI.VSTART2_B(4)
	{0x110A,0x90}, //,Vertical ROI.VSIZE2_B(0)
	{0x110B,0x1}, //,Vertical ROI.VSIZE2_B(1)
	{0x110C,0x0}, //,Vertical ROI.VSIZE2_B(2)
	{0x110D,0x0}, //,Vertical ROI.VSIZE2_B(3)
	{0x110E,0x0}, //,Vertical ROI.VSIZE2_B(4)
	
	{0x209C,0x0}, //,Mirroring && Flipping.HFLIP_A(0)
	{0x209D,0x0}, //,Mirroring && Flipping.HFLIP_B(0)
	{0x1095,0x0}, //,Mirroring && Flipping.VFLIP(0)
	{0x2063,0x0}, //,Mirroring && Flipping.BIT_ORDER(0)
	{0x6006,0x0}, //,MIPI.TX_CTRL_EN(0)
	{0x207D,0x90}, //,Horizontal ROI.MIPI_HSIZE(0)
	{0x207E,0x1}, //,Horizontal ROI.MIPI_HSIZE(1)
	{0x5004,0x1}, //,MIPI.datarate
	{0x5086,0x2}, //,MIPI.datarate
	{0x5087,0x4E}, //,MIPI.datarate
	{0x5088,0x0}, //,MIPI.datarate
	{0x5090,0x0}, //,MIPI.datarate
	{0x5091,0x8}, //,MIPI.datarate
	{0x5092,0x14}, //,MIPI.datarate
	{0x5093,0xF}, //,MIPI.datarate
	{0x5094,0x6}, //,MIPI.datarate
	{0x5095,0x32}, //,MIPI.datarate
	{0x5096,0xE}, //,MIPI.datarate
	{0x5097,0x0}, //,MIPI.datarate
	{0x5098,0x11}, //,MIPI.datarate
	{0x5004,0x0}, //,MIPI.datarate
	{0x2066,0x6C}, //,MIPI.datarate
	{0x2067,0x7}, //,MIPI.datarate
	{0x206E,0x7E}, //,MIPI.datarate
	{0x206F,0x6}, //,MIPI.datarate
	{0x20AC,0x7E}, //,MIPI.datarate
	{0x20AD,0x6}, //,MIPI.datarate
	{0x2076,0xC8}, //,MIPI.datarate
	{0x2077,0x0}, //,MIPI.datarate
	{0x20B4,0xC8}, //,MIPI.datarate
	{0x20B5,0x0}, //,MIPI.datarate
	{0x2078,0x1E}, //,MIPI.datarate
	{0x2079,0x4}, //,MIPI.datarate
	{0x20B6,0x1E}, //,MIPI.datarate
	{0x20B7,0x4}, //,MIPI.datarate
	{0x207A,0xD4}, //,MIPI.datarate
	{0x207B,0x4}, //,MIPI.datarate
	{0x20B8,0xD4}, //,MIPI.datarate
	{0x20B9,0x4}, //,MIPI.datarate
	{0x208D,0x4}, //,MIPI.CSI2_DTYPE(0)
	{0x208E,0x0}, //,MIPI.CSI2_DTYPE(1)
	{0x207C,0x0}, //,MIPI.VC_ID(0)
	{0x6001,0x7}, //,MIPI.TINIT(0)
	{0x6002,0xD8}, //,MIPI.TINIT(1)
	{0x6010,0x0}, //,MIPI.FRAME_MODE(0)
	{0x6010,0x0}, //,MIPI.EMBEDDED_FRAME_MODE(0)
	{0x6011,0x0}, //,MIPI.DATA_ENABLE_POLARITY(0)
	{0x6011,0x0}, //,MIPI.HSYNC_POLARITY(0)
	{0x6011,0x0}, //,MIPI.VSYNC_POLARITY(0)
	{0x6012,0x1}, //,MIPI.LANE(0)
	{0x6013,0x0}, //,MIPI.CLK_MODE(0)
	{0x6016,0x0}, //,MIPI.FRAME_COUNTER(0)
	{0x6017,0x0}, //,MIPI.FRAME_COUNTER(1)
	{0x6037,0x1}, //,MIPI.LINE_COUNT_RAW8(0)
	{0x6037,0x3}, //,MIPI.LINE_COUNT_RAW10(0)
	{0x6037,0x7}, //,MIPI.LINE_COUNT_RAW12(0)
	{0x6039,0x1}, //,MIPI.LINE_COUNT_EMB(0)
	{0x6018,0x0}, //,MIPI.CCI_READ_INTERRUPT_EN(0)
	{0x6018,0x0}, //,MIPI.CCI_WRITE_INTERRUPT_EN(0)
	{0x6065,0x0}, //,MIPI.TWAKE_TIMER(0)
	{0x6066,0x0}, //,MIPI.TWAKE_TIMER(1)
	{0x601C,0x0}, //,MIPI.SKEW_CAL_EN(0)
	{0x601D,0x0}, //,MIPI.SKEW_COUNT(0)
	{0x601E,0x22}, //,MIPI.SKEW_COUNT(1)
	{0x601F,0x0}, //,MIPI.SCRAMBLING_EN(0)
	{0x6003,0x1}, //,MIPI.INIT_SKEW_EN(0)
	{0x6004,0x7A}, //,MIPI.INIT_SKEW(0)
	{0x6005,0x12}, //,MIPI.INIT_SKEW(1)
	{0x6006,0x1}, //,MIPI.TX_CTRL_EN(0)
	{0x4006,0x8}, //,Processing.BSP(0)
	{0x209E,0x2}, //,Processing.BIT_DEPTH(0)
	{0x2045,0x1}, //,Processing.CDS_RNC(0)
	{0x2048,0x1}, //,Processing.CDS_IMG(0)
	{0x204B,0x3}, //,Processing.RNC_EN(0)
	{0x205B,0x64}, //,Processing.RNC_DARK_TARGET(0)
	{0x205C,0x0}, //,Processing.RNC_DARK_TARGET(1)
	{0x24DC,0x12}, //,Defect Pixel Correction.DC_ENABLE(0)
	{0x24DC,0x10}, //,Defect Pixel Correction.DC_MODE(0)
	{0x24DC,0x0}, //,Defect Pixel Correction.DC_REPLACEMENT_VALUE(0)
	{0x24DD,0x0}, //,Defect Pixel Correction.DC_LIMIT_LOW(0)
	{0x24DE,0x0}, //,Defect Pixel Correction.DC_LIMIT_HIGH(0)
	{0x24DF,0x0}, //,Defect Pixel Correction.DC_LIMIT_HIGH_MODE(0)
	{0x10D7,0x1}, //,Illumination Trigger.ILLUM_EN(0)
	{0x10D8,0x2}, //,Illumination Trigger.ILLUM_POL(0)
	{0x205D,0x0}, //,Histogram.HIST_EN(0)
	{0x205E,0x0}, //,Histogram.HIST_USAGE_RATIO(0)
	{0x2063,0x0}, //,Histogram.PIXEL_DATA_SUPP(0)
	{0x2063,0x0}, //,Histogram.PIXEL_TRANSMISSION(0)
	{0x2091,0x0}, //,Test Pattern Generator.TPG_EN(0)
	{0x2091,0x0}, //,Test Pattern Generator.TPG_CONFIG(0)
	{0x400a,0x8}, //,gain a
	{0x401a,0x8}, //,gain b
};


static const char * const mira220_test_pattern_menu[] = {
	"Disabled",
	"Vertial Gradient",
};

static const int mira220_test_pattern_val[] = {
	MIRA220_TEST_PATTERN_DISABLE,
	MIRA220_TEST_PATTERN_VERTICAL_GRADIENT,
};


/* regulator supplies */
static const char * const mira220_supply_name[] = {
	// TODO(jalv): Check supply names
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.8V) supply */
	"VDDL",  /* IF (1.2V) supply */
};

#define MIRA220_NUM_SUPPLIES ARRAY_SIZE(mira220_supply_name)

/*
 * The supported formats. All flip/mirror combinations have the same byte order because the sensor
 * is monochrome
 */
static const u32 codes[] = {
	//MEDIA_BUS_FMT_Y8_1X8,
	//MEDIA_BUS_FMT_Y10_1X10,
	//MEDIA_BUS_FMT_Y12_1X12,
	MEDIA_BUS_FMT_SGRBG8_1X8,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGRBG12_1X12,
};

/* Mode configs */
static const struct mira220_mode supported_modes[] = {

	/* 2 MPx 30fps 12bpp mode */
	{
		.width = 1600,
		.height = 1400,
		.crop = {
			.left = MIRA220_PIXEL_ARRAY_LEFT,
			.top = MIRA220_PIXEL_ARRAY_TOP,
			.width = 1600,
			.height = 1400
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(full_1600_1400_1500_12b_2lanes_reg),
			.regs = full_1600_1400_1500_12b_2lanes_reg,
		},
		// vblank is ceil(MIRA220_GLOB_NUM_CLK_CYCLES / ROW_LENGTH)  + 11
		// ROW_LENGTH is configured by register 0x102B, 0x102C.
		.row_length = 304,
		.pixel_rate = MIRA220_PIXEL_RATE,
		.min_vblank = 20, // ceil(1928 / 300) + 11
		.max_vblank = 50000, // ceil(1928 / 300) + 11
		.hblank = MIRA220_HBLANK_1600x1400_304, // TODO
		.code = MEDIA_BUS_FMT_SGRBG12_1X12,
	},

	/* VGA 120fps 8bpp mode */
	{
		.width = 640,
		.height = 480,
		.crop = {
			.left = 480,
			.top = 460,
			.width = 640,
			.height = 480
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(vga_640_480_120fps_12b_2lanes_reg),
			.regs = vga_640_480_120fps_12b_2lanes_reg,
		},
		// vblank is ceil(MIRA220_GLOB_NUM_CLK_CYCLES / ROW_LENGTH)  + 11
		// ROW_LENGTH is configured by register 0x102B, 0x102C.
		.row_length = 304,
		.pixel_rate = MIRA220_PIXEL_RATE,
		.min_vblank = 20, // ceil(1928 / 300) + 11
		.max_vblank = 50000, // ceil(1928 / 300) + 11
		.hblank = MIRA220_HBLANK_640x480_304, // TODO
		.code = MEDIA_BUS_FMT_SGRBG12_1X12,
	},
	
	{
		.width = 400,
		.height = 400,
		.crop = {
			.left = 400,
			.top = 400,
			.width = 400,
			.height = 400
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(full_400_400_250fps_12b_2lanes_reg),
			.regs = full_400_400_250fps_12b_2lanes_reg,
		},
		// vblank is ceil(MIRA220_GLOB_NUM_CLK_CYCLES / ROW_LENGTH)  + 11
		// ROW_LENGTH is configured by register 0x102B, 0x102C.
		.row_length = 304,
		.pixel_rate = MIRA220_PIXEL_RATE,
		.min_vblank = 20, // ceil(1928 / 300) + 11
		.max_vblank = 50000, // ceil(1928 / 300) + 11
		.hblank = MIRA220_HBLANK_400x400_304, // TODO
		.code = MEDIA_BUS_FMT_SGRBG12_1X12,
	}

};

struct mira220 {
	struct v4l2_subdev sd;
	struct media_pad pad[NUM_PADS];

	struct v4l2_mbus_framefmt fmt;

	struct clk *xclk; /* system clock to MIRA220 */
	u32 xclk_freq;

	//struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[MIRA220_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *gain;
	// custom v4l2 control
	struct v4l2_ctrl *mira220_reg_w;
	struct v4l2_ctrl *mira220_reg_r;
	u16 mira220_reg_w_cached_addr;
	u8 mira220_reg_w_cached_flag;


	/* Current mode */
	const struct mira220_mode *mode;
	/* Whether to skip base register sequence upload */
	u32 skip_reg_upload;
	/* Whether to reset sensor when stream on/off */
	u32 skip_reset;
	/* Whether regulator and clk are powered on */
	u32 powered;
	/* A flag to temporarily force power off */
	u32 force_power_off;
	/* A flag to force write_start/stop_streaming_regs even if (skip_reg_upload==1) */
	u8 force_stream_ctrl;
	/* Illumination trigger width/length. Use [15:0] for 16-bit register, use bit [16] for sign. */
	u32 illum_width;
	/* Illumination trigger delay. Use [15:0] for 16-bit register. */
	u32 illum_delay;

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;

	/* pmic, uC, LED */
	struct i2c_client *pmic_client;
	struct i2c_client *uc_client;
	struct i2c_client *led_client;
	/* User specified I2C device address */
	u32 tbd_client_i2c_addr;

};

static inline struct mira220 *to_mira220(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct mira220, sd);
}

static int mira220_read(struct mira220 *mira220, u16 reg, u8 *val)
{
	int ret;
	unsigned char data_w[2] = { reg >> 8, reg & 0xff };
	struct i2c_client *client = v4l2_get_subdevdata(&mira220->sd);

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

static int mira220_write(struct mira220 *mira220, u16 reg, u8 val)
{
	int ret;
	unsigned char data[3] = { reg >> 8, reg & 0xff, val};
	struct i2c_client *client = v4l2_get_subdevdata(&mira220->sd);

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

	return ret;
}

static int mira220_write16(struct mira220 *mira220, u16 reg, u16 val)
{
       int ret;
       unsigned char data[4] = { reg >> 8, reg & 0xff, val & 0xff, val >> 8};
       struct i2c_client *client = v4l2_get_subdevdata(&mira220->sd);

       ret = i2c_master_send(client, data, 4);
       /*
        * Writing the wrong number of bytes also needs to be flagged as an
        * error. Success needs to produce a 0 return code.
        */
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

/* Write a list of registers */
static int mira220_write_regs(struct mira220 *mira220,
			     const struct mira220_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira220->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = mira220_write(mira220, regs[i].address, regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    regs[i].address, ret);

			return ret;
		} else {
			// Debug code below
			//u8 val;
			//ret = mira220_read(mira220, regs[i].address, &val);
			//printk(KERN_INFO "[MIRA220]: Read reg 0x%4.4x, val = 0x%x.\n",
			//		regs[i].address, val);
		}
	}

	return 0;
}

static int mira220pmic_write(struct i2c_client *client, u8 reg, u8 val)
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

static int mira220pmic_read(struct i2c_client *client, u8 reg, u8 *val)
{
	struct i2c_msg msgs[2];
	u8 addr_buf[1] = { reg & 0xff };
	u8 data_buf[1] = { 0 };
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
static int mira220_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira220 *mira220 = to_mira220(sd);
	int ret = -EINVAL;

	printk(KERN_INFO "[MIRA220]: Entering power on function.\n");

	/* The mira220_power_on() function is called at three places:
	 * (1) by mira220_probe when driver is loaded
	 * (2) by POWER_ON command when manually issued by user
	 * (3) by mira050_start_streaming when user starts capturing
	 * Reset must happen at (1).
	 * Reset must not happen at (3) if user skip_reg_upload,
	 * because that invalidas user register upload prior to (3).
	 * Therefore, avoid reset if (skip_reg_upload == 1),
	 * or if (skip_reset == 1).
	 */
	if (mira220->skip_reset == 0 && mira220->skip_reg_upload == 0) {
		/* Pull reset to low if it is high */
		if (mira220->powered == 1) {
			ret = regulator_bulk_disable(MIRA220_NUM_SUPPLIES, mira220->supplies);
			if (ret) {
				dev_err(&client->dev, "%s: failed to disable regulators\n",
					__func__);
				return ret;
			}
			clk_disable_unprepare(mira220->xclk);
			usleep_range(MIRA220_XCLR_MIN_DELAY_US,
				     MIRA220_XCLR_MIN_DELAY_US + MIRA220_XCLR_DELAY_RANGE_US);
			mira220->powered = 0;
		} else {
			printk(KERN_INFO "[MIRA220]: Skip disabling regulator and clk due to mira220->powered == %d.\n", mira220->powered);
		}
	} else {
		printk(KERN_INFO "[MIRA220]: Skip pulling reset to low due to mira220->skip_reset=%u.\n", mira220->skip_reset);
	}

	/* Alway enable regulator even if (skip_reset == 1) */
	if (mira220->powered == 0) {
		ret = regulator_bulk_enable(MIRA220_NUM_SUPPLIES, mira220->supplies);
		if (ret) {
			dev_err(&client->dev, "%s: failed to enable regulators\n",
				__func__);
			return ret;
		}
		ret = clk_prepare_enable(mira220->xclk);
		if (ret) {
			dev_err(&client->dev, "%s: failed to enable clock\n",
				__func__);
			goto reg_off;
		}
		// gpiod_set_value_cansleep(mira220->reset_gpio, 1);
		usleep_range(MIRA220_XCLR_MIN_DELAY_US,
			     MIRA220_XCLR_MIN_DELAY_US + MIRA220_XCLR_DELAY_RANGE_US);
		mira220->powered = 1;
	} else {
		printk(KERN_INFO "[MIRA220]: Skip regulator and clk enable, because mira220->powered == %d.\n", mira220->powered);
	}

	return 0;

reg_off:
	ret = regulator_bulk_disable(MIRA220_NUM_SUPPLIES, mira220->supplies);
	mira220->powered = 0;
	return ret;
}

static int mira220_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira220 *mira220 = to_mira220(sd);
	(void)mira220;
	printk(KERN_INFO "[MIRA220]: Entering power off function.\n");

	/* Keep reset pin high, due to mira220 consums max power when reset pin is low */
	if (mira220->force_power_off == 1) {
		if (mira220->powered == 1) {
			regulator_bulk_disable(MIRA220_NUM_SUPPLIES, mira220->supplies);
			clk_disable_unprepare(mira220->xclk);
			mira220->powered = 0;
		} else {
			printk(KERN_INFO "[MIRA220]: Skip disabling regulator and clk due to mira220->powered == %d.\n", mira220->powered);
		}
	} else {
		printk(KERN_INFO "[MIRA220]: Skip disabling regulator and clk due to mira220->force_power_off=%u.\n", mira220->force_power_off);
	}

	return 0;
}

static int mira220_write_illum_trig_regs(struct mira220* mira220, u8 enable) {
	struct i2c_client* const client = v4l2_get_subdevdata(&mira220->sd);
	int ret = 0;
	u16 illum_width_reg;
	u16 illum_delay_reg;
	u8 illum_delay_sign;

	// Enable or disable illumination trigger
	printk(KERN_INFO "[MIRA220]: Writing EN_TRIG_ILLUM to %d.\n", enable);
	ret = mira220_write(mira220, MIRA220_EN_TRIG_ILLUM_REG, enable);
	if (ret) {
		dev_err(&client->dev, "Error setting EN_TRIG_ILLUM to %d.", enable);
		return ret;
	}
	
	// Set illumination width. Write 16 bits [15:0].
	illum_width_reg = (u16)(mira220->illum_width & 0x0000FFFF);
	printk(KERN_INFO "[MIRA220]: Writing ILLUM_WIDTH to %u.\n", illum_width_reg);
	ret = mira220_write16(mira220, MIRA220_ILLUM_WIDTH_REG, illum_width_reg);
	if (ret) {
		dev_err(&client->dev, "Error setting ILLUM_WIDTH to %u.", illum_width_reg);
		return ret;
	}

	// Set illumination delay. Write 16 bits [15:0] as absolute delay, and bit [16] as sign.
	illum_delay_reg = (u16)(mira220->illum_delay & 0x0000FFFF);
	printk(KERN_INFO "[MIRA220]: Writing ILLUM_DELAY to %u.\n", illum_delay_reg);
	ret = mira220_write16(mira220, MIRA220_ILLUM_DELAY_REG, illum_delay_reg);
	if (ret) {
		dev_err(&client->dev, "Error setting ILLUM_DELAY to %u.", illum_delay_reg);
		return ret;
	}
	// Set illumination delay sign. Extract bit [16] as sign.
	illum_delay_sign = (u8)((mira220->illum_delay >> 16) & 0x1);
	printk(KERN_INFO "[MIRA220]: Writing ILLUM_DELAY_SIGN to %u.\n", illum_delay_sign);
	ret = mira220_write(mira220, MIRA220_ILLUM_DELAY_SIGN_REG, illum_delay_sign);
	if (ret) {
		dev_err(&client->dev, "Error setting ILLUM_DELAY_SIGN to %u.", illum_delay_sign);
		return ret;
	}

	return ret;
}


static int mira220_write_start_streaming_regs(struct mira220* mira220) {
	struct i2c_client* const client = v4l2_get_subdevdata(&mira220->sd);
	int ret = 0;

	// Setting master control
	ret = mira220_write(mira220, MIRA220_IMAGER_STATE_REG,
				MIRA220_IMAGER_STATE_MASTER_CONTROL);
	if (ret) {
		dev_err(&client->dev, "Error setting master control");
		return ret;
	}

	// Enable continuous streaming
	ret = mira220_write(mira220, MIRA220_IMAGER_RUN_CONT_REG, 
				MIRA220_IMAGER_RUN_CONT_ENABLE);
	if (ret) {
		dev_err(&client->dev, "Error enabling continuous streaming");
		return ret;
	}

	ret = mira220_write(mira220, MIRA220_IMAGER_RUN_REG,
				MIRA220_IMAGER_RUN_START);
	if (ret) {
		dev_err(&client->dev, "Error setting internal trigger");
		return ret;
	}

	return ret;
}

static int mira220_write_stop_streaming_regs(struct mira220* mira220) {
	struct i2c_client* const client = v4l2_get_subdevdata(&mira220->sd);
	int ret = 0;
	u32 frame_time;
	int try_cnt;

	for (try_cnt = 0; try_cnt < 5; try_cnt++) {
		ret = mira220_write(mira220, MIRA220_IMAGER_STATE_REG,
					MIRA220_IMAGER_STATE_STOP_AT_ROW);
		if (ret) {
			dev_err(&client->dev, "Error setting stop-at-row imager state at try %d", try_cnt);
			usleep_range(1000, 1100);
		} else {
			break;
		}
	}
	if (ret) {
		dev_err(&client->dev, "Error setting stop-at-row imager state after multiple attempts. Exiting.");
		return ret;
	}

	ret = mira220_write(mira220, MIRA220_IMAGER_RUN_REG,
				MIRA220_IMAGER_RUN_STOP);
	if (ret) {
		dev_err(&client->dev, "Error setting run reg to stop");
		return ret;
	}

        /*
         * Wait for one frame to make sure sensor is set to
         * software standby in V-blank
         *
         * frame_time = frame length rows * Tline
         * Tline = line length / pixel clock (in MHz)
         */
        frame_time = MIRA220_DEFAULT_FRAME_LENGTH *
            MIRA220_DEFAULT_LINE_LENGTH / MIRA220_DEFAULT_PIXEL_CLOCK;

        usleep_range(frame_time, frame_time + 1000); //TODO, set to 1 frametime

	return ret;
}


static int mira220_v4l2_reg_w(struct mira220 *mira220, u32 value) {
	struct i2c_client* const client = v4l2_get_subdevdata(&mira220->sd);
	u32 ret = 0;
	u32 tmp_flag;

	u16 reg_addr = (value >> 8) & 0xFFFF;
	u8 reg_val = value & 0xFF;
	u8 reg_flag = (value >> 24) & 0xFF;

	// printk(KERN_INFO "[MIRA220]: %s reg_flag: 0x%02X; reg_addr: 0x%04X; reg_val: 0x%02X.\n",
	// 		__func__, reg_flag, reg_addr, reg_val);

	if (reg_flag & AMS_CAMERA_CID_MIRA220_REG_FLAG_CMD_SEL) {
		if (reg_flag == AMS_CAMERA_CID_MIRA220_REG_FLAG_SLEEP_US) {
			// If it is for sleep, combine all 24 bits of reg_addr and reg_val as sleep us.
			u32 sleep_us_val = value & 0x00FFFFFF;
			// Sleep range needs an interval, default to 1/8 of the sleep value.
			u32 sleep_us_interval = sleep_us_val >> 3;
			printk(KERN_INFO "[MIRA220]: %s sleep_us: %u.\n", __func__, sleep_us_val);
			usleep_range(sleep_us_val, sleep_us_val + sleep_us_interval);
		} else if (reg_flag == AMS_CAMERA_CID_MIRA220_REG_FLAG_RESET_ON) {
			printk(KERN_INFO "[MIRA220]: %s Enable reset at stream on/off.\n", __func__);
			mira220->skip_reset = 0;
		} else if (reg_flag == AMS_CAMERA_CID_MIRA220_REG_FLAG_RESET_OFF) {
			printk(KERN_INFO "[MIRA220]: %s Disable reset at stream on/off.\n", __func__);
			mira220->skip_reset = 1;
		} else if (reg_flag == AMS_CAMERA_CID_MIRA220_REG_FLAG_REG_UP_ON) {
			printk(KERN_INFO "[MIRA220]: %s Enable base register sequence upload.\n", __func__);
			mira220->skip_reg_upload = 0;
		} else if (reg_flag == AMS_CAMERA_CID_MIRA220_REG_FLAG_REG_UP_OFF) {
			printk(KERN_INFO "[MIRA220]: %s Disable base register sequence upload.\n", __func__);
			mira220->skip_reg_upload = 1;
		} else if (reg_flag == AMS_CAMERA_CID_MIRA220_REG_FLAG_POWER_ON) {
			printk(KERN_INFO "[MIRA220]: %s Call power on function mira220_power_on().\n", __func__);
			/* Temporarily disable skip_reset if manually doing power on/off */
			tmp_flag = mira220->skip_reset;
			mira220->skip_reset = 0;
			mira220_power_on(&client->dev);
			mira220->skip_reset = tmp_flag;
		} else if (reg_flag == AMS_CAMERA_CID_MIRA220_REG_FLAG_POWER_OFF) {
			printk(KERN_INFO "[MIRA220]: %s Call power off function mira220_power_off().\n", __func__);
			/* Temporarily disable skip_reset if manually doing power on/off */
			mira220->force_power_off = 1;
			mira220_power_off(&client->dev);
			mira220->force_power_off = 0;
		} else if (reg_flag == AMS_CAMERA_CID_MIRA220_REG_FLAG_ILLUM_TRIG_ON) {
			printk(KERN_INFO "[MIRA220]: %s Enable illumination trigger.\n", __func__);
			mira220_write_illum_trig_regs(mira220, 1);
		} else if (reg_flag == AMS_CAMERA_CID_MIRA220_REG_FLAG_ILLUM_TRIG_OFF) {
			printk(KERN_INFO "[MIRA220]: %s Disable illumination trigger.\n", __func__);
			mira220_write_illum_trig_regs(mira220, 0);
		} else if (reg_flag == AMS_CAMERA_CID_MIRA220_REG_FLAG_ILLUM_WIDTH) {
			// Combine 16 bits, [15:0], of reg_addr and reg_val as ILLUM_WIDTH.
			u32 illum_width = value & 0x0000FFFF;
			printk(KERN_INFO "[MIRA220]: %s Set ILLUM_WIDTH to 0x%X.\n", __func__, illum_width);
			mira220->illum_width = illum_width;
		} else if (reg_flag == AMS_CAMERA_CID_MIRA220_REG_FLAG_ILLUM_DELAY) {
			// Combine 17 bits, [16:0], of reg_addr and reg_val as ILLUM_DELAY. Bit [16] is sign.
			u32 illum_delay = value & 0x0001FFFF;
			printk(KERN_INFO "[MIRA220]: %s Set ILLUM_DELAY with sign bit to 0x%X.\n", __func__, illum_delay);
			mira220->illum_delay = illum_delay;
		} else if (reg_flag == AMS_CAMERA_CID_MIRA220_REG_FLAG_STREAM_CTRL_ON) {
			printk(KERN_INFO "[MIRA220]: %s Force stream control even if (skip_reg_upload == 1).\n", __func__);
			mira220->force_stream_ctrl = 1;
		} else if (reg_flag == AMS_CAMERA_CID_MIRA220_REG_FLAG_STREAM_CTRL_OFF) {
			printk(KERN_INFO "[MIRA220]: %s Disable stream control if (skip_reg_upload == 1).\n", __func__);
			mira220->force_stream_ctrl = 0;
		} else {
			printk(KERN_INFO "[MIRA220]: %s unknown command from flag %u, ignored.\n", __func__, reg_flag);
		}
	} else if (reg_flag & AMS_CAMERA_CID_MIRA220_REG_FLAG_FOR_READ) {
		// If it is for read, skip reagister write, cache addr and flag for read.
		mira220->mira220_reg_w_cached_addr = reg_addr;
		mira220->mira220_reg_w_cached_flag = reg_flag;
	} else {
		// If it is for write, select which I2C device by the flag "I2C_SEL".
		if ((reg_flag & AMS_CAMERA_CID_MIRA220_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_MIRA220_REG_FLAG_I2C_MIRA) {
			// Writing the actual Mira220 register
			// printk(KERN_INFO "[MIRA220]: %s write reg_addr: 0x%04X; reg_val: 0x%02X.\n", __func__, reg_addr, reg_val);
			ret = mira220_write(mira220, reg_addr, reg_val);
			if (ret) {
				dev_err_ratelimited(&client->dev, "Error AMS_CAMERA_CID_MIRA_REG_W reg_addr %X.\n", reg_addr);
				return -EINVAL;
			}
		} else if ((reg_flag & AMS_CAMERA_CID_MIRA220_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_MIRA220_REG_FLAG_I2C_SET_TBD) {
			/* User tries to set TBD I2C address, store reg_val to mira220->tbd_client_i2c_addr. Skip write. */
			printk(KERN_INFO "[MIRA220]: mira220->tbd_client_i2c_addr = 0x%X.\n", reg_val);
			mira220->tbd_client_i2c_addr = reg_val;
		} else if ((reg_flag & AMS_CAMERA_CID_MIRA220_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_MIRA220_REG_FLAG_I2C_TBD) {
			if (mira220->tbd_client_i2c_addr == MIRA220PMIC_I2C_ADDR) {
				// Write PMIC. Use pre-allocated mira220->pmic_client.
				printk(KERN_INFO "[MIRA220]: write pmic_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
				ret = mira220pmic_write(mira220->pmic_client, (u8)(reg_addr & 0xFF), reg_val);
			} else if (mira220->tbd_client_i2c_addr == MIRA220UC_I2C_ADDR) {
				// Write micro-controller. Use pre-allocated mira220->uc_client.
				printk(KERN_INFO "[MIRA220]: write uc_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
				ret = mira220pmic_write(mira220->uc_client, (u8)(reg_addr & 0xFF), reg_val);
			} else if (mira220->tbd_client_i2c_addr == MIRA220LED_I2C_ADDR) {
				// Write LED driver. Use pre-allocated mira220->led_client.
				printk(KERN_INFO "[MIRA220]: write led_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
				ret = mira220pmic_write(mira220->led_client, (u8)(reg_addr & 0xFF), reg_val);
			} else {
				/* Write other TBD I2C address.
				 * The TBD I2C address is set via AMS_CAMERA_CID_MIRA220_REG_FLAG_I2C_SET_TBD.
				 * The TBD I2C address is stored in mira220->tbd_client_i2c_addr.
				 * A temporary I2C client, tmp_client, is created and then destroyed (unregistered).
				 */
				struct i2c_client *tmp_client;
				tmp_client = i2c_new_dummy_device(client->adapter, mira220->tbd_client_i2c_addr);
				if (IS_ERR(tmp_client))
					return PTR_ERR(tmp_client);
				printk(KERN_INFO "[MIRA220]: write tbd_client, i2c_addr %u, reg_addr 0x%X, reg_val 0x%X.\n",
						mira220->tbd_client_i2c_addr, (u8)(reg_addr & 0xFF), reg_val);
				ret = mira220pmic_write(tmp_client, (u8)(reg_addr & 0xFF), reg_val);
				i2c_unregister_device(tmp_client);
			}
		}
	}

	return 0;
}

static int mira220_v4l2_reg_r(struct mira220 *mira220, u32 *value) {
	struct i2c_client* const client = v4l2_get_subdevdata(&mira220->sd);
	u32 ret = 0;

	u16 reg_addr = mira220->mira220_reg_w_cached_addr;
	u8 reg_flag = mira220->mira220_reg_w_cached_flag;
	u8 reg_val = 0;

	*value = 0;

	if ((reg_flag & AMS_CAMERA_CID_MIRA220_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_MIRA220_REG_FLAG_I2C_MIRA) {
		ret = mira220_read(mira220, reg_addr, &reg_val);
		if (ret) {
			dev_err_ratelimited(&client->dev, "Error AMS_CAMERA_CID_MIRA_REG_R reg_addr %X.\n", reg_addr);
			return -EINVAL;
		}
	} else if ((reg_flag & AMS_CAMERA_CID_MIRA220_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_MIRA220_REG_FLAG_I2C_TBD) {
		if (mira220->tbd_client_i2c_addr == MIRA220PMIC_I2C_ADDR) {
			// Read PMIC. Use pre-allocated mira220->pmic_client.
			ret = mira220pmic_read(mira220->pmic_client, (u8)(reg_addr & 0xFF), &reg_val);
			printk(KERN_INFO "[MIRA220]: read pmic_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
		} else if (mira220->tbd_client_i2c_addr == MIRA220UC_I2C_ADDR) {
			// Read micro-controller. Use pre-allocated mira220->uc_client.
			ret = mira220pmic_read(mira220->uc_client, (u8)(reg_addr & 0xFF), &reg_val);
			printk(KERN_INFO "[MIRA220]: read uc_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
		} else if (mira220->tbd_client_i2c_addr == MIRA220LED_I2C_ADDR) {
			// Read LED driver. Use pre-allocated mira220->led_client.
			ret = mira220pmic_read(mira220->led_client, (u8)(reg_addr & 0xFF), &reg_val);
			printk(KERN_INFO "[MIRA220]: read led_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
		} else {
			/* Read other TBD I2C address.
			 * The TBD I2C address is set via AMS_CAMERA_CID_MIRA220_REG_FLAG_I2C_SET_TBD.
			 * The TBD I2C address is stored in mira220->tbd_client_i2c_addr.
			 * A temporary I2C client, tmp_client, is created and then destroyed (unregistered).
			 */
			struct i2c_client *tmp_client;
			tmp_client = i2c_new_dummy_device(client->adapter, mira220->tbd_client_i2c_addr);
			if (IS_ERR(tmp_client))
				return PTR_ERR(tmp_client);
			ret = mira220pmic_read(tmp_client, (u8)(reg_addr & 0xFF), &reg_val);
			printk(KERN_INFO "[MIRA220]: read tbd_client, i2c_addr %u, reg_addr 0x%X, reg_val 0x%X.\n",
					mira220->tbd_client_i2c_addr, (u8)(reg_addr & 0xFF), reg_val);
			i2c_unregister_device(tmp_client);
		}
	}

	// Return 32-bit value that includes flags, addr, and register value
	*value = ((u32)reg_flag << 24) | ((u32)reg_addr << 8) | (u32)reg_val;

	// printk(KERN_INFO "[MIRA220]: mira220_v4l2_reg_r() reg_flag: 0x%02X; reg_addr: 0x%04X, reg_val: 0x%02X.\n",
	// 		reg_flag, reg_addr, reg_val);

	return 0;
}

// Returns the maximum exposure time in row_length (reg value).
// Calculation is baded on Mira220 datasheet Section 9.2.
static u32 mira220_calculate_max_exposure_time(u32 vsize,
					       u32 vblank, u32 row_length) {
	return (vsize + vblank) - (int)(MIRA220_GLOB_NUM_CLK_CYCLES / row_length);
}

static int mira220_write_analog_gain_reg(struct mira220 *mira220, u8 gain) {
	struct i2c_client* const client = v4l2_get_subdevdata(&mira220->sd);
	u8 reg_value;
	u32 ret;

	if ((gain < MIRA220_ANALOG_GAIN_MIN) || (gain > MIRA220_ANALOG_GAIN_MAX)) {
		return -EINVAL;
	}

	reg_value = (u8)(8 / gain);

	ret = mira220_write(mira220, MIRA220_ANALOG_GAIN_REG, reg_value);

	if (ret) {
		dev_err_ratelimited(&client->dev, "Error setting analog gain register to %d",
				reg_value);
	}

	return ret;
}

static int mira220_write_exposure_reg(struct mira220 *mira220, u32 exposure) {
	struct i2c_client* const client = v4l2_get_subdevdata(&mira220->sd);
	const u32 max_exposure = mira220_calculate_max_exposure_time(
		mira220->mode->height, mira220->vblank->val, mira220->mode->row_length);
	u32 ret = 0;
	u32 capped_exposure = exposure;

	if (exposure > max_exposure) {
		capped_exposure = max_exposure;
	}
	
	printk(KERN_INFO "[MIRA220]: exposure fun width %d, hblank %d, vblank %d, row len %d, ctrl->val %d capped to %d.\n",
				mira220->mode->width, mira220->hblank->val, mira220->vblank->val, mira220->mode->row_length, exposure, capped_exposure);
	ret = mira220_write16(mira220, MIRA220_EXP_TIME_LO_REG, capped_exposure);
	if (ret) {
		dev_err_ratelimited(&client->dev, "Error setting exposure time to %d", capped_exposure);
		return -EINVAL;
	}

	return 0;
}

// Gets the format code if supported. Otherwise returns the default format code `codes[0]`
static u32 mira220_validate_format_code_or_default(struct mira220 *mira220, u32 code)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira220->sd);
	unsigned int i;

	lockdep_assert_held(&mira220->mutex);

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

static void mira220_set_default_format(struct mira220 *mira220)
{
	struct v4l2_mbus_framefmt *fmt;

	fmt = &mira220->fmt;
	fmt->code = supported_modes[0].code;
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

static int mira220_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct mira220 *mira220 = to_mira220(sd);
	struct v4l2_mbus_framefmt *try_fmt_img =
		v4l2_subdev_get_try_format(sd, fh->state, IMAGE_PAD);
	struct v4l2_mbus_framefmt *try_fmt_meta =
		v4l2_subdev_get_try_format(sd, fh->state, METADATA_PAD);
	struct v4l2_rect *try_crop;

	mutex_lock(&mira220->mutex);

	/* Initialize try_fmt for the image pad */
	try_fmt_img->width = supported_modes[0].width;
	try_fmt_img->height = supported_modes[0].height;
	try_fmt_img->code = mira220_validate_format_code_or_default(mira220,
						   supported_modes[0].code);
	try_fmt_img->field = V4L2_FIELD_NONE;

	/* TODO(jalv): Initialize try_fmt for the embedded metadata pad */
	try_fmt_meta->width = MIRA220_EMBEDDED_LINE_WIDTH;
	try_fmt_meta->height = MIRA220_NUM_EMBEDDED_LINES;
	try_fmt_meta->code = MEDIA_BUS_FMT_SENSOR_DATA;
	try_fmt_meta->field = V4L2_FIELD_NONE;



	/* Initialize try_crop rectangle. */
	try_crop = v4l2_subdev_get_try_crop(sd, fh->state, 0);
	try_crop->top = supported_modes[0].crop.top;
	try_crop->left = supported_modes[0].crop.left;
	try_crop->width = supported_modes[0].crop.width;
	try_crop->height = supported_modes[0].crop.height;

	mutex_unlock(&mira220->mutex);

	return 0;
}

static int mira220_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mira220 *mira220 =
		container_of(ctrl->handler, struct mira220, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&mira220->sd);
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		int exposure_max, exposure_def;

		/* Update max exposure while meeting expected vblanking */
		exposure_max = mira220_calculate_max_exposure_time(
				                mira220->mode->height, ctrl->val, mira220->mode->row_length);
		exposure_def = (exposure_max < MIRA220_DEFAULT_EXPOSURE) ?
			exposure_max : MIRA220_DEFAULT_EXPOSURE;
		__v4l2_ctrl_modify_range(mira220->exposure,
					 mira220->exposure->minimum,
					 exposure_max, mira220->exposure->step,
					 exposure_def);
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

	if (mira220->skip_reg_upload == 0) {
		switch (ctrl->id) {
		case V4L2_CID_ANALOGUE_GAIN:
			// ret = mira220_write_analog_gain_reg(mira220, ctrl->val);
			break;
		case V4L2_CID_EXPOSURE:
			ret = mira220_write_exposure_reg(mira220, ctrl->val);
			break;
		case V4L2_CID_TEST_PATTERN:
			ret = mira220_write(mira220, MIRA220_REG_TEST_PATTERN,
					       mira220_test_pattern_val[ctrl->val]);
			break;
		case V4L2_CID_HFLIP:
			ret = mira220_write(mira220, MIRA220_HFLIP_REG,
						ctrl->val);
			break;
		case V4L2_CID_VFLIP:
			ret = mira220_write(mira220, MIRA220_VFLIP_REG,
						ctrl->val);
			break;
		case V4L2_CID_VBLANK:
			ret = mira220_write16(mira220, MIRA220_VBLANK_LO_REG,
						ctrl->val);
			printk(KERN_INFO "[MIRA220]: width %d, hblank %d, vblank %d, height %d, ctrl->val %d.\n",
				   mira220->mode->width, mira220->mode->hblank, mira220->mode->min_vblank, mira220->mode->height, ctrl->val);
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
	}

	pm_runtime_put(&client->dev);

	// TODO: FIXIT
	return ret;
}

static int mira220_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mira220 *mira220 =
		container_of(ctrl->handler, struct mira220, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&mira220->sd);
	int ret = 0;

	// printk(KERN_INFO "[MIRA220]: mira220_s_ctrl() id: %X value: %X.\n", ctrl->id, ctrl->val);

	/* Previously, register writes when powered off will be buffered.
	 * The buffer will be written to sensor when start_streaming.
	 * Now, register writes happens immediately, even powered off.
	 * Register writes when powered off will fail.
	 * Users need to make sure first power on then write register.
	 */

	switch (ctrl->id) {
	case AMS_CAMERA_CID_MIRA_REG_W:
		ret = mira220_v4l2_reg_w(mira220, ctrl->val);
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

static int mira220_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mira220 *mira220 =
		container_of(ctrl->handler, struct mira220, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&mira220->sd);
	int ret = 0;

	// printk(KERN_INFO "[MIRA220]: mira220_g_ctrl() id: %X.\n", ctrl->id);

	/*
	 * Ideally, V4L2 register read should happen only when powered on.
	 * However, perhaps there are use cases that,
	 * reading other I2C addr is desired when mira sensor is powered off.
	 * Therefore, the check of "powered" flag is disabled for now.
	 */

	switch (ctrl->id) {
	case AMS_CAMERA_CID_MIRA_REG_R:
		ret = mira220_v4l2_reg_r(mira220, (u32 *)&ctrl->cur.val);
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


static const struct v4l2_ctrl_ops mira220_ctrl_ops = {
	.s_ctrl = mira220_set_ctrl,
};

static const struct v4l2_ctrl_ops mira220_custom_ctrl_ops = {
	.g_volatile_ctrl = mira220_g_ctrl,
	.s_ctrl = mira220_s_ctrl,
};


/* list of custom v4l2 ctls */
static struct v4l2_ctrl_config custom_ctrl_config_list[] = {
	/* Do not change the name field for the controls! */
	{
		.ops = &mira220_custom_ctrl_ops,
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
		.ops = &mira220_custom_ctrl_ops,
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
static int mira220_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct mira220 *mira220 = to_mira220(sd);

	if (code->pad >= NUM_PADS)
		return -EINVAL;

	if (code->pad == IMAGE_PAD) {
		if (code->index >= ARRAY_SIZE(codes))
			return -EINVAL;

		code->code = mira220_validate_format_code_or_default(mira220,
						    codes[code->index]);
	} else {
		if (code->index > 0)
			return -EINVAL;

		code->code = MEDIA_BUS_FMT_SENSOR_DATA;
	}

	return 0;
}

static int mira220_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct mira220 *mira220 = to_mira220(sd);

	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	if (fse->pad == IMAGE_PAD) {
		if (fse->index >= ARRAY_SIZE(supported_modes))
			return -EINVAL;

		if (fse->code != mira220_validate_format_code_or_default(mira220, fse->code))
			return -EINVAL;

		fse->min_width = supported_modes[fse->index].width;
		fse->max_width = fse->min_width;
		fse->min_height = supported_modes[fse->index].height;
		fse->max_height = fse->min_height;
	} else {
		if (fse->code != MEDIA_BUS_FMT_SENSOR_DATA || fse->index > 0)
			return -EINVAL;

		fse->min_width = MIRA220_EMBEDDED_LINE_WIDTH;
		fse->max_width = fse->min_width;
		fse->min_height = MIRA220_NUM_EMBEDDED_LINES;
		fse->max_height = fse->min_height;
	}

	return 0;
}

static void mira220_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void mira220_update_image_pad_format(struct mira220 *mira220,
					   const struct mira220_mode *mode,
					   struct v4l2_subdev_format *fmt)
{
	if (mode != NULL) {
		printk(KERN_INFO "[MIRA220]: mira220_update_image_pad_format() width %d, height %d.\n",
				mode->width, mode->height);
	} else {
		printk(KERN_ERR "[MIRA220]: mira220_update_image_pad_format() mode is NULL.\n");
	}
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	mira220_reset_colorspace(&fmt->format);
}

static void mira220_update_metadata_pad_format(struct v4l2_subdev_format *fmt)
{
	fmt->format.width = MIRA220_EMBEDDED_LINE_WIDTH;
	fmt->format.height = MIRA220_NUM_EMBEDDED_LINES;
	fmt->format.code = MEDIA_BUS_FMT_SENSOR_DATA;
	fmt->format.field = V4L2_FIELD_NONE;

}

static int __mira220_get_pad_format(struct mira220 *mira220,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_format *fmt)
{
	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(&mira220->sd, sd_state, fmt->pad);

		try_fmt->code = fmt->pad == IMAGE_PAD ?
				mira220_validate_format_code_or_default(mira220, try_fmt->code) :
				MEDIA_BUS_FMT_SENSOR_DATA;
		fmt->format = *try_fmt;
	} else {
		if (fmt->pad == IMAGE_PAD) {
			mira220_update_image_pad_format(mira220, mira220->mode,
						       fmt);
			fmt->format.code = mira220_validate_format_code_or_default(mira220,
							      mira220->fmt.code);
		} else {
			mira220_update_metadata_pad_format(fmt);
		}
	}

	return 0;
}

static int mira220_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct mira220 *mira220 = to_mira220(sd);
	int ret;

	mutex_lock(&mira220->mutex);
	ret = __mira220_get_pad_format(mira220, sd_state, fmt);
	mutex_unlock(&mira220->mutex);

	return ret;
}

static int mira220_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct mira220 *mira220 = to_mira220(sd);
	const struct mira220_mode *mode;
	struct v4l2_mbus_framefmt *framefmt;
	u32 max_exposure = 0, default_exp = 0;

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&mira220->mutex);

	if (fmt->pad == IMAGE_PAD) {
		/* Validate format or use default */
		fmt->format.code = mira220_validate_format_code_or_default(mira220,
									  fmt->format.code);

		mode = v4l2_find_nearest_size(supported_modes,
					      ARRAY_SIZE(supported_modes),
					      width, height,
					      fmt->format.width,
					      fmt->format.height);
		mira220_update_image_pad_format(mira220, mode, fmt);
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			printk(KERN_INFO "[MIRA220]: mira220_set_pad_format() use try_format.\n");
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else if (mira220->mode != mode ||
			mira220->fmt.code != fmt->format.code) {

			printk(KERN_INFO "[MIRA220]: mira220_set_pad_format() use new mode.\n");
			printk(KERN_INFO "[MIRA220]: mira220->mode %p mode %p.\n", (void *)mira220->mode, (void *)mode);
			printk(KERN_INFO "[MIRA220]: mira220->fmt.code 0x%x fmt->format.code 0x%x.\n", mira220->fmt.code, fmt->format.code);

			mira220->fmt = fmt->format;
			mira220->mode = mode;

			// Update controls based on new mode (range and current value).
			max_exposure = mira220_calculate_max_exposure_time(
									   mira220->mode->height,
									   mira220->mode->min_vblank,
									   mira220->mode->row_length);
			default_exp = MIRA220_DEFAULT_EXPOSURE > max_exposure ? max_exposure : MIRA220_DEFAULT_EXPOSURE;
			printk(KERN_INFO "[MIRA220]: mira220_set_pad_format() min_exp %d max_exp %d, default_exp %d\n",
					MIRA220_EXPOSURE_MIN, max_exposure, default_exp);
			__v4l2_ctrl_modify_range(mira220->exposure,
						     MIRA220_EXPOSURE_MIN,
						     max_exposure, 1,
						     default_exp);

			// Update pixel rate based on new mode.
			__v4l2_ctrl_modify_range(mira220->pixel_rate,
						     mira220->mode->pixel_rate,
						     mira220->mode->pixel_rate, 1,
						     mira220->mode->pixel_rate);
			printk(KERN_INFO "[MIRA220]: mira220_set_pad_format() update V4L2_CID_PIXEL_RATE to %u\n", mira220->mode->pixel_rate);

			// Update hblank based on new mode.
			__v4l2_ctrl_modify_range(mira220->hblank,
						     mira220->mode->hblank,
						     mira220->mode->hblank, 1,
						     mira220->mode->hblank);
			printk(KERN_INFO "[MIRA220]: mira220_set_pad_format() update V4L2_CID_HBLANK to %u\n", mira220->mode->hblank);

			printk(KERN_INFO "[MIRA220]: Mira220 VBLANK  = %u.\n",
				   mira220->mode->min_vblank);

			__v4l2_ctrl_modify_range(mira220->vblank,
										  mira220->mode->min_vblank,
										  mira220->mode->max_vblank,
										  1,
										  mira220->mode->min_vblank);

			// Set the current vblank value
			printk(KERN_INFO "[MIRA220]: mira220_set_pad_format() mira220->mode->min_vblank, %d\n",
					mira220->mode->min_vblank);

			__v4l2_ctrl_s_ctrl(mira220->vblank, mira220->mode->min_vblank);
		}
	} else {
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else {
			/* Only one embedded data mode is supported */
			mira220_update_metadata_pad_format(fmt);
		}
	}

	printk(KERN_INFO "[MIRA220]: mira220_set_pad_format() to unlock and return.\n");

	mutex_unlock(&mira220->mutex);

	return 0;
}

static int mira220_set_framefmt(struct mira220 *mira220)
{
	if (mira220->skip_reg_upload == 0) {
		switch (mira220->fmt.code) {
		case MEDIA_BUS_FMT_Y8_1X8:
		case MEDIA_BUS_FMT_SGRBG8_1X8:
			printk(KERN_INFO "[MIRA220]: mira220_set_framefmt() write 8 bpp regs.\n");
			mira220_write(mira220, MIRA220_BIT_DEPTH_REG, MIRA220_BIT_DEPTH_8_BIT);
			mira220_write(mira220, MIRA220_CSI_DATA_TYPE_REG,
				MIRA220_CSI_DATA_TYPE_8_BIT);
			return 0;
		case MEDIA_BUS_FMT_Y10_1X10:
		case MEDIA_BUS_FMT_SGRBG10_1X10:
			printk(KERN_INFO "[MIRA220]: mira220_set_framefmt() write 10 bpp regs.\n");
			mira220_write(mira220, MIRA220_BIT_DEPTH_REG,MIRA220_BIT_DEPTH_10_BIT);
			mira220_write(mira220, MIRA220_CSI_DATA_TYPE_REG,
				MIRA220_CSI_DATA_TYPE_10_BIT);
			return 0;
		case MEDIA_BUS_FMT_Y12_1X12:
		case MEDIA_BUS_FMT_SGRBG12_1X12:
			printk(KERN_INFO "[MIRA220]: mira220_set_framefmt() write 12 bpp regs.\n");
			mira220_write(mira220, MIRA220_BIT_DEPTH_REG, MIRA220_BIT_DEPTH_12_BIT);
			mira220_write(mira220, MIRA220_CSI_DATA_TYPE_REG,
				MIRA220_CSI_DATA_TYPE_12_BIT);
			return 0;
		default:
			printk(KERN_ERR "Unknown format requested %d", mira220->fmt.code);
		}
	}

	return -EINVAL;
}

static const struct v4l2_rect *
__mira220_get_pad_crop(struct mira220 *mira220, struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&mira220->sd, sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &mira220->mode->crop;
	}

	return NULL;
}

static int mira220_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct mira220 *mira220 = to_mira220(sd);

		mutex_lock(&mira220->mutex);
		sel->r = *__mira220_get_pad_crop(mira220, sd_state, sel->pad,
						sel->which);
		mutex_unlock(&mira220->mutex);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = MIRA220_NATIVE_WIDTH;
		sel->r.height = MIRA220_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = MIRA220_PIXEL_ARRAY_TOP;
		sel->r.left = MIRA220_PIXEL_ARRAY_LEFT;
		sel->r.width = MIRA220_PIXEL_ARRAY_WIDTH;
		sel->r.height = MIRA220_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

static int mira220_start_streaming(struct mira220 *mira220)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira220->sd);
	const struct mira220_reg_list *reg_list;
	int ret;

	printk(KERN_INFO "[MIRA220]: Entering start streaming function.\n");

	/* Follow examples of other camera driver, here use pm_runtime_resume_and_get */
	ret = pm_runtime_resume_and_get(&client->dev);

	if (ret < 0) {
		//printk(KERN_INFO "[MIRA220]: get_sync failed, but continue.\n");
		pm_runtime_put_noidle(&client->dev);
		return ret;
	}

	/* Apply default values of current mode */
	if (mira220->skip_reg_upload == 0) {
		/* Stop treaming before uploading register sequence */
		printk(KERN_INFO "[MIRA220]: Writing stop streaming regs.\n");
		ret = mira220_write_stop_streaming_regs(mira220);
		if (ret) {
			dev_err(&client->dev, "Could not write stream-on sequence");
			goto err_rpm_put;
		}

		reg_list = &mira220->mode->reg_list;
		printk(KERN_INFO "[MIRA220]: Write %d regs.\n", reg_list->num_of_regs);
		ret = mira220_write_regs(mira220, reg_list->regs, reg_list->num_of_regs);
		if (ret) {
			dev_err(&client->dev, "%s failed to set mode\n", __func__);
			goto err_rpm_put;
		}

		ret = mira220_set_framefmt(mira220);
		if (ret) {
			dev_err(&client->dev, "%s failed to set frame format: %d\n",
				__func__, ret);
			goto err_rpm_put;
		}
	} else {
		printk(KERN_INFO "[MIRA220]: Skip base register sequence upload, due to mira220->skip_reg_upload=%u.\n", mira220->skip_reg_upload);
	}


	printk(KERN_INFO "[MIRA220]: Entering v4l2 ctrl handler setup function.\n");

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(mira220->sd.ctrl_handler);
	printk(KERN_INFO "[MIRA220]: __v4l2_ctrl_handler_setup ret = %d.\n", ret);
	if (ret)
		goto err_rpm_put;


	if (mira220->skip_reg_upload == 0 ||
		(mira220->skip_reg_upload == 1 && mira220->force_stream_ctrl == 1) ) {
		printk(KERN_INFO "[MIRA220]: Writing start streaming regs.\n");
		ret = mira220_write_start_streaming_regs(mira220);
		if (ret) {
			dev_err(&client->dev, "Could not write stream-on sequence");
			goto err_rpm_put;
		}
	} else {
		printk(KERN_INFO "[MIRA220]: Skip write_start_streaming_regs due to skip_reg_upload == %d and force_stream_ctrl == %d.\n",
				mira220->skip_reg_upload, mira220->force_stream_ctrl);
	}

	/* vflip and hflip cannot change during streaming */
	printk(KERN_INFO "[MIRA220]: Entering v4l2 ctrl grab vflip grab vflip.\n");
	__v4l2_ctrl_grab(mira220->vflip, true);
	printk(KERN_INFO "[MIRA220]: Entering v4l2 ctrl grab vflip grab hflip.\n");
	__v4l2_ctrl_grab(mira220->hflip, true);

	return 0;

err_rpm_put:
	pm_runtime_put(&client->dev);
	return ret;
}

static void mira220_stop_streaming(struct mira220 *mira220)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira220->sd);
	int ret = 0;

	/* Unlock controls for vflip and hflip */
	__v4l2_ctrl_grab(mira220->vflip, false);
	__v4l2_ctrl_grab(mira220->hflip, false);

	if (mira220->skip_reset == 0) {
		if (mira220->skip_reg_upload == 0 ||
			(mira220->skip_reg_upload == 1 && mira220->force_stream_ctrl == 1) ) {
			printk(KERN_INFO "[MIRA220]: Writing stop streaming regs.\n");
			ret = mira220_write_stop_streaming_regs(mira220);
			if (ret) {
				dev_err(&client->dev, "Could not write the stream-off sequence");
			}	
		} else {
			printk(KERN_INFO "[MIRA220]: Skip write_stop_streaming_regs due to skip_reg_upload == %d and force_stream_ctrl == %d.\n",
					mira220->skip_reg_upload, mira220->force_stream_ctrl);
		}
	} else {
		printk(KERN_INFO "[MIRA220]: Skip write_stop_streaming_regs due to mira220->skip_reset == %d.\n", mira220->skip_reset);
	}

	pm_runtime_put(&client->dev);
}

static int mira220_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct mira220 *mira220 = to_mira220(sd);
	int ret = 0;

	mutex_lock(&mira220->mutex);
	if (mira220->streaming == enable) {
		mutex_unlock(&mira220->mutex);
		return 0;
	}

	printk(KERN_INFO "[MIRA220]: Entering mira220_set_stream enable: %d.\n", enable);

	if (enable) {
		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = mira220_start_streaming(mira220);
		if (ret)
			goto err_unlock;
	} else {
		mira220_stop_streaming(mira220);
	}

	mira220->streaming = enable;

	mutex_unlock(&mira220->mutex);

	printk(KERN_INFO "[MIRA220]: Returning mira220_set_stream with ret: %d.\n", ret);

	return ret;

err_unlock:
	mutex_unlock(&mira220->mutex);

	return ret;
}

static int __maybe_unused mira220_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira220 *mira220 = to_mira220(sd);

	printk(KERN_INFO "[MIRA220]: Entering suspend function.\n");

	if (mira220->streaming)
		mira220_stop_streaming(mira220);

	return 0;
}

static int __maybe_unused mira220_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira220 *mira220 = to_mira220(sd);
	int ret;

	printk(KERN_INFO "[MIRA220]: Entering resume function.\n");

	if (mira220->streaming) {
		ret = mira220_start_streaming(mira220);
		if (ret)
			goto error;
	}

	return 0;

error:
	mira220_stop_streaming(mira220);
	mira220->streaming = false;

	return ret;
}

static int mira220_get_regulators(struct mira220 *mira220)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira220->sd);
	unsigned int i;

	for (i = 0; i < MIRA220_NUM_SUPPLIES; i++)
		mira220->supplies[i].supply = mira220_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       MIRA220_NUM_SUPPLIES,
				       mira220->supplies);
}

/* OTP power on */
static int mira220_otp_power_on(struct mira220 *mira220)
{
	int ret;

	ret = mira220_write(mira220, 0x0080, 0x04);

	return 0;
}

/* OTP power off */
static int mira220_otp_power_off(struct mira220 *mira220)
{
	int ret;

	ret = mira220_write(mira220, 0x0080, 0x08);

	return 0;
}

/* OTP power on */
static int mira220_otp_read(struct mira220 *mira220, u8 addr, u8 offset, u8 *val)
{
	int ret;

	ret = mira220_write(mira220, 0x0086, addr);
	ret = mira220_write(mira220, 0x0080, 0x02);
	ret = mira220_read(mira220, 0x0082 + offset, val);
	return 0;
}


/* Verify chip ID */
static int mira220_identify_module(struct mira220 *mira220)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira220->sd);
	int ret;
	u8 val;

	mira220_otp_power_on(mira220);

	usleep_range(100, 110);

	ret = mira220_otp_read(mira220, 0x0d, 0, &val);
	dev_err(&client->dev, "Read OTP add 0x0d with val %x\n", val);

	mira220_otp_power_off(mira220);

	return 0;
}

static const struct v4l2_subdev_core_ops mira220_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops mira220_video_ops = {
	.s_stream = mira220_set_stream,
};

static const struct v4l2_subdev_pad_ops mira220_pad_ops = {
	.enum_mbus_code = mira220_enum_mbus_code,
	.get_fmt = mira220_get_pad_format,
	.set_fmt = mira220_set_pad_format,
	.get_selection = mira220_get_selection,
	.enum_frame_size = mira220_enum_frame_size,
};

static const struct v4l2_subdev_ops mira220_subdev_ops = {
	.core = &mira220_core_ops,
	.video = &mira220_video_ops,
	.pad = &mira220_pad_ops,
};

static const struct v4l2_subdev_internal_ops mira220_internal_ops = {
	.open = mira220_open,
};

/* Initialize control handlers */
static int mira220_init_controls(struct mira220 *mira220)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira220->sd);
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct v4l2_fwnode_device_properties props;
	int ret;
	struct v4l2_ctrl_config *mira220_reg_w;
	struct v4l2_ctrl_config *mira220_reg_r;

	u32 max_exposure = 0;

	ctrl_hdlr = &mira220->ctrl_handler;
	/* v4l2_ctrl_handler_init gives a hint/guess of the number of v4l2_ctrl_new */
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 16);
	if (ret)
		return ret;

	mutex_init(&mira220->mutex);
	ctrl_hdlr->lock = &mira220->mutex;

	printk(KERN_INFO "[MIRA220]: %s V4L2_CID_PIXEL_RATE %X.\n", __func__, V4L2_CID_PIXEL_RATE);

	/* By default, PIXEL_RATE is read only */
	mira220->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &mira220_ctrl_ops,
					        V4L2_CID_PIXEL_RATE,
					        mira220->mode->pixel_rate,
					        mira220->mode->pixel_rate, 1,
					        mira220->mode->pixel_rate);

	printk(KERN_INFO "[MIRA220]: %s V4L2_CID_VBLANK %X.\n", __func__, V4L2_CID_VBLANK);

	mira220->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &mira220_ctrl_ops,
					   V4L2_CID_VBLANK, mira220->mode->min_vblank,
					   mira220->mode->max_vblank, 1,
					   mira220->mode->min_vblank);

	printk(KERN_INFO "[MIRA220]: %s V4L2_CID_HBLANK %X.\n", __func__, V4L2_CID_HBLANK);

	mira220->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &mira220_ctrl_ops,
					   V4L2_CID_HBLANK, mira220->mode->hblank,
					   mira220->mode->hblank, 1,
					   mira220->mode->hblank);

	// Make the vblank control read only. This could be changed to allow changing framerate in
	// runtime, but would require adapting other settings
	// mira220->vblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	// Exposure is indicated in number of lines here
	// Max is determined by vblank + vsize and Tglob.
	max_exposure = mira220_calculate_max_exposure_time(mira220->mode->height,
	                                                   mira220->vblank->val,
													   mira220->mode->row_length);

	printk(KERN_INFO "[MIRA220]: %s V4L2_CID_EXPOSURE %X.\n", __func__, V4L2_CID_EXPOSURE);

	mira220->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &mira220_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     MIRA220_EXPOSURE_MIN, max_exposure,
					     1,
					     MIRA220_DEFAULT_EXPOSURE);

	printk(KERN_INFO "[MIRA220]: %s V4L2_CID_ANALOGUE_GAIN %X.\n", __func__, V4L2_CID_ANALOGUE_GAIN);

	mira220->gain = v4l2_ctrl_new_std(ctrl_hdlr, &mira220_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  MIRA220_ANALOG_GAIN_MIN, MIRA220_ANALOG_GAIN_MAX,
			  MIRA220_ANALOG_GAIN_STEP, MIRA220_ANALOG_GAIN_DEFAULT);

	printk(KERN_INFO "[MIRA220]: %s V4L2_CID_HFLIP %X.\n", __func__, V4L2_CID_HFLIP);

	mira220->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &mira220_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (mira220->hflip)
		mira220->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	printk(KERN_INFO "[MIRA220]: %s V4L2_CID_VFLIP %X.\n", __func__, V4L2_CID_VFLIP);

	mira220->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &mira220_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (mira220->vflip)
		mira220->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	printk(KERN_INFO "[MIRA220]: %s V4L2_CID_TEST_PATTERN %X.\n", __func__, V4L2_CID_TEST_PATTERN);
	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &mira220_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(mira220_test_pattern_menu) - 1,
				     0, 0, mira220_test_pattern_menu);
	/*
	 * Custom op
	 */
	mira220_reg_w = &custom_ctrl_config_list[0];
	printk(KERN_INFO "[MIRA220]: %s AMS_CAMERA_CID_MIRA_REG_W %X.\n", __func__, AMS_CAMERA_CID_MIRA_REG_W);
	mira220->mira220_reg_w = v4l2_ctrl_new_custom(ctrl_hdlr, mira220_reg_w, NULL);

	mira220_reg_r = &custom_ctrl_config_list[1];
	printk(KERN_INFO "[MIRA220]: %s AMS_CAMERA_CID_MIRA_REG_R %X.\n", __func__, AMS_CAMERA_CID_MIRA_REG_R);
	mira220->mira220_reg_r = v4l2_ctrl_new_custom(ctrl_hdlr, mira220_reg_r, NULL);
	if (mira220->mira220_reg_r)
		mira220->mira220_reg_r->flags |= (V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &mira220_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	mira220->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&mira220->mutex);

	return ret;
}

static void mira220_free_controls(struct mira220 *mira220)
{
	v4l2_ctrl_handler_free(mira220->sd.ctrl_handler);
	mutex_destroy(&mira220->mutex);
}

static int mira220_check_hwcfg(struct device *dev)
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
	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 2) {
		dev_err(dev, "only 2 data lanes are currently supported\n");
		goto error_out;
	}

	/* Check the link frequency set in device tree */
	if (!ep_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		goto error_out;
	}

	if (ep_cfg.nr_of_link_frequencies != 1 ||
	    ep_cfg.link_frequencies[0] != MIRA220_DEFAULT_LINK_FREQ) {
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

static int mira220pmic_init_controls(struct i2c_client *client)
{
	int ret;
	u8 val;

	ret = mira220pmic_write(client, 0x62, 0x00);
	ret = mira220pmic_write(client, 0x61, 0x00);

	ret = mira220pmic_read(client, 0x61, &val);
	dev_err(&client->dev, "Read 0x61 with val %x\n", val);


	usleep_range(100, 110);

	ret = mira220pmic_write(client, 0x05, 0x00);
	ret = mira220pmic_write(client, 0x0e, 0x00);
	ret = mira220pmic_write(client, 0x11, 0x00);
	ret = mira220pmic_write(client, 0x14, 0x00);
	ret = mira220pmic_write(client, 0x17, 0x00);
	ret = mira220pmic_write(client, 0x1a, 0x00);
	ret = mira220pmic_write(client, 0x1c, 0x00);
	ret = mira220pmic_write(client, 0x1d, 0x00);
	ret = mira220pmic_write(client, 0x1e, 0x00);
	ret = mira220pmic_write(client, 0x1f, 0x00);

	ret = mira220pmic_write(client, 0x24, 0x48);
	ret = mira220pmic_write(client, 0x20, 0x00);
	ret = mira220pmic_write(client, 0x21, 0x00);
	ret = mira220pmic_write(client, 0x1a, 0x00);
	ret = mira220pmic_write(client, 0x01, 0x00);
	ret = mira220pmic_write(client, 0x08, 0x00);
	ret = mira220pmic_write(client, 0x02, 0x00);
	ret = mira220pmic_write(client, 0x0b, 0x00);
	ret = mira220pmic_write(client, 0x14, 0x00);
	ret = mira220pmic_write(client, 0x17, 0x00);
	ret = mira220pmic_write(client, 0x1c, 0x00);
	ret = mira220pmic_write(client, 0x1d, 0x00);
	ret = mira220pmic_write(client, 0x1f, 0x00);

	usleep_range(50, 60);

	ret = mira220pmic_write(client, 0x62, 0x0d);

	usleep_range(50, 60);
	usleep_range(50000, 50000+100);

	ret = mira220pmic_write(client, 0x27, 0xff);
	ret = mira220pmic_write(client, 0x28, 0xff);
	ret = mira220pmic_write(client, 0x29, 0xff);
	ret = mira220pmic_write(client, 0x2a, 0xff);
	ret = mira220pmic_write(client, 0x2b, 0xff);

	ret = mira220pmic_write(client, 0x41, 0x04);
	usleep_range(50, 60);

	ret = mira220pmic_read(client, 0x20, &val);
	dev_err(&client->dev, "Read 0x20 with val %x\n", val);

	// PCB V2.0 or above, enable LDO9=2.50V for VDD25
	ret = mira220pmic_write(client, 0x20, 0xb2);
	// For PCB V1.0, VDD28 on 2.85V for older PCBs
	// ret = mira220pmic_write(client, 0x20, 0xb9);

	ret = mira220pmic_read(client, 0x20, &val);
	dev_err(&client->dev, "Read 0x20 with val %x\n", val);

	usleep_range(700, 710);

	ret = mira220pmic_write(client, 0x12, 0x16);
	ret = mira220pmic_write(client, 0x10, 0x16);
	ret = mira220pmic_write(client, 0x11, 0x96);
	ret = mira220pmic_write(client, 0x1e, 0x96);
	ret = mira220pmic_write(client, 0x21, 0x96);
	usleep_range(50, 60);

	ret = mira220pmic_write(client, 0x00, 0x04);
	ret = mira220pmic_write(client, 0x04, 0x34);
	ret = mira220pmic_write(client, 0x06, 0xbf);
	ret = mira220pmic_write(client, 0x05, 0xb4);
	ret = mira220pmic_write(client, 0x03, 0x00);
	ret = mira220pmic_write(client, 0x0d, 0x34);
	ret = mira220pmic_write(client, 0x0f, 0xbf);
	ret = mira220pmic_write(client, 0x0e, 0xb4);
	usleep_range(50, 60);

	ret = mira220pmic_write(client, 0x42, 0x05);
	usleep_range(50, 60);

	ret = mira220pmic_write(client, 0x45, 0x40);
	ret = mira220pmic_write(client, 0x57, 0x02);
	ret = mira220pmic_write(client, 0x5d, 0x10);
	ret = mira220pmic_write(client, 0x61, 0x10);

	return 0;
}


static int mira220_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct mira220 *mira220;
	int ret;

	printk(KERN_INFO "[MIRA220]: probing v4l2 sensor.\n");
	printk(KERN_INFO "[MIRA220]: Driver Version 0.0.\n");

	dev_err(dev, "[MIRA220] name: %s.\n", client->name);

	mira220 = devm_kzalloc(&client->dev, sizeof(*mira220), GFP_KERNEL);
	if (!mira220)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&mira220->sd, client, &mira220_subdev_ops);

	/* Check the hardware configuration in device tree */
	if (mira220_check_hwcfg(dev))
		return -EINVAL;

	/* Parse device tree to check if dtoverlay has param skip-reg-upload=1 */
        device_property_read_u32(dev, "skip-reg-upload", &mira220->skip_reg_upload);
	printk(KERN_INFO "[MIRA220]: skip-reg-upload %d.\n", mira220->skip_reg_upload);
	/* Set default TBD I2C device address to LED I2C Address*/
	mira220->tbd_client_i2c_addr = MIRA220LED_I2C_ADDR;
	printk(KERN_INFO "[MIRA220]: User defined I2C device address defaults to LED driver I2C address 0x%X.\n", mira220->tbd_client_i2c_addr);


	/* Get system clock (xclk) */
	mira220->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(mira220->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(mira220->xclk);
	}

	mira220->xclk_freq = clk_get_rate(mira220->xclk);
	if (mira220->xclk_freq != MIRA220_SUPPORTED_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			mira220->xclk_freq);
		return -EINVAL;
	}

	ret = mira220_get_regulators(mira220);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	/* Request optional enable pin */
	// mira220->reset_gpio = devm_gpiod_get_optional(dev, "reset",
	//					     GPIOD_OUT_HIGH);

	{
		printk(KERN_INFO "[MIRA220]: Init PMIC.\n");
		mira220->pmic_client = i2c_new_dummy_device(client->adapter,
				MIRA220PMIC_I2C_ADDR);
		if (IS_ERR(mira220->pmic_client))
			return PTR_ERR(mira220->pmic_client);
		mira220->uc_client = i2c_new_dummy_device(client->adapter,
				MIRA220UC_I2C_ADDR);
		if (IS_ERR(mira220->uc_client))
			return PTR_ERR(mira220->uc_client);
		mira220->led_client = i2c_new_dummy_device(client->adapter,
				MIRA220LED_I2C_ADDR);
		if (IS_ERR(mira220->led_client))
			return PTR_ERR(mira220->led_client);

		mira220pmic_init_controls(mira220->pmic_client);
	}

	dev_err(dev, "[MIRA220] Sleep for 1 second to let PMIC driver complete init.\n");
	usleep_range(1000000, 1000000+100);

	/*
	 * The sensor must be powered for mira220_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = mira220_power_on(dev);
	if (ret)
		return ret;

	printk(KERN_INFO "[MIRA220]: Entering identify function.\n");

	ret = mira220_identify_module(mira220);
	if (ret)
		goto error_power_off;

	printk(KERN_INFO "[MIRA220]: Setting support function.\n");

	/* Initialize default illumination trigger parameters */
	/* ILLUM_WIDTH (length) is in unit of rows. */
	mira220->illum_width = MIRA220_ILLUM_WIDTH_DEFAULT;
	/* ILLUM_DELAY is in unit of rows. */
	mira220->illum_delay = MIRA220_ILLUM_DELAY_DEFAULT;

	/* Set default mode to max resolution */
	mira220->mode = &supported_modes[0];

	printk(KERN_INFO "[MIRA220]: Entering init controls function.\n");

	ret = mira220_init_controls(mira220);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	mira220->sd.internal_ops = &mira220_internal_ops;
	mira220->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	mira220->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	mira220->pad[IMAGE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	mira220->pad[METADATA_PAD].flags = MEDIA_PAD_FL_SOURCE;

	printk(KERN_INFO "[MIRA220]: Entering set default format function.\n");

	/* Initialize default format */
	mira220_set_default_format(mira220);

	printk(KERN_INFO "[MIRA220]: Entering pads init function.\n");

	ret = media_entity_pads_init(&mira220->sd.entity, NUM_PADS, mira220->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	printk(KERN_INFO "[MIRA220]: Entering subdev sensor common function.\n");

	ret = v4l2_async_register_subdev_sensor(&mira220->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

error_media_entity:
	media_entity_cleanup(&mira220->sd.entity);

error_handler_free:
	mira220_free_controls(mira220);

error_power_off:
	mira220_power_off(dev);

	i2c_unregister_device(mira220->pmic_client);
	i2c_unregister_device(mira220->uc_client);
	i2c_unregister_device(mira220->led_client);

	return ret;
}

static void mira220_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira220 *mira220 = to_mira220(sd);

	i2c_unregister_device(mira220->pmic_client);
	i2c_unregister_device(mira220->uc_client);
	i2c_unregister_device(mira220->led_client);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	mira220_free_controls(mira220);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		mira220_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);

}

static const struct dev_pm_ops mira220_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mira220_suspend, mira220_resume)
	SET_RUNTIME_PM_OPS(mira220_power_off, mira220_power_on, NULL)
};

#endif // __MIRA220_INL__

