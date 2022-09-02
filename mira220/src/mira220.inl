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

#define MIRA220_NATIVE_WIDTH			1600U
#define MIRA220_NATIVE_HEIGHT			1400U

#define MIRA220_PIXEL_ARRAY_LEFT		0U
#define MIRA220_PIXEL_ARRAY_TOP			0U
#define MIRA220_PIXEL_ARRAY_WIDTH		1600U
#define MIRA220_PIXEL_ARRAY_HEIGHT		1400U

#define MIRA220_ANALOG_GAIN_REG			0x400A
#define MIRA220_ANALOG_GAIN_MAX			4
#define MIRA220_ANALOG_GAIN_MIN			1
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
#define MIRA220_ROW_LENGTH_MIN			300

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

#define MIRA220_MAX_ROW_LENGTH			1400
#define MIRA220_MIN_VBLANK			(11 + MIRA220_GLOB_NUM_CLK_CYCLES \
						    / MIRA220_MAX_ROW_LENGTH)

// Default exposure is adjusted to 1 ms
#define MIRA220_DEFAULT_EXPOSURE		0x0b32
#define MIRA220_EXPOSURE_MIN			0

// Power on function timing
#define MIRA220_XCLR_MIN_DELAY_US		100000
#define MIRA220_XCLR_DELAY_RANGE_US		30

/* Chip ID */
#define MIRA220_REG_CHIP_ID		0x0000
#define MIRA220_CHIP_ID			0x0054

#define MIRA220_REG_VALUE_08BIT		1
#define MIRA220_REG_VALUE_16BIT		2

// pixel_rate = link_freq * 2 * nr_of_lanes / bits_per_sample
// 1.5Gb/s * 2 * 2 / 12 = 536870912
#define MIRA220_PIXEL_RATE		(536870912)
/* Should match device tree link freq */
#define MIRA220_DEFAULT_LINK_FREQ	456000000

#define MIRA220_REG_TEST_PATTERN	0x2091
#define	MIRA220_TEST_PATTERN_DISABLE	0x00
#define	MIRA220_TEST_PATTERN_VERTICAL_GRADIENT	0x01

/* Embedded metadata stream structure */
#define MIRA220_EMBEDDED_LINE_WIDTH 16384
#define MIRA220_NUM_EMBEDDED_LINES 1

/* From Jetson driver */
#define MIRA220_DEFAULT_LINE_LENGTH    (0xA80)
#define MIRA220_DEFAULT_PIXEL_CLOCK    (160)
#define MIRA220_DEFAULT_FRAME_LENGTH    (0x07C0)


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

	u32 vblank;
	u32 hblank;
	u32 code;
};

// 1600_1400_30fps_12b_2lanes
static const struct mira220_reg full_1600_1400_30fps_12b_2lanes_reg[] = {
	// Disable internal LDOs connected to VDD25
	{0x401e, 0x02},
	{0x4038, 0x3b},
	// Sensor uploads
        // Stop sensor at a row boundary (stop sensor in a known state)
        // self.stop_img_capture('row')
        // MIPI TX controller disable
        {0x6006, 0x00},
        // MIPI 2 lane mode
        {0x6012, 0x01},
        // MIPI continuous PHY clocking
        {0x6013, 0x00},
        // MIPI TX controller enable
        {0x6006, 0x01},
        // Disable statistics
        {0x205d, 0x00},
        {0x2063, 0x00},
        // Defect pixel correction disabled
        {0x24dc, 0x0},
        {0x24dd, 0x03},
        {0x24de, 0x03},
        {0x24df, 0x00},
        // 2.24MP window
        {0x4006, 0x08},
        {0x401c, 0x6f},
        // Row noise correction enabled, with flat field target of 100
        {0x204b, 0x03},
        {0x205b, 0x64},
        {0x205c, 0x00},
        // Some default values for references (some trim values are in OTP, better to use those)
        {0x4018, 0x3f},
        {0x403b, 0x0b},
        {0x403e, 0x0e},
        {0x402b, 0x06},
        // Misc. sensor settings that should not be touched
        {0x1077, 0x00},
        {0x1078, 0x00},
        {0x1009, 0x08},
        {0x100a, 0x00},
        {0x110f, 0x08},
        {0x1110, 0x00},
        {0x1006, 0x02},
        {0x402c, 0x64},
        {0x3064, 0x00 },
        {0x3065, 0xf0},
        {0x4013, 0x13},
        {0x401f, 0x09},
        {0x4020, 0x13},
        {0x4044, 0x75},
        {0x4027, 0x00},
        {0x3215, 0x69},
        {0x3216, 0x0f},
        {0x322B, 0x69},
        {0x322C, 0x0f},
        {0x4051, 0x80},
        {0x4052, 0x10},
        {0x4057, 0x80},
        {0x4058, 0x10},
        {0x3212, 0x59},
        {0x4047, 0x8f},
        {0x4026, 0x10},
        {0x4032, 0x53},
        {0x4036, 0x17},
        {0x50b8, 0xf4},
        // Related to pixel timing, do not adjust these
        {0x3016, 0x00},
        {0x3017, 0x2c},
        {0x3018, 0x8c},
        {0x3019, 0x45},
        {0x301a, 0x05},
        {0x3013, 0x0a},
        {0x301b, 0x00},
        {0x301c, 0x04},
        {0x301d, 0x88},
        {0x301e, 0x45},
        {0x301f, 0x05},
        {0x3020, 0x00},
        {0x3021, 0x04},
        {0x3022, 0x88},
        {0x3023, 0x45},
        {0x3024, 0x05},
        {0x3025, 0x00},
        {0x3026, 0x04},
        {0x3027, 0x88},
        {0x3028, 0x45},
        {0x3029, 0x05},
        {0x302f, 0x00},
        {0x3056, 0x00},
        {0x3057, 0x00},
        {0x3300, 0x01},
        {0x3301, 0x00},
        {0x3302, 0xb0},
        {0x3303, 0xb0},
        {0x3304, 0x16},
        {0x3305, 0x15},
        {0x3306, 0x01},
        {0x3307, 0x00},
        {0x3308, 0x30},
        {0x3309, 0xa0},
        {0x330a, 0x16},
        {0x330b, 0x15},
        {0x330c, 0x01},
        {0x330d, 0x00},
        {0x330e, 0x30},
        {0x330f, 0xa0},
        {0x3310, 0x16},
        {0x3311, 0x15},
        {0x3312, 0x01},
        {0x3313, 0x00},
        {0x3314, 0x30},
        {0x3315, 0xa0},
        {0x3316, 0x16},
        {0x3317, 0x15},
        {0x3318, 0x01},
        {0x3319, 0x00},
        {0x331a, 0x30},
        {0x331b, 0xa0},
        {0x331c, 0x16},
        {0x331d, 0x15},
        {0x331e, 0x01},
        {0x331f, 0x00},
        {0x3320, 0x30},
        {0x3321, 0xa0},
        {0x3322, 0x16},
        {0x3323, 0x15},
        {0x3324, 0x01},
        {0x3325, 0x00},
        {0x3326, 0x30},
        {0x3327, 0xa0},
        {0x3328, 0x16},
        {0x3329, 0x15},
        {0x332a, 0x2b},
        {0x332b, 0x00},
        {0x332c, 0x30},
        {0x332d, 0xa0},
        {0x332e, 0x16},
        {0x332f, 0x15},
        {0x3330, 0x01},
        {0x3331, 0x00},
        {0x3332, 0x10},
        {0x3333, 0xa0},
        {0x3334, 0x16},
        {0x3335, 0x15},
        {0x3058, 0x08},
        {0x3059, 0x00},
        {0x305a, 0x09},
        {0x305b, 0x00},
        {0x3336, 0x01},
        {0x3337, 0x00},
        {0x3338, 0x90},
        {0x3339, 0xb0},
        {0x333a, 0x16},
        {0x333b, 0x15},
        {0x333c, 0x1f},
        {0x333d, 0x00},
        {0x333e, 0x10},
        {0x333f, 0xa0},
        {0x3340, 0x16},
        {0x3341, 0x15},
        {0x3342, 0x52},
        {0x3343, 0x00},
        {0x3344, 0x10},
        {0x3345, 0x80},
        {0x3346, 0x16},
        {0x3347, 0x15},
        {0x3348, 0x01},
        {0x3349, 0x00},
        {0x334a, 0x10},
        {0x334b, 0x80},
        {0x334c, 0x16},
        {0x334d, 0x1d},
        {0x334e, 0x01},
        {0x334f, 0x00},
        {0x3350, 0x50},
        {0x3351, 0x84},
        {0x3352, 0x16},
        {0x3353, 0x1d},
        {0x3354, 0x18},
        {0x3355, 0x00},
        {0x3356, 0x10},
        {0x3357, 0x84},
        {0x3358, 0x16},
        {0x3359, 0x1d},
        {0x335a, 0x80},
        {0x335b, 0x02},
        {0x335c, 0x10},
        {0x335d, 0xc4},
        {0x335e, 0x14},
        {0x335f, 0x1d},
        {0x3360, 0xa5},
        {0x3361, 0x00},
        {0x3362, 0x10},
        {0x3363, 0x84},
        {0x3364, 0x16},
        {0x3365, 0x1d},
        {0x3366, 0x01},
        {0x3367, 0x00},
        {0x3368, 0x90},
        {0x3369, 0x84},
        {0x336a, 0x16},
        {0x336b, 0x1d},
        {0x336c, 0x12},
        {0x336d, 0x00},
        {0x336e, 0x10},
        {0x336f, 0x84},
        {0x3370, 0x16},
        {0x3371, 0x15},
        {0x3372, 0x32},
        {0x3373, 0x00},
        {0x3374, 0x30},
        {0x3375, 0x84},
        {0x3376, 0x16},
        {0x3377, 0x15},
        {0x3378, 0x26},
        {0x3379, 0x00},
        {0x337a, 0x10},
        {0x337b, 0x84},
        {0x337c, 0x16},
        {0x337d, 0x15},
        {0x337e, 0x80},
        {0x337f, 0x02},
        {0x3380, 0x10},
        {0x3381, 0xc4},
        {0x3382, 0x14},
        {0x3383, 0x15},
        {0x3384, 0xa9},
        {0x3385, 0x00},
        {0x3386, 0x10},
        {0x3387, 0x84},
        {0x3388, 0x16},
        {0x3389, 0x15},
        {0x338a, 0x41},
        {0x338b, 0x00},
        {0x338c, 0x10},
        {0x338d, 0x80},
        {0x338e, 0x16},
        {0x338f, 0x15},
        {0x3390, 0x02},
        {0x3391, 0x00},
        {0x3392, 0x10},
        {0x3393, 0xa0},
        {0x3394, 0x16},
        {0x3395, 0x15},
        {0x305c, 0x18},
        {0x305d, 0x00},
        {0x305e, 0x19},
        {0x305f, 0x00},
        {0x3396, 0x01},
        {0x3397, 0x00},
        {0x3398, 0x90},
        {0x3399, 0x30},
        {0x339a, 0x56},
        {0x339b, 0x57},
        {0x339c, 0x01},
        {0x339d, 0x00},
        {0x339e, 0x10},
        {0x339f, 0x20},
        {0x33a0, 0xd6},
        {0x33a1, 0x17},
        {0x33a2, 0x01},
        {0x33a3, 0x00},
        {0x33a4, 0x10},
        {0x33a5, 0x28},
        {0x33a6, 0xd6},
        {0x33a7, 0x17},
        {0x33a8, 0x03},
        {0x33a9, 0x00},
        {0x33aa, 0x10},
        {0x33ab, 0x20},
        {0x33ac, 0xd6},
        {0x33ad, 0x17},
        {0x33ae, 0x61},
        {0x33af, 0x00},
        {0x33b0, 0x10},
        {0x33b1, 0x20},
        {0x33b2, 0xd6},
        {0x33b3, 0x15},
        {0x33b4, 0x01},
        {0x33b5, 0x00},
        {0x33b6, 0x10},
        {0x33b7, 0x20},
        {0x33b8, 0xd6},
        {0x33b9, 0x1d},
        {0x33ba, 0x01},
        {0x33bb, 0x00},
        {0x33bc, 0x50},
        {0x33bd, 0x20},
        {0x33be, 0xd6},
        {0x33bf, 0x1d},
        {0x33c0, 0x2c},
        {0x33c1, 0x00},
        {0x33c2, 0x10},
        {0x33c3, 0x20},
        {0x33c4, 0xd6},
        {0x33c5, 0x1d},
        {0x33c6, 0x01},
        {0x33c7, 0x00},
        {0x33c8, 0x90},
        {0x33c9, 0x20},
        {0x33ca, 0xd6},
        {0x33cb, 0x1d},
        {0x33cc, 0x83},
        {0x33cd, 0x00},
        {0x33ce, 0x10},
        {0x33cf, 0x20},
        {0x33d0, 0xd6},
        {0x33d1, 0x15},
        {0x33d2, 0x01},
        {0x33d3, 0x00},
        {0x33d4, 0x10},
        {0x33d5, 0x30},
        {0x33d6, 0xd6},
        {0x33d7, 0x15},
        {0x33d8, 0x01},
        {0x33d9, 0x00},
        {0x33da, 0x10},
        {0x33db, 0x20},
        {0x33dc, 0xd6},
        {0x33dd, 0x15},
        {0x33de, 0x01},
        {0x33df, 0x00},
        {0x33e0, 0x10},
        {0x33e1, 0x20},
        {0x33e2, 0x56},
        {0x33e3, 0x15},
        {0x33e4, 0x07},
        {0x33e5, 0x00},
        {0x33e6, 0x10},
        {0x33e7, 0x20},
        {0x33e8, 0x16},
        {0x33e9, 0x15},
        {0x3060, 0x26},
        {0x3061, 0x00},
        {0x302a, 0xff},
        {0x302b, 0xff},
        {0x302c, 0xff},
        {0x302d, 0xff},
        {0x302e, 0x3f},
        {0x3013, 0x0b},
        // Related to ADC timing, do not adjust these
        {0x102b, 0x2c},
        {0x102c, 0x01},
        {0x1035, 0x54},
        {0x1036, 0x00},
        {0x3090, 0x2a},
        {0x3091, 0x01},
        {0x30c6, 0x05},
        {0x30c7, 0x00},
        {0x30c8, 0x00},
        {0x30c9, 0x00},
        {0x30ca, 0x00},
        {0x30cb, 0x00},
        {0x30cc, 0x00},
        {0x30cd, 0x00},
        {0x30ce, 0x00},
        {0x30cf, 0x05},
        {0x30d0, 0x00},
        {0x30d1, 0x00},
        {0x30d2, 0x00},
        {0x30d3, 0x00},
        {0x30d4, 0x00},
        {0x30d5, 0x00},
        {0x30d6, 0x00},
        {0x30d7, 0x00},
        {0x30f3, 0x05},
        {0x30f4, 0x00},
        {0x30f5, 0x00},
        {0x30f6, 0x00},
        {0x30f7, 0x00},
        {0x30f8, 0x00},
        {0x30f9, 0x00},
        {0x30fa, 0x00},
        {0x30fb, 0x00},
        {0x30d8, 0x05},
        {0x30d9, 0x00},
        {0x30da, 0x00},
        {0x30db, 0x00},
        {0x30dc, 0x00},
        {0x30dd, 0x00},
        {0x30de, 0x00},
        {0x30df, 0x00},
        {0x30e0, 0x00},
        {0x30e1, 0x05},
        {0x30e2, 0x00},
        {0x30e3, 0x00},
        {0x30e4, 0x00},
        {0x30e5, 0x00},
        {0x30e6, 0x00},
        {0x30e7, 0x00},
        {0x30e8, 0x00},
        {0x30e9, 0x00},
        {0x30f3, 0x05},
        {0x30f4, 0x02},
        {0x30f5, 0x00},
        {0x30f6, 0x17},
        {0x30f7, 0x01},
        {0x30f8, 0x00},
        {0x30f9, 0x00},
        {0x30fa, 0x00},
        {0x30fb, 0x00},
        {0x30d8, 0x03},
        {0x30d9, 0x01},
        {0x30da, 0x00},
        {0x30db, 0x19},
        {0x30dc, 0x01},
        {0x30dd, 0x00},
        {0x30de, 0x00},
        {0x30df, 0x00},
        {0x30e0, 0x00},
        {0x30a2, 0x05},
        {0x30a3, 0x02},
        {0x30a4, 0x00},
        {0x30a5, 0x22},
        {0x30a6, 0x00},
        {0x30a7, 0x00},
        {0x30a8, 0x00},
        {0x30a9, 0x00},
        {0x30aa, 0x00},
        {0x30ab, 0x05},
        {0x30ac, 0x02},
        {0x30ad, 0x00},
        {0x30ae, 0x22},
        {0x30af, 0x00},
        {0x30b0, 0x00},
        {0x30b1, 0x00},
        {0x30b2, 0x00},
        {0x30b3, 0x00},
        {0x30bd, 0x05},
        {0x30be, 0x9f},
        {0x30bf, 0x00},
        {0x30c0, 0x7d},
        {0x30c1, 0x00},
        {0x30c2, 0x00},
        {0x30c3, 0x00},
        {0x30c4, 0x00},
        {0x30c5, 0x00},
        {0x30b4, 0x04},
        {0x30b5, 0x9c},
        {0x30b6, 0x00},
        {0x30b7, 0x7d},
        {0x30b8, 0x00},
        {0x30b9, 0x00},
        {0x30ba, 0x00},
        {0x30bb, 0x00},
        {0x30bc, 0x00},
        {0x30fc, 0x05},
        {0x30fd, 0x00},
        {0x30fe, 0x00},
        {0x30ff, 0x00},
        {0x3100, 0x00},
        {0x3101, 0x00},
        {0x3102, 0x00},
        {0x3103, 0x00},
        {0x3104, 0x00},
        {0x3105, 0x05},
        {0x3106, 0x00},
        {0x3107, 0x00},
        {0x3108, 0x00},
        {0x3109, 0x00},
        {0x310a, 0x00},
        {0x310b, 0x00},
        {0x310c, 0x00},
        {0x310d, 0x00},
        {0x3099, 0x05},
        {0x309a, 0x96},
        {0x309b, 0x00},
        {0x309c, 0x06},
        {0x309d, 0x00},
        {0x309e, 0x00},
        {0x309f, 0x00},
        {0x30a0, 0x00},
        {0x30a1, 0x00},
        {0x310e, 0x05},
        {0x310f, 0x02},
        {0x3110, 0x00},
        {0x3111, 0x2b},
        {0x3112, 0x00},
        {0x3113, 0x00},
        {0x3114, 0x00},
        {0x3115, 0x00},
        {0x3116, 0x00},
        {0x3117, 0x05},
        {0x3118, 0x02},
        {0x3119, 0x00},
        {0x311a, 0x2c},
        {0x311b, 0x00},
        {0x311c, 0x00},
        {0x311d, 0x00},
        {0x311e, 0x00},
        {0x311f, 0x00},
        {0x30ea, 0x00},
        {0x30eb, 0x00},
        {0x30ec, 0x00},
        {0x30ed, 0x00},
        {0x30ee, 0x00},
        {0x30ef, 0x00},
        {0x30f0, 0x00},
        {0x30f1, 0x00},
        {0x30f2, 0x00},
        {0x313b, 0x03},
        {0x313c, 0x31},
        {0x313d, 0x00},
        {0x313e, 0x07},
        {0x313f, 0x00},
        {0x3140, 0x68},
        {0x3141, 0x00},
        {0x3142, 0x34},
        {0x3143, 0x00},
        {0x31a0, 0x03},
        {0x31a1, 0x16},
        {0x31a2, 0x00},
        {0x31a3, 0x08},
        {0x31a4, 0x00},
        {0x31a5, 0x7e},
        {0x31a6, 0x00},
        {0x31a7, 0x08},
        {0x31a8, 0x00},
        {0x31a9, 0x03},
        {0x31aa, 0x16},
        {0x31ab, 0x00},
        {0x31ac, 0x08},
        {0x31ad, 0x00},
        {0x31ae, 0x7e},
        {0x31af, 0x00},
        {0x31b0, 0x08},
        {0x31b1, 0x00},
        {0x31b2, 0x03},
        {0x31b3, 0x16},
        {0x31b4, 0x00},
        {0x31b5, 0x08},
        {0x31b6, 0x00},
        {0x31b7, 0x7e},
        {0x31b8, 0x00},
        {0x31b9, 0x08},
        {0x31ba, 0x00},
        {0x3120, 0x05},
        {0x3121, 0x45},
        {0x3122, 0x00},
        {0x3123, 0x1d},
        {0x3124, 0x00},
        {0x3125, 0xa9},
        {0x3126, 0x00},
        {0x3127, 0x6d},
        {0x3128, 0x00},
        {0x3129, 0x05},
        {0x312a, 0x15},
        {0x312b, 0x00},
        {0x312c, 0x0a},
        {0x312d, 0x00},
        {0x312e, 0x45},
        {0x312f, 0x00},
        {0x3130, 0x1d},
        {0x3131, 0x00},
        {0x3132, 0x05},
        {0x3133, 0x7d},
        {0x3134, 0x00},
        {0x3135, 0x0a},
        {0x3136, 0x00},
        {0x3137, 0xa9},
        {0x3138, 0x00},
        {0x3139, 0x6d},
        {0x313a, 0x00},
        {0x3144, 0x05},
        {0x3145, 0x00},
        {0x3146, 0x00},
        {0x3147, 0x30},
        {0x3148, 0x00},
        {0x3149, 0x00},
        {0x314a, 0x00},
        {0x314b, 0x00},
        {0x314c, 0x00},
        {0x314d, 0x03},
        {0x314e, 0x00},
        {0x314f, 0x00},
        {0x3150, 0x31},
        {0x3151, 0x00},
        {0x3152, 0x00},
        {0x3153, 0x00},
        {0x3154, 0x00},
        {0x3155, 0x00},
        {0x31d8, 0x05},
        {0x31d9, 0x3a},
        {0x31da, 0x00},
        {0x31db, 0x2e},
        {0x31dc, 0x00},
        {0x31dd, 0x9e},
        {0x31de, 0x00},
        {0x31df, 0x7e},
        {0x31e0, 0x00},
        {0x31e1, 0x05},
        {0x31e2, 0x04},
        {0x31e3, 0x00},
        {0x31e4, 0x04},
        {0x31e5, 0x00},
        {0x31e6, 0x73},
        {0x31e7, 0x00},
        {0x31e8, 0x04},
        {0x31e9, 0x00},
        {0x31ea, 0x05},
        {0x31eb, 0x00},
        {0x31ec, 0x00},
        {0x31ed, 0x00},
        {0x31ee, 0x00},
        {0x31ef, 0x00},
        {0x31f0, 0x00},
        {0x31f1, 0x00},
        {0x31f2, 0x00},
        {0x31f3, 0x00},
        {0x31f4, 0x00},
        {0x31f5, 0x00},
        {0x31f6, 0x00},
        {0x31f7, 0x00},
        {0x31f8, 0x00},
        {0x31f9, 0x00},
        {0x31fa, 0x00},
        {0x31fb, 0x05},
        {0x31fc, 0x00},
        {0x31fd, 0x00},
        {0x31fe, 0x00},
        {0x31ff, 0x00},
        {0x3200, 0x00},
        {0x3201, 0x00},
        {0x3202, 0x00},
        {0x3203, 0x00},
        {0x3204, 0x00},
        {0x3205, 0x00},
        {0x3206, 0x00},
        {0x3207, 0x00},
        {0x3208, 0x00},
        {0x3209, 0x00},
        {0x320a, 0x00},
        {0x320b, 0x00},
        {0x3164, 0x05},
        {0x3165, 0x14},
        {0x3166, 0x00},
        {0x3167, 0x0c},
        {0x3168, 0x00},
        {0x3169, 0x44},
        {0x316a, 0x00},
        {0x316b, 0x1f},
        {0x316c, 0x00},
        {0x316d, 0x05},
        {0x316e, 0x7c},
        {0x316f, 0x00},
        {0x3170, 0x0c},
        {0x3171, 0x00},
        {0x3172, 0xa8},
        {0x3173, 0x00},
        {0x3174, 0x6f},
        {0x3175, 0x00},
        {0x31c4, 0x05},
        {0x31c5, 0x24},
        {0x31c6, 0x01},
        {0x31c7, 0x04},
        {0x31c8, 0x00},
        {0x31c9, 0x05},
        {0x31ca, 0x24},
        {0x31cb, 0x01},
        {0x31cc, 0x04},
        {0x31cd, 0x00},
        {0x31ce, 0x05},
        {0x31cf, 0x24},
        {0x31d0, 0x01},
        {0x31d1, 0x04},
        {0x31d2, 0x00},
        {0x31d3, 0x05},
        {0x31d4, 0x73},
        {0x31d5, 0x00},
        {0x31d6, 0xb1},
        {0x31d7, 0x00},
        {0x3176, 0x05},
        {0x3177, 0x10},
        {0x3178, 0x00},
        {0x3179, 0x56},
        {0x317a, 0x00},
        {0x317b, 0x00},
        {0x317c, 0x00},
        {0x317d, 0x00},
        {0x317e, 0x00},
        {0x317f, 0x05},
        {0x3180, 0x6a},
        {0x3181, 0x00},
        {0x3182, 0xad},
        {0x3183, 0x00},
        {0x3184, 0x00},
        {0x3185, 0x00},
        {0x3186, 0x00},
        {0x3187, 0x00},
        // Exposure time, in row lengths
        {0x100c, 0x7e},
        {0x100d, 0x00},
        // Vertical blanking, in row lengths
        {0x1012, 0x32},
        {0x1013, 0x0b},
        // Enable continuous running
        {0x1002, 0x04},

	// Context B settings
	{0x110a, 0x78},
	{0x110b, 0x05},
	{0x110c, 0x00},
	{0x110d, 0x00},
	{0x1105, 0x00},
	{0x1106, 0x00},
	{0x209a, 0x00},
	{0x209b, 0x00},
	{0x401A, 0x08},
	{0x1103, 0x88},
	{0x1104, 0x2c},

};

// vga_640_480_120fps_12b_2lanes
static const struct mira220_reg vga_640_480_120fps_12b_2lanes_reg[] = {
	// Disable internal LDOs connected to VDD25
	{0x401e,0x2},
	{0x4038,0x3b},
	// Sensor uploads
        // Stop sensor at a row boundary (stop sensor in a known state)
        // self.stop_img_capture('row')
        // MIPI TX controller disable
	// {0x1003,0x2},
	{0x6006,0x0},
	// MIPI 2 lane mode
	{0x6012,0x1},
	{0x6013,0x0},
	{0x6006,0x1},
	{0x205d,0x0},
	{0x2063,0x0},
	{0x24dc,0x13},
	{0x24dd,0x3},
	{0x24de,0x3},
	{0x24df,0x0},
	// 2.24MP window
	{0x4006,0x8},
	{0x401c,0x6f},
	// Row noise correction enabled, with flat field target of 100
	{0x204b,0x3},
	{0x205b,0x64},
	{0x205c,0x0},
	// Some default values for references (some trim values are in OTP, better to use those)
	{0x4018,0x3f},
	{0x403b,0xb},
	{0x403e,0xe},
	{0x402b,0x6},
	// Misc. sensor settings that should not be touched
	{0x1077,0x0},
	{0x1078,0x0},
	{0x1009,0x8},
	{0x100a,0x0},
	{0x110f,0x8},
	{0x1110,0x0},
	{0x1006,0x2},
	{0x402c,0x64},
	{0x3064,0x0},
	{0x3065,0xf0},
	{0x4013,0x13},
	{0x401f,0x9},
	{0x4020,0x13},
	{0x4044,0x75},
	{0x4027,0x0},
	{0x3215,0x69},
	{0x3216,0xf},
	{0x322b,0x69},
	{0x322c,0xf},
	{0x4051,0x80},
	{0x4052,0x10},
	{0x4057,0x80},
	{0x4058,0x10},
	{0x3212,0x59},
	{0x4047,0x8f},
	{0x4026,0x10},
	{0x4032,0x53},
	{0x4036,0x17},
	{0x50b8,0xf4},
	// Related to pixel timing, do not adjust these
	{0x3016,0x0},
	{0x3017,0x2c},
	{0x3018,0x8c},
	{0x3019,0x45},
	{0x301a,0x5},
	{0x3013,0xa},
	{0x301b,0x0},
	{0x301c,0x4},
	{0x301d,0x88},
	{0x301e,0x45},
	{0x301f,0x5},
	{0x3020,0x0},
	{0x3021,0x4},
	{0x3022,0x88},
	{0x3023,0x45},
	{0x3024,0x5},
	{0x3025,0x0},
	{0x3026,0x4},
	{0x3027,0x88},
	{0x3028,0x45},
	{0x3029,0x5},
	{0x302f,0x0},
	{0x3056,0x0},
	{0x3057,0x0},
	{0x3300,0x1},
	{0x3301,0x0},
	{0x3302,0xb0},
	{0x3303,0xb0},
	{0x3304,0x16},
	{0x3305,0x15},
	{0x3306,0x1},
	{0x3307,0x0},
	{0x3308,0x30},
	{0x3309,0xa0},
	{0x330a,0x16},
	{0x330b,0x15},
	{0x330c,0x1},
	{0x330d,0x0},
	{0x330e,0x30},
	{0x330f,0xa0},
	{0x3310,0x16},
	{0x3311,0x15},
	{0x3312,0x1},
	{0x3313,0x0},
	{0x3314,0x30},
	{0x3315,0xa0},
	{0x3316,0x16},
	{0x3317,0x15},
	{0x3318,0x1},
	{0x3319,0x0},
	{0x331a,0x30},
	{0x331b,0xa0},
	{0x331c,0x16},
	{0x331d,0x15},
	{0x331e,0x1},
	{0x331f,0x0},
	{0x3320,0x30},
	{0x3321,0xa0},
	{0x3322,0x16},
	{0x3323,0x15},
	{0x3324,0x1},
	{0x3325,0x0},
	{0x3326,0x30},
	{0x3327,0xa0},
	{0x3328,0x16},
	{0x3329,0x15},
	{0x332a,0x2b},
	{0x332b,0x0},
	{0x332c,0x30},
	{0x332d,0xa0},
	{0x332e,0x16},
	{0x332f,0x15},
	{0x3330,0x1},
	{0x3331,0x0},
	{0x3332,0x10},
	{0x3333,0xa0},
	{0x3334,0x16},
	{0x3335,0x15},
	{0x3058,0x8},
	{0x3059,0x0},
	{0x305a,0x9},
	{0x305b,0x0},
	{0x3336,0x1},
	{0x3337,0x0},
	{0x3338,0x90},
	{0x3339,0xb0},
	{0x333a,0x16},
	{0x333b,0x15},
	{0x333c,0x1f},
	{0x333d,0x0},
	{0x333e,0x10},
	{0x333f,0xa0},
	{0x3340,0x16},
	{0x3341,0x15},
	{0x3342,0x52},
	{0x3343,0x0},
	{0x3344,0x10},
	{0x3345,0x80},
	{0x3346,0x16},
	{0x3347,0x15},
	{0x3348,0x1},
	{0x3349,0x0},
	{0x334a,0x10},
	{0x334b,0x80},
	{0x334c,0x16},
	{0x334d,0x1d},
	{0x334e,0x1},
	{0x334f,0x0},
	{0x3350,0x50},
	{0x3351,0x84},
	{0x3352,0x16},
	{0x3353,0x1d},
	{0x3354,0x18},
	{0x3355,0x0},
	{0x3356,0x10},
	{0x3357,0x84},
	{0x3358,0x16},
	{0x3359,0x1d},
	{0x335a,0x80},
	{0x335b,0x2},
	{0x335c,0x10},
	{0x335d,0xc4},
	{0x335e,0x14},
	{0x335f,0x1d},
	{0x3360,0xa5},
	{0x3361,0x0},
	{0x3362,0x10},
	{0x3363,0x84},
	{0x3364,0x16},
	{0x3365,0x1d},
	{0x3366,0x1},
	{0x3367,0x0},
	{0x3368,0x90},
	{0x3369,0x84},
	{0x336a,0x16},
	{0x336b,0x1d},
	{0x336c,0x12},
	{0x336d,0x0},
	{0x336e,0x10},
	{0x336f,0x84},
	{0x3370,0x16},
	{0x3371,0x15},
	{0x3372,0x32},
	{0x3373,0x0},
	{0x3374,0x30},
	{0x3375,0x84},
	{0x3376,0x16},
	{0x3377,0x15},
	{0x3378,0x26},
	{0x3379,0x0},
	{0x337a,0x10},
	{0x337b,0x84},
	{0x337c,0x16},
	{0x337d,0x15},
	{0x337e,0x80},
	{0x337f,0x2},
	{0x3380,0x10},
	{0x3381,0xc4},
	{0x3382,0x14},
	{0x3383,0x15},
	{0x3384,0xa9},
	{0x3385,0x0},
	{0x3386,0x10},
	{0x3387,0x84},
	{0x3388,0x16},
	{0x3389,0x15},
	{0x338a,0x41},
	{0x338b,0x0},
	{0x338c,0x10},
	{0x338d,0x80},
	{0x338e,0x16},
	{0x338f,0x15},
	{0x3390,0x2},
	{0x3391,0x0},
	{0x3392,0x10},
	{0x3393,0xa0},
	{0x3394,0x16},
	{0x3395,0x15},
	{0x305c,0x18},
	{0x305d,0x0},
	{0x305e,0x19},
	{0x305f,0x0},
	{0x3396,0x1},
	{0x3397,0x0},
	{0x3398,0x90},
	{0x3399,0x30},
	{0x339a,0x56},
	{0x339b,0x57},
	{0x339c,0x1},
	{0x339d,0x0},
	{0x339e,0x10},
	{0x339f,0x20},
	{0x33a0,0xd6},
	{0x33a1,0x17},
	{0x33a2,0x1},
	{0x33a3,0x0},
	{0x33a4,0x10},
	{0x33a5,0x28},
	{0x33a6,0xd6},
	{0x33a7,0x17},
	{0x33a8,0x3},
	{0x33a9,0x0},
	{0x33aa,0x10},
	{0x33ab,0x20},
	{0x33ac,0xd6},
	{0x33ad,0x17},
	{0x33ae,0x61},
	{0x33af,0x0},
	{0x33b0,0x10},
	{0x33b1,0x20},
	{0x33b2,0xd6},
	{0x33b3,0x15},
	{0x33b4,0x1},
	{0x33b5,0x0},
	{0x33b6,0x10},
	{0x33b7,0x20},
	{0x33b8,0xd6},
	{0x33b9,0x1d},
	{0x33ba,0x1},
	{0x33bb,0x0},
	{0x33bc,0x50},
	{0x33bd,0x20},
	{0x33be,0xd6},
	{0x33bf,0x1d},
	{0x33c0,0x2c},
	{0x33c1,0x0},
	{0x33c2,0x10},
	{0x33c3,0x20},
	{0x33c4,0xd6},
	{0x33c5,0x1d},
	{0x33c6,0x1},
	{0x33c7,0x0},
	{0x33c8,0x90},
	{0x33c9,0x20},
	{0x33ca,0xd6},
	{0x33cb,0x1d},
	{0x33cc,0x83},
	{0x33cd,0x0},
	{0x33ce,0x10},
	{0x33cf,0x20},
	{0x33d0,0xd6},
	{0x33d1,0x15},
	{0x33d2,0x1},
	{0x33d3,0x0},
	{0x33d4,0x10},
	{0x33d5,0x30},
	{0x33d6,0xd6},
	{0x33d7,0x15},
	{0x33d8,0x1},
	{0x33d9,0x0},
	{0x33da,0x10},
	{0x33db,0x20},
	{0x33dc,0xd6},
	{0x33dd,0x15},
	{0x33de,0x1},
	{0x33df,0x0},
	{0x33e0,0x10},
	{0x33e1,0x20},
	{0x33e2,0x56},
	{0x33e3,0x15},
	{0x33e4,0x7},
	{0x33e5,0x0},
	{0x33e6,0x10},
	{0x33e7,0x20},
	{0x33e8,0x16},
	{0x33e9,0x15},
	{0x3060,0x26},
	{0x3061,0x0},
	{0x302a,0xff},
	{0x302b,0xff},
	{0x302c,0xff},
	{0x302d,0xff},
	{0x302e,0x3f},
	{0x3013,0xb},
	// Related to ADC timing, do not adjust these
	{0x102b,0x2c},
	{0x102c,0x1},
	{0x1035,0x54},
	{0x1036,0x0},
	{0x3090,0x2a},
	{0x3091,0x1},
	{0x30c6,0x5},
	{0x30c7,0x0},
	{0x30c8,0x0},
	{0x30c9,0x0},
	{0x30ca,0x0},
	{0x30cb,0x0},
	{0x30cc,0x0},
	{0x30cd,0x0},
	{0x30ce,0x0},
	{0x30cf,0x5},
	{0x30d0,0x0},
	{0x30d1,0x0},
	{0x30d2,0x0},
	{0x30d3,0x0},
	{0x30d4,0x0},
	{0x30d5,0x0},
	{0x30d6,0x0},
	{0x30d7,0x0},
	{0x30f3,0x5},
	{0x30f4,0x0},
	{0x30f5,0x0},
	{0x30f6,0x0},
	{0x30f7,0x0},
	{0x30f8,0x0},
	{0x30f9,0x0},
	{0x30fa,0x0},
	{0x30fb,0x0},
	{0x30d8,0x5},
	{0x30d9,0x0},
	{0x30da,0x0},
	{0x30db,0x0},
	{0x30dc,0x0},
	{0x30dd,0x0},
	{0x30de,0x0},
	{0x30df,0x0},
	{0x30e0,0x0},
	{0x30e1,0x5},
	{0x30e2,0x0},
	{0x30e3,0x0},
	{0x30e4,0x0},
	{0x30e5,0x0},
	{0x30e6,0x0},
	{0x30e7,0x0},
	{0x30e8,0x0},
	{0x30e9,0x0},
	{0x30f3,0x5},
	{0x30f4,0x2},
	{0x30f5,0x0},
	{0x30f6,0x17},
	{0x30f7,0x1},
	{0x30f8,0x0},
	{0x30f9,0x0},
	{0x30fa,0x0},
	{0x30fb,0x0},
	{0x30d8,0x3},
	{0x30d9,0x1},
	{0x30da,0x0},
	{0x30db,0x19},
	{0x30dc,0x1},
	{0x30dd,0x0},
	{0x30de,0x0},
	{0x30df,0x0},
	{0x30e0,0x0},
	{0x30a2,0x5},
	{0x30a3,0x2},
	{0x30a4,0x0},
	{0x30a5,0x22},
	{0x30a6,0x0},
	{0x30a7,0x0},
	{0x30a8,0x0},
	{0x30a9,0x0},
	{0x30aa,0x0},
	{0x30ab,0x5},
	{0x30ac,0x2},
	{0x30ad,0x0},
	{0x30ae,0x22},
	{0x30af,0x0},
	{0x30b0,0x0},
	{0x30b1,0x0},
	{0x30b2,0x0},
	{0x30b3,0x0},
	{0x30bd,0x5},
	{0x30be,0x9f},
	{0x30bf,0x0},
	{0x30c0,0x7d},
	{0x30c1,0x0},
	{0x30c2,0x0},
	{0x30c3,0x0},
	{0x30c4,0x0},
	{0x30c5,0x0},
	{0x30b4,0x4},
	{0x30b5,0x9c},
	{0x30b6,0x0},
	{0x30b7,0x7d},
	{0x30b8,0x0},
	{0x30b9,0x0},
	{0x30ba,0x0},
	{0x30bb,0x0},
	{0x30bc,0x0},
	{0x30fc,0x5},
	{0x30fd,0x0},
	{0x30fe,0x0},
	{0x30ff,0x0},
	{0x3100,0x0},
	{0x3101,0x0},
	{0x3102,0x0},
	{0x3103,0x0},
	{0x3104,0x0},
	{0x3105,0x5},
	{0x3106,0x0},
	{0x3107,0x0},
	{0x3108,0x0},
	{0x3109,0x0},
	{0x310a,0x0},
	{0x310b,0x0},
	{0x310c,0x0},
	{0x310d,0x0},
	{0x3099,0x5},
	{0x309a,0x96},
	{0x309b,0x0},
	{0x309c,0x6},
	{0x309d,0x0},
	{0x309e,0x0},
	{0x309f,0x0},
	{0x30a0,0x0},
	{0x30a1,0x0},
	{0x310e,0x5},
	{0x310f,0x2},
	{0x3110,0x0},
	{0x3111,0x2b},
	{0x3112,0x0},
	{0x3113,0x0},
	{0x3114,0x0},
	{0x3115,0x0},
	{0x3116,0x0},
	{0x3117,0x5},
	{0x3118,0x2},
	{0x3119,0x0},
	{0x311a,0x2c},
	{0x311b,0x0},
	{0x311c,0x0},
	{0x311d,0x0},
	{0x311e,0x0},
	{0x311f,0x0},
	{0x30ea,0x0},
	{0x30eb,0x0},
	{0x30ec,0x0},
	{0x30ed,0x0},
	{0x30ee,0x0},
	{0x30ef,0x0},
	{0x30f0,0x0},
	{0x30f1,0x0},
	{0x30f2,0x0},
	{0x313b,0x3},
	{0x313c,0x31},
	{0x313d,0x0},
	{0x313e,0x7},
	{0x313f,0x0},
	{0x3140,0x68},
	{0x3141,0x0},
	{0x3142,0x34},
	{0x3143,0x0},
	{0x31a0,0x3},
	{0x31a1,0x16},
	{0x31a2,0x0},
	{0x31a3,0x8},
	{0x31a4,0x0},
	{0x31a5,0x7e},
	{0x31a6,0x0},
	{0x31a7,0x8},
	{0x31a8,0x0},
	{0x31a9,0x3},
	{0x31aa,0x16},
	{0x31ab,0x0},
	{0x31ac,0x8},
	{0x31ad,0x0},
	{0x31ae,0x7e},
	{0x31af,0x0},
	{0x31b0,0x8},
	{0x31b1,0x0},
	{0x31b2,0x3},
	{0x31b3,0x16},
	{0x31b4,0x0},
	{0x31b5,0x8},
	{0x31b6,0x0},
	{0x31b7,0x7e},
	{0x31b8,0x0},
	{0x31b9,0x8},
	{0x31ba,0x0},
	{0x3120,0x5},
	{0x3121,0x45},
	{0x3122,0x0},
	{0x3123,0x1d},
	{0x3124,0x0},
	{0x3125,0xa9},
	{0x3126,0x0},
	{0x3127,0x6d},
	{0x3128,0x0},
	{0x3129,0x5},
	{0x312a,0x15},
	{0x312b,0x0},
	{0x312c,0xa},
	{0x312d,0x0},
	{0x312e,0x45},
	{0x312f,0x0},
	{0x3130,0x1d},
	{0x3131,0x0},
	{0x3132,0x5},
	{0x3133,0x7d},
	{0x3134,0x0},
	{0x3135,0xa},
	{0x3136,0x0},
	{0x3137,0xa9},
	{0x3138,0x0},
	{0x3139,0x6d},
	{0x313a,0x0},
	{0x3144,0x5},
	{0x3145,0x0},
	{0x3146,0x0},
	{0x3147,0x30},
	{0x3148,0x0},
	{0x3149,0x0},
	{0x314a,0x0},
	{0x314b,0x0},
	{0x314c,0x0},
	{0x314d,0x3},
	{0x314e,0x0},
	{0x314f,0x0},
	{0x3150,0x31},
	{0x3151,0x0},
	{0x3152,0x0},
	{0x3153,0x0},
	{0x3154,0x0},
	{0x3155,0x0},
	{0x31d8,0x5},
	{0x31d9,0x3a},
	{0x31da,0x0},
	{0x31db,0x2e},
	{0x31dc,0x0},
	{0x31dd,0x9e},
	{0x31de,0x0},
	{0x31df,0x7e},
	{0x31e0,0x0},
	{0x31e1,0x5},
	{0x31e2,0x4},
	{0x31e3,0x0},
	{0x31e4,0x4},
	{0x31e5,0x0},
	{0x31e6,0x73},
	{0x31e7,0x0},
	{0x31e8,0x4},
	{0x31e9,0x0},
	{0x31ea,0x5},
	{0x31eb,0x0},
	{0x31ec,0x0},
	{0x31ed,0x0},
	{0x31ee,0x0},
	{0x31ef,0x0},
	{0x31f0,0x0},
	{0x31f1,0x0},
	{0x31f2,0x0},
	{0x31f3,0x0},
	{0x31f4,0x0},
	{0x31f5,0x0},
	{0x31f6,0x0},
	{0x31f7,0x0},
	{0x31f8,0x0},
	{0x31f9,0x0},
	{0x31fa,0x0},
	{0x31fb,0x5},
	{0x31fc,0x0},
	{0x31fd,0x0},
	{0x31fe,0x0},
	{0x31ff,0x0},
	{0x3200,0x0},
	{0x3201,0x0},
	{0x3202,0x0},
	{0x3203,0x0},
	{0x3204,0x0},
	{0x3205,0x0},
	{0x3206,0x0},
	{0x3207,0x0},
	{0x3208,0x0},
	{0x3209,0x0},
	{0x320a,0x0},
	{0x320b,0x0},
	{0x3164,0x5},
	{0x3165,0x14},
	{0x3166,0x0},
	{0x3167,0xc},
	{0x3168,0x0},
	{0x3169,0x44},
	{0x316a,0x0},
	{0x316b,0x1f},
	{0x316c,0x0},
	{0x316d,0x5},
	{0x316e,0x7c},
	{0x316f,0x0},
	{0x3170,0xc},
	{0x3171,0x0},
	{0x3172,0xa8},
	{0x3173,0x0},
	{0x3174,0x6f},
	{0x3175,0x0},
	{0x31c4,0x5},
	{0x31c5,0x24},
	{0x31c6,0x1},
	{0x31c7,0x4},
	{0x31c8,0x0},
	{0x31c9,0x5},
	{0x31ca,0x24},
	{0x31cb,0x1},
	{0x31cc,0x4},
	{0x31cd,0x0},
	{0x31ce,0x5},
	{0x31cf,0x24},
	{0x31d0,0x1},
	{0x31d1,0x4},
	{0x31d2,0x0},
	{0x31d3,0x5},
	{0x31d4,0x73},
	{0x31d5,0x0},
	{0x31d6,0xb1},
	{0x31d7,0x0},
	{0x3176,0x5},
	{0x3177,0x10},
	{0x3178,0x0},
	{0x3179,0x56},
	{0x317a,0x0},
	{0x317b,0x0},
	{0x317c,0x0},
	{0x317d,0x0},
	{0x317e,0x0},
	{0x317f,0x5},
	{0x3180,0x6a},
	{0x3181,0x0},
	{0x3182,0xad},
	{0x3183,0x0},
	{0x3184,0x0},
	{0x3185,0x0},
	{0x3186,0x0},
	{0x3187,0x0},
	// Exposure time, in row lengths
	{0x100c,0x7e},
	{0x100d,0x0},
	// Vertical blanking, in row lengths
	{0x1012,0x32},
	{0x1013,0xb},
	// Enable continuous running
	{0x1002,0x4},
	// TODO: check the 2 writes below
	// {0x3013,0xa},
	// {0x3013,0xb},
	// Context B settings
	{0x110a,0x78},
	{0x110b,0x5},
	{0x110c,0x0},
	{0x110d,0x0},
	{0x1105,0x0},
	{0x1106,0x0},
	{0x209a,0x0},
	{0x209b,0x0},
	// TODO: check the 2 writes below
	// {0x1113,0x2c},
	// {0x1114,0x1},
	{0x401a,0x8},
	{0x1103,0x88},
	{0x1104,0x2c},
	// TODO: check the 2 writes below
	// {0x1003,0x10},
	// {0x1002,0x4},
	// {0x10f0,0x1},
	// setting bpp 12bit
	// {0x1003,0x4},
	{0x209e,0x2},
	{0x208d,0x4},
	// {0x1003,0x10},
	// {0x1002,0x4},
	// {0x10f0,0x1},
	{0x1003,0x4},
	// setting roi w640 h480
	{0x1087,0xe0},
	{0x1088,0x1},
	{0x107d,0xcc},
	{0x107e,0x1},
	{0x2008,0x40},
	{0x2009,0x1},
	{0x200a,0xf0},
	{0x200b,0x0},
	{0x207d,0x80},
	{0x207e,0x2},
	// {0x1003,0x10},
	// {0x1002,0x4},
	// {0x10f0,0x1},
	// setting fps 120
	{0x1012,0x4a},
	{0x1013,0x2},
	// setting analog gain to 1
	{0x400a,0x8},
	{0x4009,0x1e},
	// setting mipi speed to 1000
	// {0x1003,0x4},
	{0x6006,0x0},
	{0x5004,0x1},
	{0x5086,0x2},
	{0x5087,0x34},
	{0x5088,0x0},
	{0x5090,0x0},
	{0x5091,0x5},
	{0x5092,0xe},
	{0x5093,0xb},
	{0x5094,0x4},
	{0x5095,0x22},
	{0x5096,0xb},
	{0x5097,0x0},
	{0x5098,0xd},
	{0x5004,0x0},
	{0x2066,0x0},
	{0x2067,0xc},
	{0x206e,0x80},
	{0x206f,0xb},
	{0x20ac,0x80},
	{0x20ad,0xb},
	{0x2076,0x0},
	{0x2077,0x6},
	{0x20b4,0x0},
	{0x20b5,0x6},
	{0x2078,0x1e},
	{0x2079,0x6},
	{0x20b6,0x1e},
	{0x20b7,0x6},
	{0x207a,0xd4},
	{0x207b,0x6},
	{0x20b8,0xd4},
	{0x20b9,0x6},
	{0x6006,0x1},
	{0x102b,0xc2},
	{0x102c,0x1},
	// {0x1003,0x10},
	// {0x1002,0x4},
	// {0x10f0,0x1},
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
		.vblank = 16, // ceil(1928 / 450) + 11
		.hblank = 0, // TODO
		.code = MEDIA_BUS_FMT_SGRBG12_1X12,
	},
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
			.num_of_regs = ARRAY_SIZE(full_1600_1400_30fps_12b_2lanes_reg),
			.regs = full_1600_1400_30fps_12b_2lanes_reg,
		},
		.vblank = 2866,
		.hblank = 0, // TODO
		.code = MEDIA_BUS_FMT_SGRBG12_1X12,
	},
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

	/* Current mode */
	const struct mira220_mode *mode;

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;
	/* Whether regulator is enabled */
	bool regulator_enabled;
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

// Returns the maximum exposure time in clock cycles (reg value)
static u32 mira220_calculate_max_exposure_time(u32 row_length, u32 vsize,
					       u32 vblank) {
  return row_length * (vsize + vblank) - MIRA220_GLOB_NUM_CLK_CYCLES;
}

static int mira220_write_analog_gain_reg(struct mira220 *mira220, u8 gain) {
	struct i2c_client* const client = v4l2_get_subdevdata(&mira220->sd);
	u32 reg_value;
	u32 ret;

	if ((gain < MIRA220_ANALOG_GAIN_MIN) || (gain > MIRA220_ANALOG_GAIN_MAX)) {
		return -EINVAL;
	}

	reg_value = 8 / gain;
	ret = mira220_write(mira220, MIRA220_ANALOG_GAIN_REG, reg_value);

	if (ret) {
		dev_err_ratelimited(&client->dev, "Error setting analog gain register to %d",
				reg_value);
	}

	return ret;
}

static int mira220_write_exposure_reg(struct mira220 *mira220, u32 exposure) {
	struct i2c_client* const client = v4l2_get_subdevdata(&mira220->sd);
	const u32 max_exposure = mira220_calculate_max_exposure_time(MIRA220_ROW_LENGTH_MIN,
		mira220->mode->height, mira220->mode->vblank);
	u32 ret = 0;

	if (exposure > max_exposure) {
		return -EINVAL;
	}

	ret = mira220_write16(mira220, MIRA220_EXP_TIME_LO_REG, exposure);
	if (ret) {
		dev_err_ratelimited(&client->dev, "Error setting exposure time to %d", exposure);
		return -EINVAL;
	}

	return 0;
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

        usleep_range(frame_time, frame_time + 1000);

	return ret;
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
		exposure_max = mira220->mode->height + ctrl->val - 4;
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
	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = mira220_write_analog_gain_reg(mira220, ctrl->val);
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
				        mira220->mode->height + ctrl->val);
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

static const struct v4l2_ctrl_ops mira220_ctrl_ops = {
	.s_ctrl = mira220_set_ctrl,
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
			max_exposure = mira220_calculate_max_exposure_time(MIRA220_ROW_LENGTH_MIN,
									   mira220->mode->height,
									   mira220->mode->vblank);
			default_exp = MIRA220_DEFAULT_EXPOSURE > max_exposure ? max_exposure : MIRA220_DEFAULT_EXPOSURE;
			printk(KERN_INFO "[MIRA220]: mira220_set_pad_format() min_exp %d max_exp %d, default_exp %d\n",
					MIRA220_EXPOSURE_MIN, max_exposure, default_exp);
			__v4l2_ctrl_modify_range(mira220->exposure,
						     MIRA220_EXPOSURE_MIN,
						     max_exposure, 1,
						     default_exp);

			// Set the current vblank value
			printk(KERN_INFO "[MIRA220]: mira220_set_pad_format() mira220->mode->vblank %d\n",
					mira220->mode->vblank);

			__v4l2_ctrl_s_ctrl(mira220->vblank, mira220->mode->vblank);
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

	ret = pm_runtime_get_sync(&client->dev);
	if (ret < 0) {
		//printk(KERN_INFO "[MIRA220]: get_sync failed, but continue.\n");
		pm_runtime_put_noidle(&client->dev);
		return ret;
	}

	printk(KERN_INFO "[MIRA220]: Writing stop streaming regs.\n");

	ret = mira220_write_stop_streaming_regs(mira220);
	if (ret) {
		dev_err(&client->dev, "Could not write stream-on sequence");
		goto err_rpm_put;
	}


	/* Apply default values of current mode */
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

	printk(KERN_INFO "[MIRA220]: Entering v4l2 ctrl handler setup function.\n");

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(mira220->sd.ctrl_handler);
	printk(KERN_INFO "[MIRA220]: __v4l2_ctrl_handler_setup ret = %d.\n", ret);
	if (ret)
		goto err_rpm_put;

	printk(KERN_INFO "[MIRA220]: Writing start streaming regs.\n");

	ret = mira220_write_start_streaming_regs(mira220);
	if (ret) {
		dev_err(&client->dev, "Could not write stream-on sequence");
		goto err_rpm_put;
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

	ret = mira220_write_stop_streaming_regs(mira220);
	if (ret) {
		dev_err(&client->dev, "Could not write the stream-off sequence");
	}

	/* Unlock controls for vflip and hflip */
	__v4l2_ctrl_grab(mira220->vflip, false);
	__v4l2_ctrl_grab(mira220->hflip, false);

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

/* Power/clock management functions */
static int mira220_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira220 *mira220 = to_mira220(sd);
	int ret = -EINVAL;

	printk(KERN_INFO "[MIRA220]: Entering power on function.\n");

	/* Pull reset to low if it is high */
	if (mira220->regulator_enabled) {
		ret = regulator_bulk_disable(MIRA220_NUM_SUPPLIES, mira220->supplies);
		if (ret) {
			dev_err(&client->dev, "%s: failed to disable regulators\n",
				__func__);
			return ret;
		}
		usleep_range(MIRA220_XCLR_MIN_DELAY_US,
			     MIRA220_XCLR_MIN_DELAY_US + MIRA220_XCLR_DELAY_RANGE_US);
		mira220->regulator_enabled = false;
	}

	ret = regulator_bulk_enable(MIRA220_NUM_SUPPLIES, mira220->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}
	mira220->regulator_enabled = true;

	ret = clk_prepare_enable(mira220->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}

	// gpiod_set_value_cansleep(mira220->reset_gpio, 1);
	usleep_range(MIRA220_XCLR_MIN_DELAY_US,
		     MIRA220_XCLR_MIN_DELAY_US + MIRA220_XCLR_DELAY_RANGE_US);
	return 0;

reg_off:
	ret = regulator_bulk_disable(MIRA220_NUM_SUPPLIES, mira220->supplies);
	mira220->regulator_enabled = false;
	return ret;
}

static int mira220_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira220 *mira220 = to_mira220(sd);

	printk(KERN_INFO "[MIRA220]: Entering power off function.\n");

	// regulator_bulk_disable(MIRA220_NUM_SUPPLIES, mira220->supplies);
	// mira220->regulator_enabled = false;
	clk_disable_unprepare(mira220->xclk);

	return 0;
}

static int __maybe_unused mira220_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira220 *mira220 = to_mira220(sd);

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
	u32 max_exposure = 0;

	ctrl_hdlr = &mira220->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 11);
	if (ret)
		return ret;

	mutex_init(&mira220->mutex);
	ctrl_hdlr->lock = &mira220->mutex;

	printk(KERN_INFO "[MIRA220]: %s V4L2_CID_PIXEL_RATE %X.\n", __func__, V4L2_CID_PIXEL_RATE);

	/* By default, PIXEL_RATE is read only */
	mira220->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &mira220_ctrl_ops,
					        V4L2_CID_PIXEL_RATE,
					        MIRA220_PIXEL_RATE,
					        MIRA220_PIXEL_RATE, 1,
					        MIRA220_PIXEL_RATE);

	printk(KERN_INFO "[MIRA220]: %s V4L2_CID_VBLANK %X.\n", __func__, V4L2_CID_VBLANK);

	mira220->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &mira220_ctrl_ops,
					   V4L2_CID_VBLANK, MIRA220_MIN_VBLANK,
					   0xFFFF, 1,
					   mira220->mode->vblank);

	printk(KERN_INFO "[MIRA220]: %s V4L2_CID_HBLANK %X.\n", __func__, V4L2_CID_HBLANK);

	mira220->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &mira220_ctrl_ops,
					   V4L2_CID_HBLANK, 0x0000,
					   0xFFFF, 1,
					   mira220->mode->hblank);
	if (mira220->hblank)
		mira220->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;


	// Make the vblank control read only. This could be changed to allow changing framerate in
	// runtime, but would require adapting other settings
	// mira220->vblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	// Exposure is indicated in number of lines here
	// Max is determined by vblank + vsize and Tglob.
	max_exposure = mira220_calculate_max_exposure_time(MIRA220_ROW_LENGTH_MIN,
	                                                   mira220->mode->height,
	                                                   mira220->mode->vblank);

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

static int mira220_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct mira220 *mira220;
	int ret;

	printk(KERN_INFO "[MIRA220]: probing v4l2 sensor.\n");
	printk(KERN_INFO "[MIRA220]: Driver Version 0.0.\n");

	dev_err(dev, "[MIRA220] name: %s.\n", client->name);
	dev_err(dev, "[MIRA220] Sleep for 1 second to let PMIC driver complete init.\n");
	usleep_range(1000000, 1000000+100);

	mira220 = devm_kzalloc(&client->dev, sizeof(*mira220), GFP_KERNEL);
	if (!mira220)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&mira220->sd, client, &mira220_subdev_ops);

	/* Check the hardware configuration in device tree */
	if (mira220_check_hwcfg(dev))
		return -EINVAL;

	// TODO(jalv): Get GPIO's, regulators and clocks from dts

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

	return ret;
}

static int mira220_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira220 *mira220 = to_mira220(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	mira220_free_controls(mira220);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		mira220_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

static const struct dev_pm_ops mira220_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mira220_suspend, mira220_resume)
	SET_RUNTIME_PM_OPS(mira220_power_off, mira220_power_on, NULL)
};

#endif // __MIRA220_INL__

