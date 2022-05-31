// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for ams MIRA050 cameras.
 * Copyright (C) 2022, ams-OSRAM
 *
 * Based on Sony IMX219 camera driver
 * Copyright (C) 2019, Raspberry Pi (Trading) Ltd
 */

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

#define MIRA050_NATIVE_WIDTH			600U
#define MIRA050_NATIVE_HEIGHT			800U

#define MIRA050_PIXEL_ARRAY_LEFT		0U
#define MIRA050_PIXEL_ARRAY_TOP			0U
#define MIRA050_PIXEL_ARRAY_WIDTH		600U
#define MIRA050_PIXEL_ARRAY_HEIGHT		800U

#define MIRA050_ANALOG_GAIN_REG			0x400A
#define MIRA050_ANALOG_GAIN_MAX			4
#define MIRA050_ANALOG_GAIN_MIN			1
#define MIRA050_ANALOG_GAIN_STEP		1
#define MIRA050_ANALOG_GAIN_DEFAULT		MIRA050_ANALOG_GAIN_MIN

#define MIRA050_BIT_DEPTH_REG			0x209E
#define MIRA050_BIT_DEPTH_12_BIT		0x02
#define MIRA050_BIT_DEPTH_10_BIT		0x04
#define MIRA050_BIT_DEPTH_8_BIT			0x06

#define MIRA050_CSI_DATA_TYPE_REG		0x208D
#define MIRA050_CSI_DATA_TYPE_12_BIT		0x04
#define MIRA050_CSI_DATA_TYPE_10_BIT		0x02
#define MIRA050_CSI_DATA_TYPE_8_BIT		0x01

#define MIRA050_BANK_SEL_REG			0xE000
#define MIRA050_RW_CONTEXT_REG			0xE004
#define MIRA050_CMD_REQ_1_REG			0x000A
#define MIRA050_CMD_HALT_BLOCK_REG		0x000C

#define MIRA050_NB_OF_FRAMES_LO_REG		0x10F2
#define MIRA050_NB_OF_FRAMES_HI_REG		0x10F3

#define MIRA050_POWER_MODE_REG			0x0043
#define MIRA050_POWER_MODE_SLEEP		0x01
#define MIRA050_POWER_MODE_IDLE			0x02
#define MIRA050_POWER_MODE_ACTIVE		0x0C

// Exposure time is indicated in number of rows
#define MIRA050_EXP_TIME_LO_REG			0x100C
#define MIRA050_EXP_TIME_HI_REG			0x100D

// VBLANK is indicated in number of rows
#define MIRA050_VBLANK_LO_REG			0x1012
#define MIRA050_VBLANK_HI_REG			0x1013

#define MIRA050_EXT_EXP_PW_SEL_REG		0x1001
#define MIRA050_EXT_EXP_PW_SEL_USE_REG		1
#define MIRA050_EXT_EXP_PW_SEL_USE_EXT		0

// Exposure delay is indicated in number of rows
#define MIRA050_EXT_EXP_DELAY_LO_REG		0x10D0
#define MIRA050_EXT_EXP_DELAY_HI_REG		0x10D1

// Sets the duration of the row length in clock cycles of CLK_IN
#define MIRA050_ROW_LENGTH_LO_REG		0x102B
#define MIRA050_ROW_LENGTH_HI_REG		0x102C
#define MIRA050_ROW_LENGTH_MIN			300

#define MIRA050_VSIZE1_LO_REG			0x1087
#define MIRA050_VSIZE1_HI_REG			0x1088
#define MIRA050_VSIZE1_MASK			0x7FF

#define MIRA050_VSTART1_LO_REG			0x107D
#define MIRA050_VSTART1_HI_REG			0x107E
#define MIRA050_VSTART1_MASK			0x7FF

// HSIZE units are number of columns / 2
#define MIRA050_HSIZE_LO_REG			0x2008
#define MIRA050_HSIZE_HI_REG			0x2009
#define MIRA050_HSIZE_MASK			0x3FF

// HSTART units are number of columns / 2
#define MIRA050_HSTART_LO_REG			0x200A
#define MIRA050_HSTART_HI_REG			0x200B
#define MIRA050_HSTART_MASK			0x3FF

// MIPI_HSIZE units are number of columns (HSIZE * 2)
#define MIRA050_MIPI_HSIZE_LO_REG		0x207D
#define MIRA050_MIPI_HSIZE_HI_REG		0x207E
#define MIRA050_MIPI_HSIZE_MASK			0xFFFF

#define MIRA050_HFLIP_REG			0x209C
#define MIRA050_HFLIP_ENABLE_MIRROR		1
#define MIRA050_HFLIP_DISABLE_MIRROR		0

#define MIRA050_VFLIP_REG			0x1095
#define MIRA050_VFLIP_ENABLE_FLIP		1
#define MIRA050_VFLIP_DISABLE_FLIP		0

#define MIRA050_BIT_ORDER_REG			0x2063
#define MIRA050_BIT_ORDER_NORMAL		0
#define MIRA050_BIT_ORDER_REVERSED		1

#define MIRA050_BSP_REG				0x4006
#define MIRA050_BSP_ENABLE			0x08
#define MIRA050_BSP_DISABLE			0x0F

#define MIRA050_MIPI_SOFT_RESET_REG		0x5004
#define MIRA050_MIPI_SOFT_RESET_DPHY		0x01
#define MIRA050_MIPI_SOFT_RESET_NONE		0x00

#define MIRA050_FSYNC_EOF_MAX_CTR_LO_REG	0x2066
#define MIRA050_FSYNC_EOF_MAX_CTR_HI_REG	0x2067

#define MIRA050_FSYNC_EOF_VEND_ST_LO_REG	0x206E
#define MIRA050_FSYNC_EOF_VEND_ST_HI_REG	0x206F

#define MIRA050_FSYNC_EOF_HSTART_EMB_ST_LO_REG	0x2076
#define MIRA050_FSYNC_EOF_HSTART_EMB_ST_HI_REG	0x2077

#define MIRA050_FSYNC_EOF_DSTART_EMB_ST_LO_REG	0x2078
#define MIRA050_FSYNC_EOF_DSTART_EMB_ST_HI_REG	0x2079

#define MIRA050_FSYNC_EOF_HEND_EMB_ST_LO_REG	0x207A
#define MIRA050_FSYNC_EOF_HEND_EMB_ST_HI_REG	0x207B

#define MIRA050_GLOB_NUM_CLK_CYCLES		1928

#define MIRA050_SUPPORTED_XCLK_FREQ		24000000

#define MIRA050_MIN_ROW_LENGTH			300
#define MIRA050_MIN_VBLANK			(11 + MIRA050_GLOB_NUM_CLK_CYCLES \
						    / MIRA050_MIN_ROW_LENGTH)

// Default exposure is adjusted to 1 ms
#define MIRA050_DEFAULT_EXPOSURE		0x0b32
#define MIRA050_EXPOSURE_MIN			0

// Power on function timing
#define MIRA050_XCLR_MIN_DELAY_US		40000
#define MIRA050_XCLR_DELAY_RANGE_US		30

/* Chip ID */
#define MIRA050_REG_CHIP_ID		0x0000
#define MIRA050_CHIP_ID			0x0054

#define MIRA050_REG_VALUE_08BIT		1
#define MIRA050_REG_VALUE_16BIT		2

// pixel_rate = link_freq * 2 * nr_of_lanes / bits_per_sample
// 1.5Gb/s * 2 * 2 / 12 = 536870912
#define MIRA050_PIXEL_RATE		(536870912)
/* Should match device tree link freq */
#define MIRA050_DEFAULT_LINK_FREQ	456000000

#define MIRA050_REG_TEST_PATTERN	0x2091
#define	MIRA050_TEST_PATTERN_DISABLE	0x00
#define	MIRA050_TEST_PATTERN_VERTICAL_GRADIENT	0x01

/* Embedded metadata stream structure */
#define MIRA050_EMBEDDED_LINE_WIDTH 16384
#define MIRA050_NUM_EMBEDDED_LINES 1

/* From Jetson driver */
#define MIRA050_DEFAULT_LINE_LENGTH    (0xA80)
#define MIRA050_DEFAULT_PIXEL_CLOCK    (160)
#define MIRA050_DEFAULT_FRAME_LENGTH    (0x07C0)


enum pad_types {
	IMAGE_PAD,
	METADATA_PAD,
	NUM_PADS
};

struct mira050_reg {
	u16 address;
	u8 val;
};

struct mira050_reg_list {
	unsigned int num_of_regs;
	const struct mira050_reg *regs;
};

/* Mode : resolution and related config&values */
struct mira050_mode {
	/* Frame width */
	unsigned int width;
	/* Frame height */
	unsigned int height;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* Default register values */
	struct mira050_reg_list reg_list;

	u32 vblank;
	u32 hblank;
};

// 600_800_30fps_10b_2lanes
// Taken from ams_jetcis/scripts/Mira050/Mira050-bringup.py initSensor_2()
static const struct mira050_reg full_600_800_30fps_10b_2lanes_reg[] = {

    {0xE000, 0},
    {0xE1E4, 0},
    {0xE1E5, 19},
    {0xE1E2, 23},
    {0xE1E3, 136},
    {0xE1E6, 0},
    {0xE1E7, 202},
    {0xE16C, 1},
    {0xE16B, 1},
    {0xE16D, 50},
    {0xE31F, 0},
    {0xE320, 10},
    {0xE321, 4},
    {0xE322, 131},
    {0xE1A2, 0},
    {0xE1A3, 1},
    {0xE1A4, 4},
    {0xE1A5, 122},
    {0xE19E, 0},
    {0xE19F, 0},
    {0xE1A6, 0},
    {0xE1A7, 152},
    {0xE1A8, 5},
    {0xE1A9, 17},
    {0xE1A0, 0},
    {0xE1A1, 76},
    {0xE1B0, 0},
    {0xE1B1, 95},
    {0xE16E, 44},
    {0xE16F, 0},
    {0xE170, 0},
    {0xE171, 134},
    {0xE172, 0},
    {0xE173, 0},
    {0xE174, 0},
    {0xE175, 0},
    {0xE176, 0},
    {0xE177, 0},
    {0xE178, 0},
    {0xE179, 0},
    {0xE17A, 0},
    {0xE17B, 0},
    {0xE17C, 0},
    {0xE17D, 0},
    {0xE208, 1},
    {0xE209, 240},
    {0xE20A, 3},
    {0xE20B, 77},
    {0xE20C, 2},
    {0xE20D, 16},
    {0xE20E, 3},
    {0xE20F, 1},
    {0xE210, 0},
    {0xE211, 19},
    {0xE212, 0},
    {0xE213, 3},
    {0xE214, 3},
    {0xE215, 239},
    {0xE216, 0},
    {0xE217, 33},
    {0xE218, 0},
    {0xE219, 2},
    {0xE21A, 1},
    {0xE21B, 242},
    {0xE21C, 3},
    {0xE21D, 113},
    {0xE21E, 0},
    {0xE21F, 33},
    {0xE220, 3},
    {0xE221, 240},
    {0xE222, 3},
    {0xE223, 241},
    {0xE224, 3},
    {0xE225, 242},
    {0xE226, 0},
    {0xE227, 33},
    {0xE228, 0},
    {0xE229, 2},
    {0xE22A, 1},
    {0xE22B, 242},
    {0xE22C, 3},
    {0xE22D, 117},
    {0xE22E, 3},
    {0xE22F, 255},
    {0xE230, 3},
    {0xE231, 49},
    {0xE232, 2},
    {0xE233, 32},
    {0xE234, 3},
    {0xE235, 47},
    {0xE236, 0},
    {0xE237, 10},
    {0xE238, 2},
    {0xE239, 185},
    {0xE23A, 3},
    {0xE23B, 164},
    {0xE23C, 0},
    {0xE23D, 7},
    {0xE23E, 3},
    {0xE23F, 239},
    {0xE240, 3},
    {0xE241, 0},
    {0xE242, 0},
    {0xE243, 7},
    {0xE244, 0},
    {0xE245, 12},
    {0xE246, 2},
    {0xE247, 33},
    {0xE248, 3},
    {0xE249, 147},
    {0xE24A, 2},
    {0xE24B, 135},
    {0xE24C, 3},
    {0xE24D, 240},
    {0xE24E, 3},
    {0xE24F, 241},
    {0xE250, 3},
    {0xE251, 242},
    {0xE252, 3},
    {0xE253, 0},
    {0xE254, 2},
    {0xE255, 135},
    {0xE256, 0},
    {0xE257, 1},
    {0xE258, 3},
    {0xE259, 255},
    {0xE25A, 3},
    {0xE25B, 49},
    {0xE25C, 1},
    {0xE25D, 245},
    {0xE25E, 3},
    {0xE25F, 16},
    {0xE260, 0},
    {0xE261, 10},
    {0xE262, 2},
    {0xE263, 185},
    {0xE264, 3},
    {0xE265, 164},
    {0xE266, 0},
    {0xE267, 7},
    {0xE268, 3},
    {0xE269, 239},
    {0xE26A, 3},
    {0xE26B, 0},
    {0xE26C, 2},
    {0xE26D, 87},
    {0xE26E, 3},
    {0xE26F, 1},
    {0xE270, 1},
    {0xE271, 172},
    {0xE272, 1},
    {0xE273, 246},
    {0xE274, 3},
    {0xE275, 88},
    {0xE276, 2},
    {0xE278, 3},
    {0xE279, 240},
    {0xE27A, 3},
    {0xE27B, 241},
    {0xE27C, 3},
    {0xE27D, 242},
    {0xE27E, 3},
    {0xE27F, 0},
    {0xE280, 2},
    {0xE281, 103},
    {0xE282, 0},
    {0xE283, 8},
    {0xE284, 3},
    {0xE285, 255},
    {0xE286, 3},
    {0xE287, 0},
    {0xE288, 3},
    {0xE289, 255},
    {0xE28A, 2},
    {0xE28B, 135},
    {0xE28C, 3},
    {0xE28D, 2},
    {0xE28E, 2},
    {0xE28F, 54},
    {0xE290, 3},
    {0xE291, 2},
    {0xE292, 2},
    {0xE293, 64},
    {0xE294, 3},
    {0xE295, 0},
    {0xE296, 0},
    {0xE297, 5},
    {0xE298, 0},
    {0xE299, 2},
    {0xE29A, 1},
    {0xE29B, 241},
    {0xE29C, 3},
    {0xE29D, 3},
    {0xE29E, 0},
    {0xE29F, 18},
    {0xE2A0, 0},
    {0xE2A1, 55},
    {0xE2A2, 1},
    {0xE2A3, 247},
    {0xE2A4, 3},
    {0xE2A5, 3},
    {0xE2A6, 2},
    {0xE2A7, 64},
    {0xE2A8, 0},
    {0xE2A9, 5},
    {0xE2AA, 0},
    {0xE2AB, 1},
    {0xE2AC, 2},
    {0xE2AD, 54},
    {0xE2AE, 0},
    {0xE2AF, 39},
    {0xE2B0, 0},
    {0xE2B1, 8},
    {0xE2B2, 3},
    {0xE2B3, 255},
    {0xE2B4, 1},
    {0xE2B5, 248},
    {0xE2B6, 3},
    {0xE2B7, 21},
    {0xE2B8, 0},
    {0xE2B9, 23},
    {0xE2BA, 0},
    {0xE2BB, 8},
    {0xE2BC, 3},
    {0xE2BD, 255},
    {0xE2BE, 0},
    {0xE2BF, 56},
    {0xE2C0, 0},
    {0xE2C1, 23},
    {0xE2C2, 0},
    {0xE2C3, 8},
    {0xE2C4, 3},
    {0xE2C5, 255},
    {0xE2C6, 3},
    {0xE2C7, 255},
    {0xE2C8, 3},
    {0xE2C9, 255},
    {0xE2CA, 3},
    {0xE2CB, 255},
    {0xE2CC, 3},
    {0xE2CD, 255},
    {0xE2CE, 3},
    {0xE2CF, 255},
    {0xE2D0, 3},
    {0xE2D1, 255},
    {0xE2D2, 3},
    {0xE2D3, 255},
    {0xE2D4, 3},
    {0xE2D5, 255},
    {0xE2D6, 3},
    {0xE2D7, 255},
    {0xE2D8, 3},
    {0xE2D9, 255},
    {0xE2DA, 3},
    {0xE2DB, 255},
    {0xE2DC, 3},
    {0xE2DD, 255},
    {0xE2DE, 3},
    {0xE2DF, 255},
    {0xE2E0, 3},
    {0xE2E1, 255},
    {0xE2E2, 3},
    {0xE2E3, 255},
    {0xE2E4, 3},
    {0xE2E5, 255},
    {0xE2E6, 3},
    {0xE2E7, 255},
    {0xE2E8, 3},
    {0xE2E9, 255},
    {0xE2EA, 3},
    {0xE2EB, 255},
    {0xE2EC, 3},
    {0xE2ED, 255},
    {0xE2EE, 3},
    {0xE2EF, 255},
    {0xE2F0, 3},
    {0xE2F1, 255},
    {0xE2F2, 3},
    {0xE2F3, 255},
    {0xE2F4, 3},
    {0xE2F5, 255},
    {0xE2F6, 3},
    {0xE2F7, 255},
    {0xE2F8, 3},
    {0xE2F9, 255},
    {0xE2FA, 3},
    {0xE2FB, 255},
    {0xE2FC, 3},
    {0xE2FD, 255},
    {0xE2FE, 3},
    {0xE2FF, 255},
    {0xE300, 3},
    {0xE301, 255},
    {0xE302, 3},
    {0xE303, 255},
    {0xE1E9, 0},
    {0xE1E8, 20},
    {0xE1EA, 63},
    {0xE1EB, 65},
    {0xE1EC, 86},
    {0xE1ED, 91},
    {0x01EE, 10},
    {0x01EF, 140},
    {0x01F8, 15},
    {0x01D8, 1},
    {0x01DA, 1},
    {0x01DC, 1},
    {0x01DE, 1},
    {0x0189, 1},
    {0x01B7, 1},
    {0x01C1, 14},
    {0x01C2, 255},
    {0x01C3, 255},
    {0x01B8, 1},
    {0x01BA, 59},
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
    {0x01AC, 0},
    {0x01AD, 95},
    {0x209D, 0},
    {0x0063, 1},
    {0x2000, 0},
    {0x207C, 0},
    {0xE000, 0},
    {0x2077, 0},
    {0x2076, 222},
    {0x00CE, 2},
    {0x0070, 7},
    {0x016D, 40},
    {0x20C6, 0},
    {0x20C7, 0},
    {0x20C8, 1},
    {0x20C9, 0},
    {0x20CA, 0},
    {0x20CB, 1},
    {0x2075, 0},
    {0x2000, 0},
    {0x207C, 1},
    {0xE000, 0},
    {0xE0A0, 1},
    {0xE000, 0},
    {0xE0BD, 1},
    {0xE000, 0},
    {0xE1D9, 1},
    {0xE000, 0},
    {0xE1DB, 1},
    {0xE000, 0},
    {0xE1DD, 1},
    {0xE000, 0},
    {0xE1DF, 1},
    {0xE000, 0},
    {0xE060, 2},
    {0xE061, 170},
    {0xE000, 0},
    {0xE062, 2},
    {0xE000, 0},
    {0x207E, 0},
    {0x207F, 0},
    {0x2080, 0},
    {0x2081, 3},
    {0x2082, 0},
    {0x2083, 2},
    {0x0090, 1},
    {0x001E, 0},
    {0x2097, 0},
    {0x01B2, 0},
    {0x01B3, 100},
    {0xE000, 0},
    {0x0011, 3},
    {0x011D, 0},
    {0xE000, 0},
    {0x0012, 0},
    {0x0013, 38},
    {0x015A, 0},
    {0x015B, 27},
    {0x015C, 0},
    {0x015D, 27},
    {0x015E, 0},
    {0x015F, 27},
    {0x0162, 0},
    {0x0163, 3},
    {0x0164, 4},
    {0x0165, 88},
    {0x0166, 4},
    {0x0167, 88},
    {0xE000, 0},
    {0x005C, 0},
    {0x005D, 32},
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
    {0xE004, 0},
    {0xE000, 1},
    {0xE02C, 0},
    {0xE02D, 0},
    {0xE02E, 2},
    {0xE02F, 87},
    {0xE030, 0},
    {0xE025, 0},
    {0xE02A, 0},
    {0x2029, 70},
    {0x0034, 1},
    {0x0035, 44},
    {0xE004, 0},
    {0x001E, 0},
    {0x001F, 1},
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
    {0x0007, 5},
    {0xE004, 0},
    {0x0008, 0},
    {0x0009, 0},
    {0x000A, 97},
    {0x000B, 168},
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
    {0xE004, 0},
    {0xE000, 1},
    {0x001E, 0},
    {0x001F, 1},
    {0x002B, 0},
    {0xE004, 1},
    {0x001E, 0},
    {0x001F, 1},
    {0x002B, 0},
    {0xE000, 0},
    {0x001F, 0},
    {0x0020, 0},
    {0x0023, 0},
    {0x0024, 3},
    {0x0025, 32},
    {0x0026, 0},
    {0x0027, 8},
    {0x0028, 0},
    {0x0029, 0},
    {0x002A, 0},
    {0x002B, 0},
    {0x002C, 0},
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
    {0xE000, 0},
    {0xE0F3, 3},
    {0xE100, 34},
    {0xE0F4, 4},
    {0xE101, 35},
    {0xE0F5, 5},
    {0xE102, 13},
    {0xE0F6, 6},
    {0xE103, 12},
    {0xE004, 0},

};

static const char * const mira050_test_pattern_menu[] = {
	"Disabled",
	"Vertial Gradient",
};

static const int mira050_test_pattern_val[] = {
	MIRA050_TEST_PATTERN_DISABLE,
	MIRA050_TEST_PATTERN_VERTICAL_GRADIENT,
};


/* regulator supplies */
static const char * const mira050_supply_name[] = {
	// TODO(jalv): Check supply names
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.8V) supply */
	"VDDL",  /* IF (1.2V) supply */
};

#define MIRA050_NUM_SUPPLIES ARRAY_SIZE(mira050_supply_name)

/*
 * The supported formats. All flip/mirror combinations have the same byte order because the sensor
 * is monochrome
 */
static const u32 codes[] = {
	//MEDIA_BUS_FMT_Y8_1X8,
	//MEDIA_BUS_FMT_Y10_1X10,
	//MEDIA_BUS_FMT_Y12_1X12,
	MEDIA_BUS_FMT_SRGGB8_1X8,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SRGGB12_1X12,
};

/* Mode configs */
static const struct mira050_mode supported_modes[] = {
	{
		/* 2 MPx 30fps mode */
		.width = 600,
		.height = 800,
		.crop = {
			.left = MIRA050_PIXEL_ARRAY_LEFT,
			.top = MIRA050_PIXEL_ARRAY_TOP,
			.width = 600,
			.height = 800
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(full_600_800_30fps_10b_2lanes_reg),
			.regs = full_600_800_30fps_10b_2lanes_reg,
		},
		.vblank = 2866,
		.hblank = 0, // TODO
	},
};

struct mira050 {
	struct v4l2_subdev sd;
	struct media_pad pad[NUM_PADS];

	struct v4l2_mbus_framefmt fmt;

	struct clk *xclk; /* system clock to MIRA050 */
	u32 xclk_freq;

	//struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[MIRA050_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *gain;

	/* Current mode */
	const struct mira050_mode *mode;

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;

};

static inline struct mira050 *to_mira050(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct mira050, sd);
}

static int mira050_read(struct mira050 *mira050, u16 reg, u8 *val)
{
	int ret;
	unsigned char data_w[2] = { reg >> 8, reg & 0xff };
	struct i2c_client *client = v4l2_get_subdevdata(&mira050->sd);

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

static int mira050_write(struct mira050 *mira050, u16 reg, u8 val)
{
	int ret;
	unsigned char data[3] = { reg >> 8, reg & 0xff, val};
	struct i2c_client *client = v4l2_get_subdevdata(&mira050->sd);

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

static int mira050_write16(struct mira050 *mira050, u16 reg, u16 val)
{
       int ret;
       unsigned char data[4] = { reg >> 8, reg & 0xff, val & 0xff, val >> 8};
       struct i2c_client *client = v4l2_get_subdevdata(&mira050->sd);

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
static int mira050_write_regs(struct mira050 *mira050,
			     const struct mira050_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira050->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = mira050_write(mira050, regs[i].address, regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    regs[i].address, ret);

			return ret;
		} else {
			// Debug code below
			/*
			u8 val;
			ret = mira050_read(mira050, regs[i].address, &val);
			printk(KERN_INFO "[MIRA050]: Read reg 0x%4.4x, val = 0x%x.\n",
					regs[i].address, val);
			*/
		}
	}

	return 0;
}

// Returns the maximum exposure time in clock cycles (reg value)
static u32 mira050_calculate_max_exposure_time(u32 row_length, u32 vsize,
					       u32 vblank) {
  return row_length * (vsize + vblank) - MIRA050_GLOB_NUM_CLK_CYCLES;
}

static int mira050_write_analog_gain_reg(struct mira050 *mira050, u8 gain) {
	struct i2c_client* const client = v4l2_get_subdevdata(&mira050->sd);
	u32 reg_value;
	u32 ret;

	if ((gain < MIRA050_ANALOG_GAIN_MIN) || (gain > MIRA050_ANALOG_GAIN_MAX)) {
		return -EINVAL;
	}

	reg_value = 8 / gain;
	ret = mira050_write(mira050, MIRA050_ANALOG_GAIN_REG, reg_value);

	if (ret) {
		dev_err_ratelimited(&client->dev, "Error setting analog gain register to %d",
				reg_value);
	}

	return ret;
}

static int mira050_write_exposure_reg(struct mira050 *mira050, u32 exposure) {
	struct i2c_client* const client = v4l2_get_subdevdata(&mira050->sd);
	const u32 max_exposure = mira050_calculate_max_exposure_time(MIRA050_ROW_LENGTH_MIN,
		mira050->mode->height, mira050->mode->vblank);
	u32 ret = 0;

	if (exposure > max_exposure) {
		return -EINVAL;
	}

	ret = mira050_write16(mira050, MIRA050_EXP_TIME_LO_REG, exposure);
	if (ret) {
		dev_err_ratelimited(&client->dev, "Error setting exposure time to %d", exposure);
		return -EINVAL;
	}

	return 0;
}

static int mira050_write_start_streaming_regs(struct mira050* mira050) {
	struct i2c_client* const client = v4l2_get_subdevdata(&mira050->sd);
	int ret = 0;

	// Set conetxt bank 0 or 1
	ret = mira050_write(mira050, MIRA050_BANK_SEL_REG, 0);
	if (ret) {
		dev_err(&client->dev, "Error setting BANK_SEL_REG.");
		return ret;
	}

	// Set context bank 1A or bank 1B
	ret = mira050_write(mira050, MIRA050_RW_CONTEXT_REG, 0);
	if (ret) {
		dev_err(&client->dev, "Error setting RW_CONTEXT.");
		return ret;
	}


	// Raising CMD_REQ_1 to 1 for REQ_EXP
	ret = mira050_write(mira050, MIRA050_CMD_REQ_1_REG,
				1);
	if (ret) {
		dev_err(&client->dev, "Error setting CMD_REQ_1 to 1 for REQ_EXP.");
		return ret;
	}

	// Setting CMD_REQ_1 tp 0 for REQ_EXP
	ret = mira050_write(mira050, MIRA050_CMD_REQ_1_REG,
				0);
	if (ret) {
		dev_err(&client->dev, "Error setting CMD_REQ_1 to 0 for REQ_EXP.");
		return ret;
	}

	return ret;
}

static int mira050_write_stop_streaming_regs(struct mira050* mira050) {
	struct i2c_client* const client = v4l2_get_subdevdata(&mira050->sd);
	int ret = 0;
	u32 frame_time;

	// Set conetxt bank 0 or 1
	ret = mira050_write(mira050, MIRA050_BANK_SEL_REG, 0);
	if (ret) {
		dev_err(&client->dev, "Error setting BANK_SEL_REG.");
		return ret;
	}

	// Raising CMD_HALT_BLOCK to 1 to stop streaming
	ret = mira050_write(mira050, MIRA050_CMD_HALT_BLOCK_REG,
				1);
	if (ret) {
		dev_err(&client->dev, "Error setting CMD_HALT_BLOCK to 1.");
		return ret;
	}

	// Setting CMD_HALT_BLOCK to 0 to stop streaming
	ret = mira050_write(mira050, MIRA050_CMD_HALT_BLOCK_REG,
				0);
	if (ret) {
		dev_err(&client->dev, "Error setting CMD_HALT_BLOCK to 0.");
		return ret;
	}

        /*
         * Wait for one frame to make sure sensor is set to
         * software standby in V-blank
         *
         * frame_time = frame length rows * Tline
         * Tline = line length / pixel clock (in MHz)
         */
	/*
        frame_time = MIRA050_DEFAULT_FRAME_LENGTH *
            MIRA050_DEFAULT_LINE_LENGTH / MIRA050_DEFAULT_PIXEL_CLOCK;

        usleep_range(frame_time, frame_time + 1000);
	*/

	return ret;
}

// Gets the format code if supported. Otherwise returns the default format code `codes[0]`
static u32 mira050_validate_format_code_or_default(struct mira050 *mira050, u32 code)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira050->sd);
	unsigned int i;

	lockdep_assert_held(&mira050->mutex);

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

static void mira050_set_default_format(struct mira050 *mira050)
{
	struct v4l2_mbus_framefmt *fmt;

	fmt = &mira050->fmt;
	fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10; // MEDIA_BUS_FMT_Y12_1X12;
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

static int mira050_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct mira050 *mira050 = to_mira050(sd);
	struct v4l2_mbus_framefmt *try_fmt_img =
		v4l2_subdev_get_try_format(sd, fh->state, IMAGE_PAD);
	struct v4l2_mbus_framefmt *try_fmt_meta =
		v4l2_subdev_get_try_format(sd, fh->state, METADATA_PAD);
	struct v4l2_rect *try_crop;

	mutex_lock(&mira050->mutex);

	/* Initialize try_fmt for the image pad */
	try_fmt_img->width = supported_modes[0].width;
	try_fmt_img->height = supported_modes[0].height;
	try_fmt_img->code = mira050_validate_format_code_or_default(mira050,
						   MEDIA_BUS_FMT_SRGGB10_1X10);
	try_fmt_img->field = V4L2_FIELD_NONE;

	/* TODO(jalv): Initialize try_fmt for the embedded metadata pad */
	try_fmt_meta->width = MIRA050_EMBEDDED_LINE_WIDTH;
	try_fmt_meta->height = MIRA050_NUM_EMBEDDED_LINES;
	try_fmt_meta->code = MEDIA_BUS_FMT_SENSOR_DATA;
	try_fmt_meta->field = V4L2_FIELD_NONE;



	/* Initialize try_crop rectangle. */
	try_crop = v4l2_subdev_get_try_crop(sd, fh->state, 0);
	try_crop->top = MIRA050_PIXEL_ARRAY_TOP;
	try_crop->left = MIRA050_PIXEL_ARRAY_LEFT;
	try_crop->width = MIRA050_PIXEL_ARRAY_WIDTH;
	try_crop->height = MIRA050_PIXEL_ARRAY_HEIGHT;

	mutex_unlock(&mira050->mutex);

	return 0;
}

static int mira050_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mira050 *mira050 =
		container_of(ctrl->handler, struct mira050, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&mira050->sd);
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		int exposure_max, exposure_def;

		/* Update max exposure while meeting expected vblanking */
		exposure_max = mira050->mode->height + ctrl->val - 4;
		exposure_def = (exposure_max < MIRA050_DEFAULT_EXPOSURE) ?
			exposure_max : MIRA050_DEFAULT_EXPOSURE;
		__v4l2_ctrl_modify_range(mira050->exposure,
					 mira050->exposure->minimum,
					 exposure_max, mira050->exposure->step,
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
		ret = mira050_write_analog_gain_reg(mira050, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = mira050_write_exposure_reg(mira050, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = mira050_write(mira050, MIRA050_REG_TEST_PATTERN,
				       mira050_test_pattern_val[ctrl->val]);
		break;
	case V4L2_CID_HFLIP:
		ret = mira050_write(mira050, MIRA050_HFLIP_REG,
				        ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		ret = mira050_write(mira050, MIRA050_VFLIP_REG,
				        ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = mira050_write16(mira050, MIRA050_VBLANK_LO_REG,
				        mira050->mode->height + ctrl->val);
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

static const struct v4l2_ctrl_ops mira050_ctrl_ops = {
	.s_ctrl = mira050_set_ctrl,
};

// This function should enumerate all the media bus formats for the requested pads. If the requested
// format index is beyond the number of avaialble formats it shall return -EINVAL;
static int mira050_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct mira050 *mira050 = to_mira050(sd);

	if (code->pad >= NUM_PADS)
		return -EINVAL;

	if (code->pad == IMAGE_PAD) {
		if (code->index >= ARRAY_SIZE(codes))
			return -EINVAL;

		code->code = mira050_validate_format_code_or_default(mira050,
						    codes[code->index]);
	} else {
		if (code->index > 0)
			return -EINVAL;

		code->code = MEDIA_BUS_FMT_SENSOR_DATA;
	}

	return 0;
}

static int mira050_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct mira050 *mira050 = to_mira050(sd);

	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	if (fse->pad == IMAGE_PAD) {
		if (fse->index >= ARRAY_SIZE(supported_modes))
			return -EINVAL;

		if (fse->code != mira050_validate_format_code_or_default(mira050, fse->code))
			return -EINVAL;

		fse->min_width = supported_modes[fse->index].width;
		fse->max_width = fse->min_width;
		fse->min_height = supported_modes[fse->index].height;
		fse->max_height = fse->min_height;
	} else {
		if (fse->code != MEDIA_BUS_FMT_SENSOR_DATA || fse->index > 0)
			return -EINVAL;

		fse->min_width = MIRA050_EMBEDDED_LINE_WIDTH;
		fse->max_width = fse->min_width;
		fse->min_height = MIRA050_NUM_EMBEDDED_LINES;
		fse->max_height = fse->min_height;
	}

	return 0;
}

static void mira050_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void mira050_update_image_pad_format(struct mira050 *mira050,
					   const struct mira050_mode *mode,
					   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	mira050_reset_colorspace(&fmt->format);
}

static void mira050_update_metadata_pad_format(struct v4l2_subdev_format *fmt)
{
	fmt->format.width = MIRA050_EMBEDDED_LINE_WIDTH;
	fmt->format.height = MIRA050_NUM_EMBEDDED_LINES;
	fmt->format.code = MEDIA_BUS_FMT_SENSOR_DATA;
	fmt->format.field = V4L2_FIELD_NONE;

}

static int __mira050_get_pad_format(struct mira050 *mira050,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_format *fmt)
{
	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(&mira050->sd, sd_state, fmt->pad);

		try_fmt->code = fmt->pad == IMAGE_PAD ?
				mira050_validate_format_code_or_default(mira050, try_fmt->code) :
				MEDIA_BUS_FMT_SENSOR_DATA;
		fmt->format = *try_fmt;
	} else {
		if (fmt->pad == IMAGE_PAD) {
			mira050_update_image_pad_format(mira050, mira050->mode,
						       fmt);
			fmt->format.code = mira050_validate_format_code_or_default(mira050,
							      mira050->fmt.code);
		} else {
			mira050_update_metadata_pad_format(fmt);
		}
	}

	return 0;
}

static int mira050_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct mira050 *mira050 = to_mira050(sd);
	int ret;

	mutex_lock(&mira050->mutex);
	ret = __mira050_get_pad_format(mira050, sd_state, fmt);
	mutex_unlock(&mira050->mutex);

	return ret;
}

static int mira050_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mira050 *mira050 = to_mira050(sd);
	const struct mira050_mode *mode;
	struct v4l2_mbus_framefmt *framefmt;
	u32 max_exposure = 0, default_exp = 0;
	int rc = 0;

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&mira050->mutex);

	if (fmt->pad == IMAGE_PAD) {
		/* Validate format or use default */
		fmt->format.code = mira050_validate_format_code_or_default(mira050,
									  fmt->format.code);

		mode = v4l2_find_nearest_size(supported_modes,
					      ARRAY_SIZE(supported_modes),
					      width, height,
					      fmt->format.width,
					      fmt->format.height);
		mira050_update_image_pad_format(mira050, mode, fmt);
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else if (mira050->mode != mode ||
			mira050->fmt.code != fmt->format.code) {
			mira050->fmt = fmt->format;
			mira050->mode = mode;

			// Update controls based on new mode (range and current value).
			max_exposure = mira050_calculate_max_exposure_time(MIRA050_MIN_ROW_LENGTH,
									   mira050->mode->height,
									   mira050->mode->vblank);
			default_exp = MIRA050_DEFAULT_EXPOSURE > max_exposure ? max_exposure : MIRA050_DEFAULT_EXPOSURE;
			rc = v4l2_ctrl_modify_range(mira050->exposure,
							     MIRA050_EXPOSURE_MIN,
							     max_exposure, 1,
							     default_exp);
			if (rc) {
				dev_err(&client->dev, "Error setting exposure range");
			}

			// Set the current vblank value
			rc = v4l2_ctrl_s_ctrl(mira050->vblank, mira050->mode->vblank);
			if (rc) {
				dev_err(&client->dev, "Error setting vblank value to %u",
					mira050->mode->vblank);
			}
		}
	} else {
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else {
			/* Only one embedded data mode is supported */
			mira050_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&mira050->mutex);

	return 0;
}

static int mira050_set_framefmt(struct mira050 *mira050)
{
	switch (mira050->fmt.code) {
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		mira050_write(mira050, MIRA050_BIT_DEPTH_REG, MIRA050_BIT_DEPTH_8_BIT);
		mira050_write(mira050, MIRA050_CSI_DATA_TYPE_REG,
			MIRA050_CSI_DATA_TYPE_8_BIT);
		return 0;
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		mira050_write(mira050, MIRA050_BIT_DEPTH_REG,MIRA050_BIT_DEPTH_10_BIT);
		mira050_write(mira050, MIRA050_CSI_DATA_TYPE_REG,
			MIRA050_CSI_DATA_TYPE_10_BIT);
		return 0;
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		mira050_write(mira050, MIRA050_BIT_DEPTH_REG, MIRA050_BIT_DEPTH_12_BIT);
		mira050_write(mira050, MIRA050_CSI_DATA_TYPE_REG,
			MIRA050_CSI_DATA_TYPE_12_BIT);
		return 0;
	default:
		printk(KERN_ERR "Unknown format requested %d", mira050->fmt.code);
	}

	return -EINVAL;
}

static const struct v4l2_rect *
__mira050_get_pad_crop(struct mira050 *mira050, struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&mira050->sd, sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &mira050->mode->crop;
	}

	return NULL;
}

static int mira050_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct mira050 *mira050 = to_mira050(sd);

		mutex_lock(&mira050->mutex);
		sel->r = *__mira050_get_pad_crop(mira050, sd_state, sel->pad,
						sel->which);
		mutex_unlock(&mira050->mutex);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = MIRA050_NATIVE_WIDTH;
		sel->r.height = MIRA050_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = MIRA050_PIXEL_ARRAY_TOP;
		sel->r.left = MIRA050_PIXEL_ARRAY_LEFT;
		sel->r.width = MIRA050_PIXEL_ARRAY_WIDTH;
		sel->r.height = MIRA050_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

static int mira050_start_streaming(struct mira050 *mira050)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira050->sd);
	const struct mira050_reg_list *reg_list;
	int ret;

	printk(KERN_INFO "[MIRA050]: Entering start streaming function.\n");

	ret = pm_runtime_get_sync(&client->dev);
	if (ret < 0) {
		printk(KERN_INFO "[MIRA050]: get_sync failed, but continue.\n");
		pm_runtime_put_noidle(&client->dev);
		// return ret;
	}

	printk(KERN_INFO "[MIRA050]: Writing stop streaming regs.\n");

	//ret = mira050_write_stop_streaming_regs(mira050);
	//if (ret) {
	//	dev_err(&client->dev, "Could not write stream-on sequence");
	//	goto err_rpm_put;
	//}


	/* Apply default values of current mode */
	reg_list = &mira050->mode->reg_list;
	printk(KERN_INFO "[MIRA050]: Write %d regs.\n", reg_list->num_of_regs);
	ret = mira050_write_regs(mira050, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		goto err_rpm_put;
	}

	ret = mira050_set_framefmt(mira050);
	if (ret) {
		dev_err(&client->dev, "%s failed to set frame format: %d\n",
			__func__, ret);
		goto err_rpm_put;
	}

	printk(KERN_INFO "[MIRA050]: Entering v4l2 ctrl handler setup function.\n");

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(mira050->sd.ctrl_handler);
	printk(KERN_INFO "[MIRA050]: __v4l2_ctrl_handler_setup ret = %d.\n", ret);
	if (ret)
		goto err_rpm_put;

	printk(KERN_INFO "[MIRA050]: Writing start streaming regs.\n");

	ret = mira050_write_start_streaming_regs(mira050);
	if (ret) {
		dev_err(&client->dev, "Could not write stream-on sequence");
		goto err_rpm_put;
	}

	/* vflip and hflip cannot change during streaming */
	printk(KERN_INFO "[MIRA050]: Entering v4l2 ctrl grab vflip grab vflip.\n");
	__v4l2_ctrl_grab(mira050->vflip, true);
	printk(KERN_INFO "[MIRA050]: Entering v4l2 ctrl grab vflip grab hflip.\n");
	__v4l2_ctrl_grab(mira050->hflip, true);

	return 0;

err_rpm_put:
	pm_runtime_put(&client->dev);
	return ret;
}

static void mira050_stop_streaming(struct mira050 *mira050)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira050->sd);
	int ret = 0;

	ret = mira050_write_stop_streaming_regs(mira050);
	if (ret) {
		dev_err(&client->dev, "Could not write the stream-off sequence");
	}

	/* Unlock controls for vflip and hflip */
	__v4l2_ctrl_grab(mira050->vflip, false);
	__v4l2_ctrl_grab(mira050->hflip, false);

	pm_runtime_put(&client->dev);
}

static int mira050_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct mira050 *mira050 = to_mira050(sd);
	int ret = 0;

	mutex_lock(&mira050->mutex);
	if (mira050->streaming == enable) {
		mutex_unlock(&mira050->mutex);
		return 0;
	}

	printk(KERN_INFO "[MIRA050]: Entering mira050_set_stream enable: %d.\n", enable);

	if (enable) {
		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = mira050_start_streaming(mira050);
		if (ret)
			goto err_unlock;
	} else {
		mira050_stop_streaming(mira050);
	}

	mira050->streaming = enable;

	mutex_unlock(&mira050->mutex);

	printk(KERN_INFO "[MIRA050]: Returning mira050_set_stream with ret: %d.\n", ret);

	return ret;

err_unlock:
	mutex_unlock(&mira050->mutex);

	return ret;
}

/* Power/clock management functions */
static int mira050_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira050 *mira050 = to_mira050(sd);
	int ret = -EINVAL;

	ret = regulator_bulk_enable(MIRA050_NUM_SUPPLIES, mira050->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(mira050->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}

	// gpiod_set_value_cansleep(mira050->reset_gpio, 1);
	usleep_range(MIRA050_XCLR_MIN_DELAY_US,
		     MIRA050_XCLR_MIN_DELAY_US + MIRA050_XCLR_DELAY_RANGE_US);
	return 0;

reg_off:
	ret = regulator_bulk_disable(MIRA050_NUM_SUPPLIES, mira050->supplies);
	return ret;
}

static int mira050_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira050 *mira050 = to_mira050(sd);

	regulator_bulk_disable(MIRA050_NUM_SUPPLIES, mira050->supplies);
	clk_disable_unprepare(mira050->xclk);

	return 0;
}

static int __maybe_unused mira050_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira050 *mira050 = to_mira050(sd);

	if (mira050->streaming)
		mira050_stop_streaming(mira050);

	return 0;
}

static int __maybe_unused mira050_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira050 *mira050 = to_mira050(sd);
	int ret;

	if (mira050->streaming) {
		ret = mira050_start_streaming(mira050);
		if (ret)
			goto error;
	}

	return 0;

error:
	mira050_stop_streaming(mira050);
	mira050->streaming = false;

	return ret;
}

static int mira050_get_regulators(struct mira050 *mira050)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira050->sd);
	unsigned int i;

	for (i = 0; i < MIRA050_NUM_SUPPLIES; i++)
		mira050->supplies[i].supply = mira050_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       MIRA050_NUM_SUPPLIES,
				       mira050->supplies);
}

/* OTP power on */
static int mira050_otp_power_on(struct mira050 *mira050)
{
	int ret;

	ret = mira050_write(mira050, 0x0080, 0x04);

	return 0;
}

/* OTP power off */
static int mira050_otp_power_off(struct mira050 *mira050)
{
	int ret;

	ret = mira050_write(mira050, 0x0080, 0x08);

	return 0;
}

/* OTP power on */
static int mira050_otp_read(struct mira050 *mira050, u8 addr, u8 offset, u8 *val)
{
	int ret;

	ret = mira050_write(mira050, 0x0086, addr);
	ret = mira050_write(mira050, 0x0080, 0x02);
	ret = mira050_read(mira050, 0x0082 + offset, val);
	return 0;
}


/* Verify chip ID */
static int mira050_identify_module(struct mira050 *mira050)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira050->sd);
	int ret;
	u8 val;

	/*
	mira050_otp_power_on(mira050);
	usleep_range(100, 110);
	ret = mira050_otp_read(mira050, 0x0d, 0, &val);
	dev_err(&client->dev, "Read OTP add 0x0d with val %x\n", val);
	mira050_otp_power_off(mira050);
	*/

	ret = mira050_read(mira050, 0x25, &val);
	printk(KERN_INFO "[MIRA050]: Read reg 0x%4.4x, val = 0x%x.\n",
		      0x25, val);
	ret = mira050_read(mira050, 0x3, &val);
	printk(KERN_INFO "[MIRA050]: Read reg 0x%4.4x, val = 0x%x.\n",
		      0x3, val);
	ret = mira050_read(mira050, 0x4, &val);
	printk(KERN_INFO "[MIRA050]: Read reg 0x%4.4x, val = 0x%x.\n",
		      0x4, val);



	return 0;
}

static const struct v4l2_subdev_core_ops mira050_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops mira050_video_ops = {
	.s_stream = mira050_set_stream,
};

static const struct v4l2_subdev_pad_ops mira050_pad_ops = {
	.enum_mbus_code = mira050_enum_mbus_code,
	.get_fmt = mira050_get_pad_format,
	.set_fmt = mira050_set_pad_format,
	.get_selection = mira050_get_selection,
	.enum_frame_size = mira050_enum_frame_size,
};

static const struct v4l2_subdev_ops mira050_subdev_ops = {
	.core = &mira050_core_ops,
	.video = &mira050_video_ops,
	.pad = &mira050_pad_ops,
};

static const struct v4l2_subdev_internal_ops mira050_internal_ops = {
	.open = mira050_open,
};

/* Initialize control handlers */
static int mira050_init_controls(struct mira050 *mira050)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira050->sd);
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct v4l2_fwnode_device_properties props;
	int ret;
	u32 max_exposure = 0;

	ctrl_hdlr = &mira050->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 11);
	if (ret)
		return ret;

	mutex_init(&mira050->mutex);
	ctrl_hdlr->lock = &mira050->mutex;

	printk(KERN_INFO "[MIRA050]: %s V4L2_CID_PIXEL_RATE %X.\n", __func__, V4L2_CID_PIXEL_RATE);

	/* By default, PIXEL_RATE is read only */
	mira050->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &mira050_ctrl_ops,
					        V4L2_CID_PIXEL_RATE,
					        MIRA050_PIXEL_RATE,
					        MIRA050_PIXEL_RATE, 1,
					        MIRA050_PIXEL_RATE);

	printk(KERN_INFO "[MIRA050]: %s V4L2_CID_VBLANK %X.\n", __func__, V4L2_CID_VBLANK);

	mira050->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &mira050_ctrl_ops,
					   V4L2_CID_VBLANK, MIRA050_MIN_VBLANK,
					   0xFFFF, 1,
					   mira050->mode->vblank);

	printk(KERN_INFO "[MIRA050]: %s V4L2_CID_HBLANK %X.\n", __func__, V4L2_CID_HBLANK);

	mira050->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &mira050_ctrl_ops,
					   V4L2_CID_HBLANK, 0x0000,
					   0xFFFF, 1,
					   mira050->mode->hblank);
	if (mira050->hblank)
		mira050->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;


	// Make the vblank control read only. This could be changed to allow changing framerate in
	// runtime, but would require adapting other settings
	// mira050->vblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	// Exposure is indicated in number of lines here
	// Max is determined by vblank + vsize and Tglob.
	max_exposure = mira050_calculate_max_exposure_time(MIRA050_MIN_ROW_LENGTH,
	                                                   mira050->mode->height,
	                                                   mira050->mode->vblank);

	printk(KERN_INFO "[MIRA050]: %s V4L2_CID_EXPOSURE %X.\n", __func__, V4L2_CID_EXPOSURE);

	mira050->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &mira050_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     MIRA050_EXPOSURE_MIN, max_exposure,
					     1,
					     MIRA050_DEFAULT_EXPOSURE);

	printk(KERN_INFO "[MIRA050]: %s V4L2_CID_ANALOGUE_GAIN %X.\n", __func__, V4L2_CID_ANALOGUE_GAIN);

	mira050->gain = v4l2_ctrl_new_std(ctrl_hdlr, &mira050_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  MIRA050_ANALOG_GAIN_MIN, MIRA050_ANALOG_GAIN_MAX,
			  MIRA050_ANALOG_GAIN_STEP, MIRA050_ANALOG_GAIN_DEFAULT);

	printk(KERN_INFO "[MIRA050]: %s V4L2_CID_HFLIP %X.\n", __func__, V4L2_CID_HFLIP);

	mira050->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &mira050_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (mira050->hflip)
		mira050->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	printk(KERN_INFO "[MIRA050]: %s V4L2_CID_VFLIP %X.\n", __func__, V4L2_CID_VFLIP);

	mira050->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &mira050_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (mira050->vflip)
		mira050->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	printk(KERN_INFO "[MIRA050]: %s V4L2_CID_TEST_PATTERN %X.\n", __func__, V4L2_CID_TEST_PATTERN);
	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &mira050_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(mira050_test_pattern_menu) - 1,
				     0, 0, mira050_test_pattern_menu);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &mira050_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	mira050->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&mira050->mutex);

	return ret;
}

static void mira050_free_controls(struct mira050 *mira050)
{
	v4l2_ctrl_handler_free(mira050->sd.ctrl_handler);
	mutex_destroy(&mira050->mutex);
}

static int mira050_check_hwcfg(struct device *dev)
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
	    ep_cfg.link_frequencies[0] != MIRA050_DEFAULT_LINK_FREQ) {
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

static int mira050_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct mira050 *mira050;
	int ret;

	printk(KERN_INFO "[MIRA050]: probing v4l2 sensor.\n");
	printk(KERN_INFO "[MIRA050]: Driver Version 0.0.\n");

	dev_err(dev, "[MIRA050] name: %s.\n", client->name);
	dev_err(dev, "[MIRA050] Sleep for 1 second to let PMIC driver complete init.\n");
	usleep_range(1000000, 1000000+100);

	mira050 = devm_kzalloc(&client->dev, sizeof(*mira050), GFP_KERNEL);
	if (!mira050)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&mira050->sd, client, &mira050_subdev_ops);

	/* Check the hardware configuration in device tree */
	if (mira050_check_hwcfg(dev))
		return -EINVAL;

	// TODO(jalv): Get GPIO's, regulators and clocks from dts

	/* Get system clock (xclk) */
	mira050->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(mira050->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(mira050->xclk);
	}

	mira050->xclk_freq = clk_get_rate(mira050->xclk);
	if (mira050->xclk_freq != MIRA050_SUPPORTED_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			mira050->xclk_freq);
		return -EINVAL;
	}

	ret = mira050_get_regulators(mira050);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	/* Request optional enable pin */
	// mira050->reset_gpio = devm_gpiod_get_optional(dev, "reset",
	//					     GPIOD_OUT_HIGH);


	printk(KERN_INFO "[MIRA050]: Entering power on function.\n");

	/*
	 * The sensor must be powered for mira050_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = mira050_power_on(dev);
	if (ret)
		return ret;

	printk(KERN_INFO "[MIRA050]: Entering identify function.\n");

	ret = mira050_identify_module(mira050);
	if (ret)
		goto error_power_off;

	printk(KERN_INFO "[MIRA050]: Setting support function.\n");

	/* Set default mode to max resolution */
	mira050->mode = &supported_modes[0];

	printk(KERN_INFO "[MIRA050]: Entering init controls function.\n");

	ret = mira050_init_controls(mira050);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	mira050->sd.internal_ops = &mira050_internal_ops;
	mira050->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	mira050->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	mira050->pad[IMAGE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	mira050->pad[METADATA_PAD].flags = MEDIA_PAD_FL_SOURCE;

	printk(KERN_INFO "[MIRA050]: Entering set default format function.\n");

	/* Initialize default format */
	mira050_set_default_format(mira050);

	printk(KERN_INFO "[MIRA050]: Entering pads init function.\n");

	ret = media_entity_pads_init(&mira050->sd.entity, NUM_PADS, mira050->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	printk(KERN_INFO "[MIRA050]: Entering subdev sensor common function.\n");

	ret = v4l2_async_register_subdev_sensor(&mira050->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	/* For debug purpose */
	// mira050_start_streaming(mira050);

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

error_media_entity:
	media_entity_cleanup(&mira050->sd.entity);

error_handler_free:
	mira050_free_controls(mira050);

error_power_off:
	mira050_power_off(dev);

	return ret;
}

static int mira050_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira050 *mira050 = to_mira050(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	mira050_free_controls(mira050);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		mira050_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

static const struct of_device_id mira050_dt_ids[] = {
	{ .compatible = "ams,mira050" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mira050_dt_ids);

static const struct i2c_device_id mira050_ids[] = {
	{ "mira050", 1 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mira050_ids);


static const struct dev_pm_ops mira050_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mira050_suspend, mira050_resume)
	SET_RUNTIME_PM_OPS(mira050_power_off, mira050_power_on, NULL)
};

static struct i2c_driver mira050_i2c_driver = {
	.driver = {
		.name = "mira050",
		.of_match_table	= mira050_dt_ids,
		.pm = &mira050_pm_ops,
	},
	.probe_new = mira050_probe,
	.remove = mira050_remove,
	.id_table = mira050_ids,
};

module_i2c_driver(mira050_i2c_driver);

MODULE_AUTHOR("Zhenyu Ye <zhenyu.ye@ams-osram.com>");
MODULE_DESCRIPTION("ams MIRA050 sensor driver");
MODULE_LICENSE("GPL v2");
