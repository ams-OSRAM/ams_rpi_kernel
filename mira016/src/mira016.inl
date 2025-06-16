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

#include "mira016_registers.inl"

#define AMS_CAMERA_CID_BASE (V4L2_CTRL_CLASS_CAMERA | 0x2000)
#define AMS_CAMERA_CID_MIRA_REG_W (AMS_CAMERA_CID_BASE + 0)
#define AMS_CAMERA_CID_MIRA_REG_R (AMS_CAMERA_CID_BASE + 1)

/* Most significant Byte is flag, and most significant bit is unused. */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_FOR_READ 0b00000001
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_USE_BANK 0b00000010
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_BANK 0b00000100
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_CONTEXT 0b00001000
/* Use bit 5 to indicate special command, bit 1,2,3,4 for command. */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_CMD_SEL 0b00010000
/* Special command for sleep. The other 3 Bytes (addr+val) is sleep values in us. */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_SLEEP_US 0b00010000
/* Special command to enable power on (/off) when stream on (/off). */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_RESET_ON 0b00010010
/* Special command to disable power on (/off) when stream on (/off). */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_RESET_OFF 0b00010100
/* Special command to enable base register sequence upload, overwrite skip-reg-upload in dtoverlay */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_REG_UP_ON 0b00010110
/* Special command to disable base register sequence upload, overwrite skip-reg-upload in dtoverlay */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_REG_UP_OFF 0b00011000
/* Special command to manually power on */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_POWER_ON 0b00011010
/* Special command to manually power off */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_POWER_OFF 0b00011100
/* Special command to turn illumination trigger on */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_ILLUM_TRIG_ON 0b00011110
/* Special command to turn illumination trigger off */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_ILLUM_TRIG_OFF 0b00010001
/* Special command to set ILLUM_WIDTH. The other 3 Bytes (addr+val) is width value. */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_ILLUM_WIDTH 0b00010011
/* Special command to set ILLUM_DELAY. The other 3 Bytes (addr+val) is width value. */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_ILLUM_DELAY 0b00010101
/* Special command to enable ILLUM_WIDTH automatically tracking exposure time */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_ILLUM_EXP_T_ON 0b00010111
/* Special command to disable ILLUM_WIDTH automatically tracking exposure time */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_ILLUM_EXP_T_OFF 0b00011001
/* Special command to enable force_stream_ctrl */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_STREAM_CTRL_ON 0b00011011
/* Special command to disable force_stream_ctrl */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_STREAM_CTRL_OFF 0b00011101

/*
 * Bit 6&7 of flag are combined to specify I2C dev (default is Mira).
 * If bit 6&7 is 0b01, the reg_addr and reg_val are for a TBD I2C address.
 * The TBD I2C address is default to MIRA016LED_I2C_ADDR.
 * To change the TBD I2C address, set bit 6&7 to 0b10,
 * then the reg_val will become TBD I2C address.
 * The TBD I2C address is stored in mira016->tbd_client_i2c_addr.
 */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_SEL 0b01100000
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_MIRA 0b00000000
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_TBD 0b00100000
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_SET_TBD 0b01000000

/* Pre-allocated i2c_client */
#define MIRA016PMIC_I2C_ADDR 0x2D
#define MIRA016UC_I2C_ADDR 0x0A
#define MIRA016LED_I2C_ADDR 0x53

#define MIRA016_NATIVE_WIDTH 400U
#define MIRA016_NATIVE_HEIGHT 400U

#define MIRA016_PIXEL_ARRAY_LEFT 0U
#define MIRA016_PIXEL_ARRAY_TOP 0U
#define MIRA016_PIXEL_ARRAY_WIDTH 400U
#define MIRA016_PIXEL_ARRAY_HEIGHT 400U

#define MIRA016_ANALOG_GAIN_MIN 0
#define MIRA016_ANALOG_GAIN_STEP 1
#define MIRA016_ANALOG_GAIN_DEFAULT MIRA016_ANALOG_GAIN_MIN

#define MIRA016_BANK_SEL_REG 0xE000
#define MIRA016_RW_CONTEXT_REG 0xE004
#define MIRA016_CMD_REQ_1_REG 0x000A
#define MIRA016_CMD_HALT_BLOCK_REG 0x000C

// Exposure time is indicated in us
#define MIRA016_EXP_TIME_L_REG 0x000E
#define MIRA016_EXP_TIME_S_REG 0x0012
// Target frame time is indicated in us
#define MIRA016_TARGET_FRAME_TIME_REG 0x0008

#define MIRA016_SUPPORTED_XCLK_FREQ 24000000
#define MIRA016_XCLR_MIN_DELAY_US 150000
#define MIRA016_XCLR_DELAY_RANGE_US 3000

/* Embedded metadata stream structure */
#define MIRA016_EMBEDDED_LINE_WIDTH 16384
#define MIRA016_NUM_EMBEDDED_LINES 1


#define MIRA016_GDIG_PREAMP 0x0024
#define MIRA016_BIAS_RG_ADCGAIN 0x01F0
#define MIRA016_BIAS_RG_MULT 0x01F3

#define MIRA016_OTP_COMMAND 0x0066
#define MIRA016_OTP_ADDR 0x0067
#define MIRA016_OTP_START 0x0064
#define MIRA016_OTP_BUSY 0x0065
#define MIRA016_OTP_DOUT 0x006C
#define MIRA016_OTP_CAL_VALUE_DEFAULT 2250
#define MIRA016_OTP_CAL_FINE_VALUE_DEFAULT 35
#define MIRA016_OTP_CAL_FINE_VALUE_MIN 1
#define MIRA016_OTP_CAL_FINE_VALUE_MAX 60 // TODO


#define MIRA016_DEFAULT_LINE_LENGTH (2) //  (HSIZE+HBLANK)  / pixel rate

// Some timings
#define MIRA016_DATA_RATE 1000 // Mbit/s
#define MIRA016_SEQ_TIME_BASE 8 / MIRA016_DATA_RATE
#define MIRA016_LPS_CYCLE_TIME 1145
#define MIRA016_GLOB_TIME 68
#define MIRA016_ROW_LENGTH 1504 // 12b gain 1x TODO fix it for other modes
#define MIRA016_LPS_DISABLED 0
#define MIRA016_TROW_US MIRA016_ROW_LENGTH * 8 / MIRA016_DATA_RATE

#define MIRA016_READOUT_TIME MIRA016_TROW_US * (11 + MIRA016_PIXEL_ARRAY_HEIGHT)

// Default exposure is adjusted to 1 ms
#define MIRA016_GRAN_TG MIRA016_DATA_RATE * 50 / 1500  // 33
#define MIRA016_LUT_DEL_008 0

#define MIRA016_MIN_ROW_LENGTH MIRA016_ROW_LENGTH // 1042 for 8 bit
#define MIRA016_MIN_ROW_LENGTH_US (MIRA016_MIN_ROW_LENGTH * 8 / MIRA016_DATA_RATE)
#define MIRA016_EXPOSURE_MIN_US (int)(1 + (151 + MIRA016_LUT_DEL_008) * MIRA016_GRAN_TG * MIRA016_SEQ_TIME_BASE)
#define MIRA016_EXPOSURE_MAX_US (1000000)
#define MIRA016_EXPOSURE_MIN_LINES (MIRA016_EXPOSURE_MIN_US / MIRA016_DEFAULT_LINE_LENGTH)
#define MIRA016_EXPOSURE_MAX_LINES (MIRA016_EXPOSURE_MAX_US / MIRA016_DEFAULT_LINE_LENGTH)


#define MIRA016_DEFAULT_EXPOSURE_LINES 1000
#define MIRA016_DEFAULT_EXPOSURE_US MIRA016_DEFAULT_EXPOSURE_LINES *MIRA016_DEFAULT_LINE_LENGTH

// Default exposure for V4L2 is in row time

// #define MIRA016_MIN_VBLANK 11 // for 10b or 8b, 360fps
#define MIRA016_MIN_VBLANK_60 8000 // 200 fps
#define MIRA016_MIN_VBLANK_200 2100 // 200 fps
#define MIRA016_MIN_VBLANK_360 1000 // 200 fps
#define MIRA016_MAX_VBLANK 500000

#define MIRA016_DEFAULT_VBLANK_60 8000 // 200 fps
#define MIRA016_HBLANK 0

#define MIRA016_DEFAULT_LINK_FREQ 750000000
#define MIRA016_PIXEL_RATE (200000000) /*reduce factor 2 because max isp pixel rate is 380Mpix/s*/


// pixel_rate = link_freq * 2 * nr_of_lanes / bits_per_sample
// 0.9Gb/s * 2 * 1 / 12 = 157286400
// 1.5 Gbit/s * 2 * 1 / 12 = 250 000 000
/* Should match device tree link freq */
// #define MIRA016_DEFAULT_LINK_FREQ 750000000


/* Illumination trigger */
#define MIRA016_EN_TRIG_SYNC 0x001D		  // bank 1
#define MIRA016_TRIG_SYNC_DELAY 0x001A	  // bank 0
#define MIRA016_DMUX0_SEL 0x00F3		  // bank 0
#define MIRA016_TRIG_SYNC_ON_REQ_1 0x001D // bank 0
/* Illumination trigger */
#define MIRA016_EN_TRIG_ILLUM 0x001C
#define MIRA016_ILLUM_WIDTH_REG 0x0019
#define MIRA016_ILLUM_DELAY_REG 0x0016
#define MIRA016_ILLUM_WIDTH_DEFAULT (MIRA016_DEFAULT_EXPOSURE_US * MIRA016_DATA_RATE / 8)
#define MIRA016_ILLUM_DELAY_DEFAULT (1 << 19)
#define MIRA016_ILLUM_ENABLE_DEFAULT 1
#define MIRA016_ILLUM_SYNC_DEFAULT 1

/* 
 * EOB target = Electro Optical Black Level target
 * From the user guide:
 * The EOB target is the value that the sensor will try to achieve in the
 * EOB region aka under shielded pixels.
 * The EOB target is set in the register 0x005C.
 * ADC CONFIG | EOB TARGET | EFFECTIVE BLACK LEVEL
 * 8 bit fine gain | 6 | 6
 * 10 bit fine gain | 24 | 24
 * 12 bit coarse gain | 96 | 96

 */

#define MIRA016_EOB_TARGET_8BIT 6
#define MIRA016_EOB_TARGET_10BIT 24
#define MIRA016_EOB_TARGET_12BIT 96

#define MIRA016_YWIN_DIR_REG 0x0023 // YWIN direction register
#define MIRA016_YWIN_START_REG 0x002B // YWIN start register
#define MIRA016_XMIRROR_REG 0xe030 // YWIN start register

enum pad_types
{
	IMAGE_PAD,
	METADATA_PAD,
	NUM_PADS
};




/* Mode : resolution and related config&values */
struct mira016_mode
{
	/* Frame width */
	unsigned int width;
	/* Frame height */
	unsigned int height;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* Default register values */
	struct mira016_reg_list reg_list_pre_soft_reset;
	struct mira016_reg_list reg_list_post_soft_reset;
	u32 gain_min;
	u32 gain_max;
	u32 min_vblank;
	u32 max_vblank;
	u32 hblank;
	u32 row_length;

	/* Format code */
	u32 code;

	/* bit_depth needed for analog gain selection */
	u8 bit_depth;
};





/* regulator supplies */
static const char *const mira016_supply_name[] = {
	// TODO(jalv): Check supply names
	/* Supplies can be enabled in any order */
	"VANA", /* Analog (2.8V) supply */
	"VDIG", /* Digital Core (1.8V) supply */
	"VDDL", /* IF (1.2V) supply */
};

#define MIRA016_NUM_SUPPLIES ARRAY_SIZE(mira016_supply_name)

/*
 * The supported formats. All flip/mirror combinations have the same byte order because the sensor
 * is monochrome
 */
static const u32 codes[] = {
	// MEDIA_BUS_FMT_Y8_1X8,
	// MEDIA_BUS_FMT_Y10_1X10,
	// MEDIA_BUS_FMT_Y12_1X12,
	MEDIA_BUS_FMT_SGRBG8_1X8,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGRBG12_1X12,
};

/* Mode configs */
/*
 * Only one mode is exposed to the public (576x768 at 12 bit).
 * Three codes (8/10/12 bit) are exposed to public.
 * The public user specifies the code.
 * That is used to specify which internal supported_mode to use.
 */
#define MIRA016_SUPPORTED_MODE_SIZE_PUBLIC 1
static const struct mira016_mode supported_modes[] = {
	{
		/* 12 bit mode */
		.width = 400,
		.height = 400,
		.crop = {
			.left = MIRA016_PIXEL_ARRAY_LEFT,
			.top = MIRA016_PIXEL_ARRAY_TOP,
			.width = 400,
			.height = 400},
		.reg_list_pre_soft_reset = {
			.num_of_regs = ARRAY_SIZE(full_400_400_100fps_12b_1lane_reg_pre_soft_reset),
			.regs = full_400_400_100fps_12b_1lane_reg_pre_soft_reset,
		},
		.reg_list_post_soft_reset = {
			.num_of_regs = ARRAY_SIZE(full_400_400_100fps_12b_1lane_reg_post_soft_reset),
			.regs = full_400_400_100fps_12b_1lane_reg_post_soft_reset,
		},
		.min_vblank = MIRA016_MIN_VBLANK_60,
		.max_vblank = MIRA016_MAX_VBLANK,
		.hblank = MIRA016_HBLANK, // TODO
		.bit_depth = 12,
		.code = MEDIA_BUS_FMT_SGRBG12_1X12,
		.gain_min = 0,
		.gain_max = 1, // this is means 0,1,2 correspond to 1x 2x 4x gain
	},
	{
		/* 10 bit highspeed / low power mode */
		.width = 400,
		.height = 400,
		.crop = {.left = MIRA016_PIXEL_ARRAY_LEFT, .top = MIRA016_PIXEL_ARRAY_TOP,
		.width = 400, .height = 400},
		.reg_list_pre_soft_reset = {
			.num_of_regs = ARRAY_SIZE(full_400_400_100fps_10b_1lane_reg_pre_soft_reset),
			.regs = full_400_400_100fps_10b_1lane_reg_pre_soft_reset,
		},
		.reg_list_post_soft_reset = {
			.num_of_regs = ARRAY_SIZE(full_400_400_100fps_10b_1lane_reg_post_soft_reset),
			.regs = full_400_400_100fps_10b_1lane_reg_post_soft_reset,
		},
		.min_vblank = MIRA016_MIN_VBLANK_60,
		.max_vblank = MIRA016_MAX_VBLANK,
		.hblank = MIRA016_HBLANK, // TODO
		.bit_depth = 10,
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.gain_min = 0,
		.gain_max = ARRAY_SIZE(fine_gain_lut_10bit_hs_4x) - 1,
	},
	{
		/* 8 bit mode */
		.width = 400,
		.height = 400,
		.crop = {.left = MIRA016_PIXEL_ARRAY_LEFT, .top = MIRA016_PIXEL_ARRAY_TOP, 
		.width = 400, .height = 400},
		.reg_list_pre_soft_reset = {
			.num_of_regs = ARRAY_SIZE(full_400_400_100fps_8b_1lane_reg_pre_soft_reset),
			.regs = full_400_400_100fps_8b_1lane_reg_pre_soft_reset,
		},
		.reg_list_post_soft_reset = {
			.num_of_regs = ARRAY_SIZE(full_400_400_100fps_8b_1lane_reg_post_soft_reset),
			.regs = full_400_400_100fps_8b_1lane_reg_post_soft_reset,
		},
		.min_vblank = MIRA016_MIN_VBLANK_60,
		.max_vblank = MIRA016_MAX_VBLANK,
		.hblank = MIRA016_HBLANK, // TODO
		.bit_depth = 8,
		.code = MEDIA_BUS_FMT_SGRBG8_1X8,
		.gain_min = 0,
		.gain_max = ARRAY_SIZE(fine_gain_lut_8bit_16x) - 1,
	},

};

struct mira016
{
	struct v4l2_subdev sd;
	struct media_pad pad[NUM_PADS];

	struct v4l2_mbus_framefmt fmt;

	struct clk *xclk; /* system clock to MIRA016 */
	u32 xclk_freq;

	// struct gpio_desc *reset_gpio;
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

	// u16 otp_dark_cal_8bit;
	// u16 otp_dark_cal_10bit_hs;
	// u16 otp_dark_cal_10bit;
	// u16 otp_dark_cal_12bit;

	/* Whether to skip base register sequence upload */
	u32 skip_reg_upload;
	/* Whether to reset sensor when stream on/off */
	u32 skip_reset;
	/* Whether regulator and clk are powered on */
	u32 powered;
	/* Illumination trigger enable */
	u8 illum_enable;
	/* Illumination trigger width. Use [23:0] for 24-bit register. */
	u32 illum_width;
	/* Illumination trigger delay. Use [19:0] for 20-bit register */
	u32 illum_delay;
	/* Illumination trigger width automatically set to exposure time */
	u8 illum_width_auto;
	/* A flag to force write_start/stop_streaming_regs even if (skip_reg_upload==1) */
	u8 force_stream_ctrl;
	u32 target_frame_time_us;
	u32 row_length;
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

static inline struct mira016 *to_mira016(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct mira016, sd);
}

static int mira016_read(struct mira016 *mira016, u16 reg, u8 *val)
{
	int ret;
	unsigned char data_w[2] = {reg >> 8, reg & 0xff};
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);

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

static int mira016_write(struct mira016 *mira016, u16 reg, u8 val)
{
	int ret;
	unsigned char data[3] = {reg >> 8, reg & 0xff, val};
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);

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
static int mira016_write_be16(struct mira016 *mira016, u16 reg, u16 val)
{
	int ret;
	unsigned char data[4] = {reg >> 8, reg & 0xff, (val >> 8) & 0xff, val & 0xff};
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);

	ret = i2c_master_send(client, data, 4);
	/*
	 * Writing the wrong number of bytes also needs to be flagged as an
	 * error. Success needs to produce a 0 return code.
	 */
	if (ret == 4)
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
 * mira016 is big-endian: msb of val goes to lower reg addr
 */
static int mira016_write_be24(struct mira016 *mira016, u16 reg, u32 val)
{
	int ret;
	unsigned char data[5] = {reg >> 8, reg & 0xff, (val >> 16) & 0xff, (val >> 8) & 0xff, val & 0xff};
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);

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
 * mira016 is big-endian: msb of val goes to lower reg addr
 */
static int mira016_write_be32(struct mira016 *mira016, u16 reg, u32 val)
{
	int ret;
	unsigned char data[6] = {reg >> 8, reg & 0xff, (val >> 24) & 0xff, (val >> 16) & 0xff, (val >> 8) & 0xff, val & 0xff};
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);

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
 * mira016 OTP 32-bit val on I2C is big-endian. However, val content can be little-endian.
 */
static int mira016_read_be32(struct mira016 *mira016, u16 reg, u32 *val)
{
	int ret;
	unsigned char data_w[2] = {reg >> 8, reg & 0xff};
	/* Big-endian 32-bit buffer. */
	unsigned char data_r[4];
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);

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
static int mira016_write_regs(struct mira016 *mira016,
							  const struct mira016_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++)
	{
		ret = mira016_write(mira016, regs[i].address, regs[i].val);
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
	usleep_range(15, 50);
	mira016_write(mira016, MIRA016_OTP_START, 0);
	for (poll_cnt = 0; poll_cnt < poll_cnt_max; poll_cnt++)
	{
		mira016_read(mira016, MIRA016_OTP_BUSY, &busy_status);
		if (busy_status == 0)
		{
			break;
		}
		else
		{
			usleep_range(5, 10);
		}
	}
	if (poll_cnt < poll_cnt_max && busy_status == 0)
	{
		usleep_range(15, 50);
		ret = mira016_read_be32(mira016, MIRA016_OTP_DOUT, val);
		printk(KERN_INFO "[MIRA016]: Read OTP 0x%x, val = 0x%x.\n",
		 		addr,*val);
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
static int mira016pmic_write(struct i2c_client *client, u8 reg, u8 val)
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

static int mira016pmic_read(struct i2c_client *client, u8 reg, u8 *val)
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
static int mira016_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira016 *mira016 = to_mira016(sd);
	int ret = -EINVAL;

	printk(KERN_INFO "[MIRA016]: Entering power on function.\n");

	if (mira016->powered == 0)
	{
		ret = regulator_bulk_enable(MIRA016_NUM_SUPPLIES, mira016->supplies);
		if (ret)
		{
			dev_err(&client->dev, "%s: failed to enable regulators\n",
					__func__);
			return ret;
		}

		ret = clk_prepare_enable(mira016->xclk);
		if (ret)
		{
			dev_err(&client->dev, "%s: failed to enable clock\n",
					__func__);
			goto reg_off;
		}

		usleep_range(MIRA016_XCLR_MIN_DELAY_US,
					 MIRA016_XCLR_MIN_DELAY_US + MIRA016_XCLR_DELAY_RANGE_US);
		mira016->powered = 1;
	}
	else
	{
		printk(KERN_INFO "[MIRA016]: Skip regulator and clk enable, because mira015->powered == %d.\n", mira016->powered);
	}
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

	if (mira016->skip_reset == 0)
	{
		if (mira016->powered == 1)
		{
			regulator_bulk_disable(MIRA016_NUM_SUPPLIES, mira016->supplies);
			clk_disable_unprepare(mira016->xclk);
			mira016->powered = 0;
		}
		else
		{
			printk(KERN_INFO "[MIRA016]: Skip disabling regulator and clk due to mira015->powered == %d.\n", mira016->powered);
		}
	}
	else
	{
		printk(KERN_INFO "[MIRA016]: Skip disabling regulator and clk due to mira016->skip_reset=%u.\n", mira016->skip_reset);
	}

	return 0;
}


static int mira016_write_illum_trig_regs(struct mira016 *mira016)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&mira016->sd);
	int ret = 0;
	u32 lps_time = 0;
	u32 width_adjust = 0;

	// Set context bank 1A or bank 1B
	ret = mira016_write(mira016, MIRA016_RW_CONTEXT_REG, 0);
	if (ret)
	{
		dev_err(&client->dev, "Error setting RW_CONTEXT.");
		return ret;
	}

	// Set conetxt bank 0 or 1
	ret = mira016_write(mira016, MIRA016_BANK_SEL_REG, 1);
	if (ret)
	{
		dev_err(&client->dev, "Error setting BANK_SEL_REG.");
		return ret;
	}

	// Enable or disable illumination trigger
	printk(KERN_INFO "[MIRA016]: Writing EN_TRIG_ILLUM to %d.\n", mira016->illum_enable);
	ret = mira016_write(mira016, MIRA016_EN_TRIG_ILLUM, mira016->illum_enable);
	if (ret)
	{
		dev_err(&client->dev, "Error setting EN_TRIG_ILLUM to %d.", mira016->illum_enable);
		return ret;
	}

	if (MIRA016_LPS_DISABLED)
	{
		// Set illumination width. Write 24 bits. All 24 bits are valid.
		printk(KERN_INFO "[MIRA016]: LPS DISABLED. Writing ILLUM_WIDTH to %u.\n", mira016->illum_width);
		ret = mira016_write_be24(mira016, MIRA016_ILLUM_WIDTH_REG, mira016->illum_width);
		if (ret)
		{
			dev_err(&client->dev, "LPS DISABLED. Error setting ILLUM_WIDTH to %u.", mira016->illum_width);
			return ret;
		}
	}
	else
	{
		// LSP active, adjust pulse with to compensate for dead time during exposure
		// INPUT PARAMS: LPS_CYCLE_TIME, EXP_TIME, FRAME_TIME, GLOB_TIME, READOUT_TIME
		//
		//
		// printk(KERN_INFO "[MIRA016]: LPS DISABLED. Exposure name is  to %u.\n", mira016->exposure->name);
		u32 cur_exposure = (mira016->exposure->val * MIRA016_DEFAULT_LINE_LENGTH);

		printk(KERN_INFO "[MIRA016]: LPS ENABLED. Exposure cur is  to %u.\n", mira016->exposure->val);
		printk(KERN_INFO "[MIRA016]: LPS ENABLED. Exposure cur IN US  is  to %u.\n", cur_exposure);

		u32 readout_time = (11 + MIRA016_PIXEL_ARRAY_HEIGHT) * mira016->row_length * 8 / MIRA016_DATA_RATE;

		printk(KERN_INFO "[MIRA016]: LPS ENABLED. MIRA016_LPS_CYCLE_TIME is  to %u.\n", MIRA016_LPS_CYCLE_TIME);
		printk(KERN_INFO "[MIRA016]: LPS ENABLED. MIRA016_GLOB_TIME is  to %u.\n", MIRA016_GLOB_TIME);
		printk(KERN_INFO "[MIRA016]: LPS ENABLED. frame time is  to %u.\n", mira016->target_frame_time_us);
		printk(KERN_INFO "[MIRA016]: LPS ENABLED. glob time is  to %u.\n", MIRA016_GLOB_TIME);
		printk(KERN_INFO "[MIRA016]: LPS ENABLED. read time is  to %u.\n", MIRA016_READOUT_TIME);
		printk(KERN_INFO "[MIRA016]: LPS ENABLED. new read time is  to %u.\n", readout_time);
		printk(KERN_INFO "[MIRA016]: LPS ENABLED. mira016->target_frame_time_us - MIRA016_GLOB_TIME - readout_time is  to %u.\n", mira016->target_frame_time_us - MIRA016_GLOB_TIME - MIRA016_READOUT_TIME);

		// case 1: EXP_TIME < LPS_CYCLE_TIME
		if (cur_exposure < MIRA016_LPS_CYCLE_TIME)
		{
			printk(KERN_INFO "[MIRA016]: LPS CASE 1 to %u.\n", mira016->illum_width);
			lps_time = 0;
		}
		// case 2: LPS_ CYCLE_ TIME<EXP_ TIME≤FRAME_ TIME-GLOB_ TIME-READOUT_TIME
		else if ((MIRA016_LPS_CYCLE_TIME < cur_exposure) && (cur_exposure < (mira016->target_frame_time_us - MIRA016_GLOB_TIME - readout_time)))
		{
			lps_time = cur_exposure - MIRA016_LPS_CYCLE_TIME;
			printk(KERN_INFO "[MIRA016]: LPS CASE 2 - LPS TIME is %u.\n", lps_time);
		}
		// case 3: LPS_ CYCLE_ TIME≤FRAME_ TIME-GLOB_ TIME-READOUT_TIME<EXP_ TIME
		else if ((MIRA016_LPS_CYCLE_TIME < (mira016->target_frame_time_us - MIRA016_GLOB_TIME - readout_time)) && ((mira016->target_frame_time_us - MIRA016_GLOB_TIME - readout_time) < cur_exposure))
		{
			lps_time = (mira016->target_frame_time_us - MIRA016_GLOB_TIME - readout_time) - MIRA016_LPS_CYCLE_TIME;
			printk(KERN_INFO "[MIRA016]: LPS CASE 3 - LPS TIME is %u.\n", lps_time);
		}
		// case 4: FRAME_ TIME-GLOB_ TIME-READOUT_ TIME≤LPS_ CYCLE_ TIME<EXP_ TIME
		else if (((mira016->target_frame_time_us - MIRA016_GLOB_TIME - readout_time) < MIRA016_LPS_CYCLE_TIME) && (MIRA016_LPS_CYCLE_TIME < cur_exposure))
		{
			printk(KERN_INFO "[MIRA016]: LPS CASE 4 to %u.\n", mira016->illum_width);
			lps_time = 0;
		}
		else
		{
			printk(KERN_INFO "[MIRA016]: LPS CASE 5 invalid to %u.\n", mira016->illum_width);
		}

		width_adjust = (lps_time > 0 ? lps_time * 1500 / 8 - 30 : 0);
		printk(KERN_INFO "[MIRA016]: LPS ENABLE -s width adjust is  %u.\n", width_adjust);

		ret = mira016_write_be24(mira016, MIRA016_ILLUM_WIDTH_REG, mira016->illum_width - width_adjust);

		if (ret)
		{
			dev_err(&client->dev, "LPS ENABLED. Error setting ILLUM_WIDTH to %u.", mira016->illum_width - width_adjust);
			return ret;
		}
	}
	return ret;
}


static int mira016_v4l2_reg_w(struct mira016 *mira016, u32 value)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&mira016->sd);
	u32 ret = 0;
	u32 tmp_flag;

	u16 reg_addr = (value >> 8) & 0xFFFF;
	u8 reg_val = value & 0xFF;
	u8 reg_flag = (value >> 24) & 0xFF;

	// printk(KERN_INFO "[MIRA016]: %s reg_flag: 0x%02X; reg_addr: 0x%04X; reg_val: 0x%02X.\n",
	// 		__func__, reg_flag, reg_addr, reg_val);

	if (reg_flag & AMS_CAMERA_CID_MIRA016_REG_FLAG_CMD_SEL)
	{
		if (reg_flag == AMS_CAMERA_CID_MIRA016_REG_FLAG_SLEEP_US)
		{
			// If it is for sleep, combine all 24 bits of reg_addr and reg_val as sleep us.
			u32 sleep_us_val = value & 0x00FFFFFF;
			// Sleep range needs an interval, default to 1/8 of the sleep value.
			u32 sleep_us_interval = sleep_us_val >> 3;
			printk(KERN_INFO "[MIRA016]: %s sleep_us: %u.\n", __func__, sleep_us_val);
			usleep_range(sleep_us_val, sleep_us_val + sleep_us_interval);
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA016_REG_FLAG_RESET_ON)
		{
			printk(KERN_INFO "[MIRA016]: %s Enable reset at stream on/off.\n", __func__);
			mira016->skip_reset = 0;
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA016_REG_FLAG_RESET_OFF)
		{
			printk(KERN_INFO "[MIRA016]: %s Disable reset at stream on/off.\n", __func__);
			mira016->skip_reset = 1;
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA016_REG_FLAG_REG_UP_ON)
		{
			printk(KERN_INFO "[MIRA016]: %s Enable base register sequence upload.\n", __func__);
			mira016->skip_reg_upload = 0;
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA016_REG_FLAG_REG_UP_OFF)
		{
			printk(KERN_INFO "[MIRA016]: %s Disable base register sequence upload.\n", __func__);
			mira016->skip_reg_upload = 1;
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA016_REG_FLAG_POWER_ON)
		{
			printk(KERN_INFO "[MIRA016]: %s Call power on function mira016_power_on().\n", __func__);
			/* Temporarily disable skip_reset if manually doing power on/off */
			tmp_flag = mira016->skip_reset;
			mira016->skip_reset = 0;
			mira016_power_on(&client->dev);
			mira016->skip_reset = tmp_flag;
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA016_REG_FLAG_POWER_OFF)
		{
			printk(KERN_INFO "[MIRA016]: %s Call power off function mira016_power_off().\n", __func__);
			/* Temporarily disable skip_reset if manually doing power on/off */
			tmp_flag = mira016->skip_reset;
			mira016->skip_reset = 0;
			mira016_power_off(&client->dev);
			mira016->skip_reset = tmp_flag;
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA016_REG_FLAG_ILLUM_TRIG_ON)
		{
			printk(KERN_INFO "[MIRA016]: %s Enable illumination trigger.\n", __func__);
			mira016->illum_enable = 1;
			mira016_write_illum_trig_regs(mira016);
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA016_REG_FLAG_ILLUM_TRIG_OFF)
		{
			printk(KERN_INFO "[MIRA016]: %s Disable illumination trigger.\n", __func__);
			mira016->illum_enable = 0;
			mira016_write_illum_trig_regs(mira016);
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA016_REG_FLAG_ILLUM_WIDTH)
		{
			// Combine all 24 bits of reg_addr and reg_val as ILLUM_WIDTH.
			u32 illum_width = value & 0x00FFFFFF;
			printk(KERN_INFO "[MIRA016]: %s Set ILLUM_WIDTH to 0x%X.\n", __func__, illum_width);
			mira016->illum_width = illum_width;
			mira016_write_illum_trig_regs(mira016);
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA016_REG_FLAG_ILLUM_DELAY)
		{
			// Combine reg_addr and reg_val, then select 20 bits from [19:0] as ILLUM_DELAY.
			u32 illum_delay = value & 0x000FFFFF;
			printk(KERN_INFO "[MIRA016]: %s Set ILLUM_DELAY to 0x%X.\n", __func__, illum_delay);
			mira016->illum_delay = illum_delay;
			mira016_write_illum_trig_regs(mira016);
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA016_REG_FLAG_ILLUM_EXP_T_ON)
		{
			printk(KERN_INFO "[MIRA016]: %s enable ILLUM_WIDTH to automatically track exposure time.\n", __func__);
			mira016->illum_width_auto = 1;
			mira016_write_illum_trig_regs(mira016);
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA016_REG_FLAG_ILLUM_EXP_T_OFF)
		{
			printk(KERN_INFO "[MIRA016]: %s disable ILLUM_WIDTH to automatically track exposure time.\n", __func__);
			mira016->illum_width_auto = 0;
			mira016_write_illum_trig_regs(mira016);
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA016_REG_FLAG_STREAM_CTRL_ON)
		{
			printk(KERN_INFO "[MIRA016]: %s Force stream control even if (skip_reg_upload == 1).\n", __func__);
			mira016->force_stream_ctrl = 1;
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA016_REG_FLAG_STREAM_CTRL_OFF)
		{
			printk(KERN_INFO "[MIRA016]: %s Disable stream control if (skip_reg_upload == 1).\n", __func__);
			mira016->force_stream_ctrl = 0;
		}
		else
		{
			printk(KERN_INFO "[MIRA016]: %s unknown command from flag %u, ignored.\n", __func__, reg_flag);
		}
	}
	else if (reg_flag & AMS_CAMERA_CID_MIRA016_REG_FLAG_FOR_READ)
	{
		// If it is for read, skip reagister write, cache addr and flag for read.
		mira016->mira016_reg_w_cached_addr = reg_addr;
		mira016->mira016_reg_w_cached_flag = reg_flag;
	}
	else
	{
		// If it is for write, select which I2C device by the flag "I2C_SEL".
		if ((reg_flag & AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_MIRA)
		{
			// Before writing MIRA016 register, first optionally select BANK and CONTEXT
			if (reg_flag & AMS_CAMERA_CID_MIRA016_REG_FLAG_USE_BANK)
			{
				u8 bank;
				u8 context;
				// Set conetxt bank 0 or 1
				if (reg_flag & AMS_CAMERA_CID_MIRA016_REG_FLAG_BANK)
				{
					bank = 1;
				}
				else
				{
					bank = 0;
				}
				// printk(KERN_INFO "[MIRA016]: %s select bank: %u.\n", __func__, bank);
				ret = mira016_write(mira016, MIRA016_BANK_SEL_REG, bank);
				if (ret)
				{
					dev_err(&client->dev, "Error setting BANK_SEL_REG.");
					return ret;
				}
				// Set context bank 1A or bank 1B
				if (reg_flag & AMS_CAMERA_CID_MIRA016_REG_FLAG_CONTEXT)
				{
					context = 1;
				}
				else
				{
					context = 0;
				}
				// printk(KERN_INFO "[MIRA016]: %s select context: %u.\n", __func__, context);
				ret = mira016_write(mira016, MIRA016_RW_CONTEXT_REG, context);
				if (ret)
				{
					dev_err(&client->dev, "Error setting RW_CONTEXT.");
					return ret;
				}
			}
			// Writing the actual MIRA016 register
			// printk(KERN_INFO "[MIRA016]: %s write reg_addr: 0x%04X; reg_val: 0x%02X.\n", __func__, reg_addr, reg_val);
			ret = mira016_write(mira016, reg_addr, reg_val);
			if (ret)
			{
				dev_err_ratelimited(&client->dev, "Error AMS_CAMERA_CID_MIRA_REG_W reg_addr %X.\n", reg_addr);
				return -EINVAL;
			}
		}
		else if ((reg_flag & AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_SET_TBD)
		{
			/* User tries to set TBD I2C address, store reg_val to mira016->tbd_client_i2c_addr. Skip write. */
			printk(KERN_INFO "[MIRA016]: mira016->tbd_client_i2c_addr = 0x%X.\n", reg_val);
			mira016->tbd_client_i2c_addr = reg_val;
		}
		else if ((reg_flag & AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_TBD)
		{
			if (mira016->tbd_client_i2c_addr == MIRA016PMIC_I2C_ADDR)
			{
				// Write PMIC. Use pre-allocated mira016->pmic_client.
				printk(KERN_INFO "[MIRA016]: write pmic_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
				ret = mira016pmic_write(mira016->pmic_client, (u8)(reg_addr & 0xFF), reg_val);
			}
			else if (mira016->tbd_client_i2c_addr == MIRA016UC_I2C_ADDR)
			{
				// Write micro-controller. Use pre-allocated mira016->uc_client.
				printk(KERN_INFO "[MIRA016]: write uc_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
				ret = mira016pmic_write(mira016->uc_client, (u8)(reg_addr & 0xFF), reg_val);
			}
			else if (mira016->tbd_client_i2c_addr == MIRA016LED_I2C_ADDR)
			{
				// Write LED driver. Use pre-allocated mira016->led_client.
				printk(KERN_INFO "[MIRA016]: write led_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
				ret = mira016pmic_write(mira016->led_client, (u8)(reg_addr & 0xFF), reg_val);
			}
			else
			{
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

static int mira016_v4l2_reg_r(struct mira016 *mira016, u32 *value)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&mira016->sd);
	u32 ret = 0;

	u16 reg_addr = mira016->mira016_reg_w_cached_addr;
	u8 reg_flag = mira016->mira016_reg_w_cached_flag;
	u8 reg_val = 0;

	*value = 0;

	if ((reg_flag & AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_MIRA)
	{
		if (reg_flag & AMS_CAMERA_CID_MIRA016_REG_FLAG_USE_BANK)
		{
			u8 bank;
			u8 context;
			// Set conetxt bank 0 or 1
			if (reg_flag & AMS_CAMERA_CID_MIRA016_REG_FLAG_BANK)
			{
				bank = 1;
			}
			else
			{
				bank = 0;
			}
			// printk(KERN_INFO "[MIRA016]: %s select bank: %u.\n", __func__, bank);
			ret = mira016_write(mira016, MIRA016_BANK_SEL_REG, bank);
			if (ret)
			{
				dev_err(&client->dev, "Error setting BANK_SEL_REG.");
				return ret;
			}
			// Set context bank 1A or bank 1B
			if (reg_flag & AMS_CAMERA_CID_MIRA016_REG_FLAG_CONTEXT)
			{
				context = 1;
			}
			else
			{
				context = 0;
			}
			// printk(KERN_INFO "[MIRA016]: %s select context: %u.\n", __func__, context);
			ret = mira016_write(mira016, MIRA016_RW_CONTEXT_REG, context);
			if (ret)
			{
				dev_err(&client->dev, "Error setting RW_CONTEXT.");
				return ret;
			}
		}
		ret = mira016_read(mira016, reg_addr, &reg_val);
		if (ret)
		{
			dev_err_ratelimited(&client->dev, "Error AMS_CAMERA_CID_MIRA_REG_R reg_addr %X.\n", reg_addr);
			return -EINVAL;
		}
	}
	else if ((reg_flag & AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_TBD)
	{
		if (mira016->tbd_client_i2c_addr == MIRA016PMIC_I2C_ADDR)
		{
			// Read PMIC. Use pre-allocated mira016->pmic_client.
			ret = mira016pmic_read(mira016->pmic_client, (u8)(reg_addr & 0xFF), &reg_val);
			printk(KERN_INFO "[MIRA016]: read pmic_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
		}
		else if (mira016->tbd_client_i2c_addr == MIRA016UC_I2C_ADDR)
		{
			// Read micro-controller. Use pre-allocated mira016->uc_client.
			ret = mira016pmic_read(mira016->uc_client, (u8)(reg_addr & 0xFF), &reg_val);
			printk(KERN_INFO "[MIRA016]: read uc_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
		}
		else if (mira016->tbd_client_i2c_addr == MIRA016LED_I2C_ADDR)
		{
			// Read LED driver. Use pre-allocated mira016->led_client.
			ret = mira016pmic_read(mira016->led_client, (u8)(reg_addr & 0xFF), &reg_val);
			printk(KERN_INFO "[MIRA016]: read led_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
		}
		else
		{
			/* Read other TBD I2C address.
			 * The TBD I2C address is set via AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_SET_TBD.
			 * The TBD I2C address is stored in mira016->tbd_client_i2c_addr.
			 * A temporary I2C client, tmp_client, is created and then destroyed (unregistered).
			 */
			struct i2c_client *tmp_client;
			tmp_client = i2c_new_dummy_device(client->adapter, mira016->tbd_client_i2c_addr);
			if (IS_ERR(tmp_client))
				return PTR_ERR(tmp_client);
			ret = mira016pmic_read(tmp_client, (u8)(reg_addr & 0xFF), &reg_val);
			printk(KERN_INFO "[MIRA016]: read tbd_client, i2c_addr %u, reg_addr 0x%X, reg_val 0x%X.\n",
				   mira016->tbd_client_i2c_addr, (u8)(reg_addr & 0xFF), reg_val);
			i2c_unregister_device(tmp_client);
		}
	}

	// Return 32-bit value that includes flags, addr, and register value
	*value = ((u32)reg_flag << 24) | ((u32)reg_addr << 8) | (u32)reg_val;

	// printk(KERN_INFO "[MIRA016]: mira016_v4l2_reg_r() reg_flag: 0x%02X; reg_addr: 0x%04X, reg_val: 0x%02X.\n",
	// 		reg_flag, reg_addr, reg_val);

	return 0;
}

// Returns the maximum exposure time in microseconds (reg value)
static u32 mira016_calculate_max_exposure_time(u32 row_length, u32 vsize,
											   u32 vblank)
{
	(void)(row_length);
	(void)(vsize);
	(void)(vblank);
	/* MIRA016 does not have a max exposure limit besides register bits */
	return MIRA016_EXPOSURE_MAX_LINES;
	return MIRA016_EXPOSURE_MAX_LINES;
}


static int mira016_write_exposure_reg(struct mira016 *mira016, u32 exposure_lines)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&mira016->sd);
	const u32 min_exposure = MIRA016_EXPOSURE_MIN_US;
	u32 max_exposure = mira016->exposure->maximum;
	u32 exposure = exposure_lines * MIRA016_DEFAULT_LINE_LENGTH;
	u32 ret = 0;

	if (exposure < min_exposure)
	{
		exposure = min_exposure;
	}
	if (exposure > max_exposure)
	{
		exposure = max_exposure;
	}

	/* Write Bank 1 context 0 */
	ret = mira016_write(mira016, MIRA016_RW_CONTEXT_REG, 0);
	ret = mira016_write(mira016, MIRA016_BANK_SEL_REG, 1);
	ret = mira016_write_be32(mira016, MIRA016_EXP_TIME_L_REG, exposure);
	/* Write Bank 1 context 1 */
	ret = mira016_write(mira016, MIRA016_RW_CONTEXT_REG, 1);
	ret = mira016_write_be32(mira016, MIRA016_EXP_TIME_L_REG, exposure);
	if (ret)
	{
		dev_err_ratelimited(&client->dev, "Error setting exposure time to %d", exposure);
		return -EINVAL;
	}
	if (mira016->illum_width_auto == 1)
	{
		mira016->illum_width = exposure * MIRA016_DATA_RATE / 8;
		mira016_write_illum_trig_regs(mira016);
	}

	return 0;
}


static int mira016_write_target_frame_time_reg(struct mira016 *mira016, u32 target_frame_time_us)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&mira016->sd);
	u32 ret = 0;

	/* Write Bank 1 context 0 */
	ret = mira016_write(mira016, MIRA016_RW_CONTEXT_REG, 0);
	ret = mira016_write(mira016, MIRA016_BANK_SEL_REG, 1);
	ret = mira016_write_be32(mira016, MIRA016_TARGET_FRAME_TIME_REG, target_frame_time_us);
	/* Write Bank 1 context 1 */
	ret = mira016_write(mira016, MIRA016_RW_CONTEXT_REG, 1);
	ret = mira016_write_be32(mira016, MIRA016_TARGET_FRAME_TIME_REG, target_frame_time_us);
	if (ret)
	{
		dev_err_ratelimited(&client->dev, "Error setting target frame time to %d", target_frame_time_us);
		return -EINVAL;
	}

	return 0;
}

static int mira016_write_start_streaming_regs(struct mira016 *mira016)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&mira016->sd);
	int ret = 0;

	// Set conetxt bank 0 or 1
	ret = mira016_write(mira016, MIRA016_BANK_SEL_REG, 0);
	if (ret)
	{
		dev_err(&client->dev, "Error setting BANK_SEL_REG.");
		return ret;
	}

	// Set context bank 1A or bank 1B
	ret = mira016_write(mira016, MIRA016_RW_CONTEXT_REG, 0);
	if (ret)
	{
		dev_err(&client->dev, "Error setting RW_CONTEXT.");
		return ret;
	}

	// Raising CMD_REQ_1 to 1 for REQ_EXP
	ret = mira016_write(mira016, MIRA016_CMD_REQ_1_REG,
						1);
	if (ret)
	{
		dev_err(&client->dev, "Error setting CMD_REQ_1 to 1 for REQ_EXP.");
		return ret;
	}

	usleep_range(10, 20);

	// Setting CMD_REQ_1 tp 0 for REQ_EXP
	ret = mira016_write(mira016, MIRA016_CMD_REQ_1_REG,
						0);
	if (ret)
	{
		dev_err(&client->dev, "Error setting CMD_REQ_1 to 0 for REQ_EXP.");
		return ret;
	}
	usleep_range(10, 20);

	return ret;
}

static int mira016_write_stop_streaming_regs(struct mira016 *mira016)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&mira016->sd);
	int ret = 0;
	printk(KERN_INFO "[MIRA016]: mira016_write_stop_streaming_regs  function.\n");

	// Set conetxt bank 0 or 1
	ret = mira016_write(mira016, MIRA016_BANK_SEL_REG, 0);
	if (ret)
	{
		dev_err(&client->dev, "Error setting BANK_SEL_REG.");
		return ret;
	}

	// Raising CMD_HALT_BLOCK to 1 to stop streaming
	ret = mira016_write(mira016, MIRA016_CMD_HALT_BLOCK_REG,
						1);
	if (ret)
	{
		dev_err(&client->dev, "Error setting CMD_HALT_BLOCK to 1.");
		return ret;
	}

	usleep_range(10, 20);

	// Setting CMD_HALT_BLOCK to 0 to stop streaming
	ret = mira016_write(mira016, MIRA016_CMD_HALT_BLOCK_REG,
						0);
	if (ret)
	{
		dev_err(&client->dev, "Error setting CMD_HALT_BLOCK to 0.");
		return ret;
	}
	usleep_range(10, 20);


	return ret;
}


static int mira016_write_analog_gain_reg(struct mira016 *mira016, u8 gain)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&mira016->sd);
	u32 num_of_regs;
	u32 ret = 0;
	u32 wait_us = 20000;
	u16 cds_offset = 1700;
	u16 dark_offset_100 = 1794; // noncont clock
	u16 scale_factor = 1;
	u16 preamp_gain_inv = 1;
	u16 preamp_gain = 1;

	u16 analog_gain = 1;
	u16 offset_clipping = 0;
	u16 scaled_offset = 0;
	printk(KERN_INFO "[MIRA016]: Write analog gain %u",gain);

	// Select partial register sequence according to bit depth
	if (mira016->bit_depth == 12)
	{
		// Select register sequence according to gain value
		if (gain == 0)
		{
			mira016_write_stop_streaming_regs(mira016);
			usleep_range(wait_us, wait_us + 100);
			printk(KERN_INFO "[mira016]: Write reg sequence for analog gain x1 in 12 bit mode");
			num_of_regs = ARRAY_SIZE(partial_analog_gain_x1_12bit);
			ret = mira016_write_regs(mira016, partial_analog_gain_x1_12bit, num_of_regs);
			mira016_write_start_streaming_regs(mira016);
			mira016->row_length = 1504;
		}
		else if (gain == 1)
		{
			mira016_write_stop_streaming_regs(mira016);
			usleep_range(wait_us, wait_us + 100);
			printk(KERN_INFO "[mira016]: Write reg sequence for analog gain x2 in 12 bit mode");
			num_of_regs = ARRAY_SIZE(partial_analog_gain_x2_12bit);
			ret = mira016_write_regs(mira016, partial_analog_gain_x2_12bit, num_of_regs);
			mira016_write_start_streaming_regs(mira016);
			mira016->row_length = 2056;
		}
		else
		{
			// Other gains are not supported
			printk(KERN_INFO "[mira016]: Ignore analog gain %d in 12 bit mode", gain);
		}
	}
	else if (mira016->bit_depth == 10)
	{
		// Select register sequence according to gain value
		if (gain < ARRAY_SIZE(fine_gain_lut_10bit_hs_4x))
		{
			u32 analog_gain = fine_gain_lut_10bit_hs_4x[gain].analog_gain;
			u8 gdig_preamp = fine_gain_lut_10bit_hs_4x[gain].gdig_preamp;
			u8 rg_adcgain = fine_gain_lut_10bit_hs_4x[gain].rg_adcgain;
			u8 rg_mult = fine_gain_lut_10bit_hs_4x[gain].rg_mult;
			/* otp_cal_val should come from OTP, but OTP may have incorrect value. */
			u16 preamp_gain_inv = 16 / (gdig_preamp + 1); // invert because fixed point arithmetic

	
			/* Stop streaming and wait for frame data transmission done */
			mira016_write_stop_streaming_regs(mira016);
			usleep_range(wait_us, wait_us + 100);
			/* Write fine gain registers */
			printk(KERN_INFO "[MIRA016]: Write reg sequence for analog gain %u in 10 bit mode", gain);
			printk(KERN_INFO "[MIRA016]: analoggain: %u,gdig_preamp: %u rg_adcgain: %u, rg_mult: %u\n",
				   analog_gain, gdig_preamp, rg_adcgain, rg_mult );
			mira016_write(mira016, MIRA016_RW_CONTEXT_REG, 0);
			mira016_write(mira016, MIRA016_BANK_SEL_REG, 1);
			mira016_write(mira016, MIRA016_GDIG_PREAMP, gdig_preamp);
			mira016_write(mira016, MIRA016_BANK_SEL_REG, 0);
			mira016_write(mira016, MIRA016_BIAS_RG_ADCGAIN, rg_adcgain);
			mira016_write(mira016, MIRA016_BIAS_RG_MULT, rg_mult);
			/* Resume streaming */
			mira016_write_start_streaming_regs(mira016);
		}



		else
		{
			// Other gains are not supported
			printk(KERN_INFO "[mira016]: Ignore analog gain %d in 12 bit mode", gain);
		}
	}
	else if (mira016->bit_depth == 8)
	{
		dark_offset_100 = 72; // noncont clock
		scale_factor = 16;
		cds_offset = 1540;

		if (gain < ARRAY_SIZE(fine_gain_lut_8bit_16x))
		{
			u32 analog_gain = fine_gain_lut_8bit_16x[gain].analog_gain;
			u8 gdig_preamp = fine_gain_lut_8bit_16x[gain].gdig_preamp;
			u8 rg_adcgain = fine_gain_lut_8bit_16x[gain].rg_adcgain;
			u8 rg_mult = fine_gain_lut_8bit_16x[gain].rg_mult;
			/* otp_cal_val should come from OTP, but OTP may have incorrect value. */
			u16 preamp_gain_inv = 16 / (gdig_preamp + 1);

				//  = (int)(cds_offset - (target_black_level*digital_gain - offset_clipping)) < 0 ? 0 : (int)(cds_offset - (target_black_level*digital_gain - offset_clipping));

			// u16 offset_clipping = (offset_clipping_calc < 0) ? 0 : (int)(offset_clipping_calc);
			/* Stop streaming and wait for frame data transmission done */
			mira016_write_stop_streaming_regs(mira016);
			usleep_range(wait_us, wait_us + 100);
			/* Write fine gain registers */
			printk(KERN_INFO "[MIRA016]: Write reg sequence for analog gain %u in 8 bit mode", gain);
			printk(KERN_INFO "[MIRA016]: analoggain: %u,gdig_preamp: %u rg_adcgain: %u, rg_mult: %u\n",
				   analog_gain, gdig_preamp, rg_adcgain, rg_mult );
			mira016_write(mira016, MIRA016_RW_CONTEXT_REG, 0);
			mira016_write(mira016, MIRA016_BANK_SEL_REG, 1);
			mira016_write(mira016, MIRA016_GDIG_PREAMP, gdig_preamp);
			mira016_write(mira016, MIRA016_BANK_SEL_REG, 0);
			mira016_write(mira016, MIRA016_BIAS_RG_ADCGAIN, rg_adcgain);
			mira016_write(mira016, MIRA016_BIAS_RG_MULT, rg_mult);
			/* Resume streaming */
			mira016_write_start_streaming_regs(mira016);
		}
		else
		{
			// Other gains are not supported
			printk(KERN_INFO "[mira016]: Ignore analog gain %d in 8 bit mode", gain);
		}
	}
	else
	{
		// Other bit depths are not supported
		printk(KERN_INFO "[mira016]: Ignore analog gain in %u bit mode", mira016->mode->bit_depth);
	}

	if (ret)
	{
		dev_err(&client->dev, "%s failed to set mode because wrong gain\n", __func__);
	}

	// Always return 0 even if it fails
	return 0;
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

	if (i >= ARRAY_SIZE(codes))
	{
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
	fmt->code = MEDIA_BUS_FMT_SGRBG12_1X12; // MEDIA_BUS_FMT_Y12_1X12;
	mira016->bit_depth = 12;
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
																MEDIA_BUS_FMT_SGRBG12_1X12);
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
	u32 target_frame_time_us;

	// Debug print
	// printk(KERN_INFO "[MIRA016]: mira016_set_ctrl() id: 0x%X value: 0x%X.\n", ctrl->id, ctrl->val);

	if (ctrl->id == V4L2_CID_VBLANK)
	{
		int exposure_max, exposure_def;

		/* Update max exposure while meeting expected vblanking */
		exposure_max = mira016_calculate_max_exposure_time(MIRA016_MIN_ROW_LENGTH,
														   mira016->mode->height,
														   ctrl->val);
		exposure_def = (exposure_max < MIRA016_DEFAULT_EXPOSURE_LINES) ? exposure_max : MIRA016_DEFAULT_EXPOSURE_LINES;
		exposure_def = (exposure_max < MIRA016_DEFAULT_EXPOSURE_LINES) ? exposure_max : MIRA016_DEFAULT_EXPOSURE_LINES;
		__v4l2_ctrl_modify_range(mira016->exposure,
								 mira016->exposure->minimum,
								 (int)( exposure_max ), mira016->exposure->step,
								 (int)( exposure_def ));
	}

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

	if (mira016->skip_reg_upload == 0)
	{
		switch (ctrl->id)
		{
		case V4L2_CID_ANALOGUE_GAIN:
			printk(KERN_INFO "[MIRA016]: V4L2_CID_ANALOGUE_GAIN: = %u !!!!!!!!!!!!!\n",
					ctrl->val);
			ret = mira016_write_analog_gain_reg(mira016, ctrl->val);
			break;
		case V4L2_CID_EXPOSURE:
			printk(KERN_INFO "[MIRA016]: V4L2_CID_EXPOSURE: exp line = %u \n",
					ctrl->val);
			ret = mira016_write_exposure_reg(mira016, ctrl->val);
			break;
		case V4L2_CID_TEST_PATTERN:

			break;
		case V4L2_CID_HFLIP:
			// TODO: HFLIP requires multiple register writes
			// ret = mira016_write(mira016, MIRA016_HFLIP_REG,
			//		        ctrl->val);
			printk(KERN_ERR "[MIRA016]: HFLIP: set %d.\n", ctrl->val);

			if (ctrl->val == 0)
			{
				printk(KERN_ERR "[MIRA016]: HFLIP: disable %d.\n", ctrl->val);
				ret = mira016_write(mira016, MIRA016_BANK_SEL_REG, 0x01);
				ret = mira016_write(mira016, MIRA016_XMIRROR_REG, 0);

			}
			else
			{
				printk(KERN_ERR "[MIRA016]: HFLIP: enable %d.\n", ctrl->val);
				ret = mira016_write(mira016, MIRA016_BANK_SEL_REG, 0x01);
				ret = mira016_write(mira016, MIRA016_XMIRROR_REG, 1);
			}
			break;
		case V4L2_CID_VFLIP:
			// {0x0029, 0x1},	// None
			// {0x002A, 0x90}, // None
			// {0x002B, 0x0},	// None
			// {0x002C, 0xE},	// None
			// TODO: VFLIP seems not supported in MIRA016
			printk(KERN_ERR "[MIRA016]: VFLIP: set %d.\n", ctrl->val);
			ret = mira016_write(mira016, MIRA016_BANK_SEL_REG, 0x00);

			if (ctrl->val == 0)
			{
				printk(KERN_ERR "[MIRA016]: VFLIP: disable %d.\n", ctrl->val);
				ret = mira016_write(mira016, MIRA016_YWIN_DIR_REG, 0x0);
				ret = mira016_write_be16(mira016, MIRA016_YWIN_START_REG, 14);

			}
			else
			{
				printk(KERN_ERR "[MIRA016]: VFLIP: enable %d.\n", ctrl->val);
				ret = mira016_write(mira016, MIRA016_YWIN_DIR_REG, 0x1);
				ret = mira016_write_be16(mira016, MIRA016_YWIN_START_REG, 413);
			}

			break;
		case V4L2_CID_VBLANK:
			/*
			 * In libcamera, frame time (== 1/framerate) is controlled by VBLANK:
			 * TARGET_FRAME_TIME (us) = 1000000 * ((1/PIXEL_RATE)*(WIDTH+HBLANK)*(HEIGHT+VBLANK))
			 */
			mira016->target_frame_time_us = (u32)((u64)(1000000 * (u64)(mira016->mode->width + mira016->mode->hblank) * (u64)(mira016->mode->height + ctrl->val)) / MIRA016_PIXEL_RATE);
			// Debug print
			printk(KERN_INFO "[MIRA016]: mira016_write_target_frame_time_reg target_frame_time_us = %u.\n",
				   mira016->target_frame_time_us);
			printk(KERN_INFO "[MIRA016]: width %d, hblank %d, vblank %d, height %d, ctrl->val %d.\n",
				   mira016->mode->width, mira016->mode->hblank, mira016->mode->min_vblank, mira016->mode->height, ctrl->val);
			ret = mira016_write_target_frame_time_reg(mira016, mira016->target_frame_time_us);
			break;
		case V4L2_CID_HBLANK:
			printk(KERN_INFO "[MIRA016]: V4L2_CID_HBLANK CALLED = %d.\n",
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

static int mira016_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mira016 *mira016 =
		container_of(ctrl->handler, struct mira016, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);
	int ret = 0;

	// printk(KERN_INFO "[MIRA016]: mira016_s_ctrl() id: %X value: %X.\n", ctrl->id, ctrl->val);

	/* Previously, register writes when powered off will be buffered.
	 * The buffer will be written to sensor when start_streaming.
	 * Now, register writes happens immediately, even powered off.
	 * Register writes when powered off will fail.
	 * Users need to make sure first power on then write register.
	 */

	switch (ctrl->id)
	{
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
	 * Ideally, V4L2 register read should happen only when powered on.
	 * However, perhaps there are use cases that,
	 * reading other I2C addr is desired when mira sensor is powered off.
	 * Therefore, the check of "powered" flag is disabled for now.
	 */

	switch (ctrl->id)
	{
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

	if (code->pad == IMAGE_PAD)
	{
		if (code->index >= ARRAY_SIZE(codes))
			return -EINVAL;

		code->code = mira016_validate_format_code_or_default(mira016,
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

static int mira016_enum_frame_size(struct v4l2_subdev *sd,
								   struct v4l2_subdev_state *sd_state,
								   struct v4l2_subdev_frame_size_enum *fse)
{
	struct mira016 *mira016 = to_mira016(sd);

	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	if (fse->pad == IMAGE_PAD)
	{
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
	}
	else
	{
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

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
	{
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(&mira016->sd, sd_state, fmt->pad);

		try_fmt->code = fmt->pad == IMAGE_PAD ? mira016_validate_format_code_or_default(mira016, try_fmt->code) : MEDIA_BUS_FMT_SENSOR_DATA;
		fmt->format = *try_fmt;
	}
	else
	{
		if (fmt->pad == IMAGE_PAD)
		{
			mira016_update_image_pad_format(mira016, mira016->mode,
											fmt);
			fmt->format.code = mira016_validate_format_code_or_default(mira016,
																	   mira016->fmt.code);
		}
		else
		{
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

	if (fmt->pad == IMAGE_PAD)
	{
		/* Validate format or use default */
		fmt->format.code = mira016_validate_format_code_or_default(mira016,
																   fmt->format.code);

		switch (fmt->format.code)
		{
		case MEDIA_BUS_FMT_SGRBG10_1X10:
			printk(KERN_INFO "[MIRA016]: fmt->format.code() selects 10 bit mode.\n");
			mira016->mode = &supported_modes[1];
			mira016->bit_depth = 10;
			// return 0;
			break;

		case MEDIA_BUS_FMT_SGRBG12_1X12:
			printk(KERN_INFO "[MIRA016]: fmt->format.code() selects 12 bit mode.\n");
			mira016->mode = &supported_modes[0];
			mira016->bit_depth = 12;
			// return 0;
			break;

		case MEDIA_BUS_FMT_SGRBG8_1X8:
			printk(KERN_INFO "[MIRA016]: fmt->format.code() selects 8 bit mode.\n");
			mira016->mode = &supported_modes[2];
			mira016->bit_depth = 8;
			// return 0;
			break;
		default:
			printk(KERN_ERR "Unknown format requested fmt->format.code() %d", fmt->format.code);
		}
		mode = v4l2_find_nearest_size(supported_modes,
									  ARRAY_SIZE(supported_modes),
									  width, height,
									  fmt->format.width,
									  fmt->format.height);
		mira016_update_image_pad_format(mira016, mode, fmt);
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		{
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
												  fmt->pad);
			*framefmt = fmt->format;
		}
		else if (mira016->mode != mode ||
				 mira016->fmt.code != fmt->format.code)
		{

			mira016->fmt = fmt->format;
			// mira016->mode = mode;

			// Update controls based on new mode (range and current value).
			max_exposure = mira016_calculate_max_exposure_time(MIRA016_MIN_ROW_LENGTH,
															   mira016->mode->height,
															   mira016->mode->min_vblank);

			default_exp = MIRA016_DEFAULT_EXPOSURE_LINES > max_exposure ? max_exposure : MIRA016_DEFAULT_EXPOSURE_LINES;
			rc = __v4l2_ctrl_modify_range(mira016->exposure,
										  mira016->exposure->minimum,
										  (int)( max_exposure), mira016->exposure->step,
										  (int)( default_exp));
			if (rc)
			{
				dev_err(&client->dev, "Error setting exposure range");
			}

			printk(KERN_INFO "[MIRA016]: MIRA016 SETTING ANA GAIN RANGE  = %u.\n",
				   ARRAY_SIZE(fine_gain_lut_8bit_16x) - 1);
			// #FIXME #TODO
			//  rc = __v4l2_ctrl_modify_range(mira016->gain,
			//  					 0, ARRAY_SIZE(fine_gain_lut_8bit_16x) - 1, 1, 0);
			rc = __v4l2_ctrl_modify_range(mira016->gain,
										  mira016->mode->gain_min,
										  mira016->mode->gain_max,
										  1,
										  0);
			if (rc)
			{
				dev_err(&client->dev, "Error setting gain range");
			}

			printk(KERN_INFO "[MIRA016]: MIRA016 VBLANK  = %u.\n",
				   mira016->mode->min_vblank);

			rc = __v4l2_ctrl_modify_range(mira016->vblank,
										  mira016->mode->min_vblank,
										  mira016->mode->max_vblank,
										  1,
										  MIRA016_DEFAULT_VBLANK_60 // 200 fps
);
			if (rc)
			{
				dev_err(&client->dev, "Error setting exposure range");
			}
			// Set the current vblank value
			rc = __v4l2_ctrl_s_ctrl(mira016->vblank, MIRA016_MIN_VBLANK_60);
			if (rc)
			{
				dev_err(&client->dev, "Error setting vblank value to %u",
						mira016->mode->min_vblank);
			}
		}
	}
	else
	{
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		{
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
												  fmt->pad);
			*framefmt = fmt->format;
		}
		else
		{
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
	switch (mira016->fmt.code)
	{
	case MEDIA_BUS_FMT_SGRBG8_1X8:
		printk(KERN_INFO "[MIRA016]: mira016_set_framefmt() selects 8 bit mode.\n");
		mira016->mode = &supported_modes[2];
		mira016->bit_depth = 8;
		__v4l2_ctrl_modify_range(mira016->gain,
								 0, ARRAY_SIZE(fine_gain_lut_8bit_16x) - 1, 1, 0);
		return 0;
	case MEDIA_BUS_FMT_SGRBG10_1X10:
		printk(KERN_INFO "[MIRA016]: mira016_set_framefmt() selects 10 bit mode.\n");
		mira016->mode = &supported_modes[1];
		mira016->bit_depth = 10;
		__v4l2_ctrl_modify_range(mira016->gain,
								 0, ARRAY_SIZE(fine_gain_lut_10bit_hs_4x) - 1, 1, 0);
		return 0;
	case MEDIA_BUS_FMT_SGRBG12_1X12:
		printk(KERN_INFO "[MIRA016]: mira016_set_framefmt() selects 12 bit mode.\n");
		mira016->mode = &supported_modes[0];
		mira016->bit_depth = 12;
		__v4l2_ctrl_modify_range(mira016->gain,
								 mira016->mode->gain_min, mira016->mode->gain_max,
								 MIRA016_ANALOG_GAIN_STEP, MIRA016_ANALOG_GAIN_DEFAULT);
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
	switch (which)
	{
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
	switch (sel->target)
	{
	case V4L2_SEL_TGT_CROP:
	{
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

	int ret;

	printk(KERN_INFO "[MIRA016]: Entering start streaming function.\n");

	/* Follow examples of other camera driver, here use pm_runtime_resume_and_get */
	ret = pm_runtime_resume_and_get(&client->dev);

	if (ret < 0)
	{
		printk(KERN_INFO "[MIRA016]: get_sync failed, but continue.\n");
		pm_runtime_put_noidle(&client->dev);
		return ret;
	}

	/* Set current mode according to frame format bit depth */
	ret = mira016_set_framefmt(mira016);
	if (ret)
	{
		dev_err(&client->dev, "%s failed to set frame format: %d\n",
				__func__, ret);
		goto err_rpm_put;
	}
	printk(KERN_INFO "[MIRA016]: Register sequence for %d bit mode will be used.\n", mira016->mode->bit_depth);
	usleep_range(100000, 150000);

	if (mira016->skip_reg_upload == 0)
	{
		/* Apply pre soft reset default values of current mode */
		reg_list = &mira016->mode->reg_list_pre_soft_reset;
		printk(KERN_INFO "[MIRA016]: Write %d regs.\n", reg_list->num_of_regs);
		ret = mira016_write_regs(mira016, reg_list->regs, reg_list->num_of_regs);
		if (ret)
		{
			dev_err(&client->dev, "%s failed to set mode\n", __func__);
			goto err_rpm_put;
		}
	}
	else
	{
		printk(KERN_INFO "[MIRA016]: Skip base register sequence upload, due to mira016->skip_reg_upload=%u.\n", mira016->skip_reg_upload);
	}

	printk(KERN_INFO "[MIRA016]: Entering v4l2 ctrl handler setup function.\n");

	/* Apply customized values from user */
	ret = __v4l2_ctrl_handler_setup(mira016->sd.ctrl_handler);
	printk(KERN_INFO "[MIRA016]: __v4l2_ctrl_handler_setup ret = %d.\n", ret);
	if (ret)
		goto err_rpm_put;

	usleep_range(8000, 10000);


	if (mira016->skip_reg_upload == 0 ||
		(mira016->skip_reg_upload == 1 && mira016->force_stream_ctrl == 1))
	{
		printk(KERN_INFO "[MIRA016]: Writing start streaming regs.\n");
		ret = mira016_write_start_streaming_regs(mira016);
		if (ret)
		{
			dev_err(&client->dev, "Could not write stream-on sequence");
			goto err_rpm_put;
		}
	}
	else
	{
		printk(KERN_INFO "[MIRA016]: Skip write_start_streaming_regs due to skip_reg_upload == %d and force_stream_ctrl == %d.\n",
			   mira016->skip_reg_upload, mira016->force_stream_ctrl);
	}

	/* vflip and hflip cannot change during streaming */
	printk(KERN_INFO "[MIRA016]: Entering v4l2 ctrl grab vflip grab vflip.\n");
	__v4l2_ctrl_grab(mira016->vflip, true);
	printk(KERN_INFO "[MIRA016]: Entering v4l2 ctrl grab vflip grab hflip.\n");
	__v4l2_ctrl_grab(mira016->hflip, true);

	printk(KERN_INFO "[MIRA016]: %s Enable illumination trigger.\n", __func__);
	mira016->illum_enable = 1;
	mira016_write_illum_trig_regs(mira016);

	return 0;

err_rpm_put:
	pm_runtime_put(&client->dev);
	return ret;
}
static void mira016_stop_streaming(struct mira016 *mira016)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);
	int ret = 0;
	printk(KERN_INFO "[MIRA016]: Entering mira016_stop_streaming function.\n");

	/* Unlock controls for vflip and hflip */
	__v4l2_ctrl_grab(mira016->vflip, false);
	__v4l2_ctrl_grab(mira016->hflip, false);

	if (mira016->skip_reset == 0)
	{
		if (mira016->skip_reg_upload == 0 ||
			(mira016->skip_reg_upload == 1 && mira016->force_stream_ctrl == 1))
		{
			printk(KERN_INFO "[MIRA016]: Writing stop streaming regs.\n");
			ret = mira016_write_stop_streaming_regs(mira016);
			if (ret)
			{
				dev_err(&client->dev, "Could not write the stream-off sequence");
			}
		}
		else
		{
			printk(KERN_INFO "[MIRA016]: Skip write_stop_streaming_regs due to skip_reg_upload == %d and force_stream_ctrl == %d.\n",
				   mira016->skip_reg_upload, mira016->force_stream_ctrl);
		}
	}
	else
	{
		printk(KERN_INFO "[MIRA016]: Skip write_stop_streaming_regs due to mira016->skip_reset == %d.\n", mira016->skip_reset);
	}

	pm_runtime_put(&client->dev);
}
static int mira016_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct mira016 *mira016 = to_mira016(sd);
	int ret = 0;

	mutex_lock(&mira016->mutex);
	if (mira016->streaming == enable)
	{
		mutex_unlock(&mira016->mutex);
		return 0;
	}

	printk(KERN_INFO "[MIRA016]: Entering mira016_set_stream enable: %d.\n", enable);

	if (enable)
	{
		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = mira016_start_streaming(mira016);
		if (ret)
			goto err_unlock;
	}
	else
	{
		mira016_stop_streaming(mira016);
	}

	mira016->streaming = enable;

	mutex_unlock(&mira016->mutex);

	printk(KERN_INFO "[MIRA016]: Returning mira016_set_stream with ret: %d.\n", ret);

	return ret;

err_unlock:
	mira016_stop_streaming(mira016);
	mutex_unlock(&mira016->mutex);

	return ret;
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

	if (mira016->streaming)
	{
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
	printk(KERN_INFO "[MIRA016]: %s INIT_CONTROLS bitmode %X.\n", __func__, mira016->mode->bit_depth);

	/* By default, PIXEL_RATE is read only */
	mira016->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &mira016_ctrl_ops,
											V4L2_CID_PIXEL_RATE,
											MIRA016_PIXEL_RATE,
											MIRA016_PIXEL_RATE, 1,
											MIRA016_PIXEL_RATE);

	printk(KERN_INFO "[MIRA016]: %s V4L2_CID_VBLANK %X.\n", __func__, V4L2_CID_VBLANK);

	mira016->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &mira016_ctrl_ops,
										V4L2_CID_VBLANK, mira016->mode->min_vblank,
										mira016->mode->max_vblank, 1,
										MIRA016_MIN_VBLANK_60);

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
										  MIRA016_EXPOSURE_MIN_LINES, MIRA016_EXPOSURE_MAX_LINES,
										  1,
										  MIRA016_DEFAULT_EXPOSURE_LINES);
	printk(KERN_INFO "[MIRA016]: %s V4L2_CID_ANALOGUE_GAIN %X.\n", __func__, V4L2_CID_ANALOGUE_GAIN);

	mira016->gain = v4l2_ctrl_new_std(ctrl_hdlr, &mira016_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
									  mira016->mode->gain_min, mira016->mode->gain_max,
									  MIRA016_ANALOG_GAIN_STEP, MIRA016_ANALOG_GAIN_DEFAULT);

	printk(KERN_INFO "[MIRA016]: %s V4L2_CID_HFLIP new %X.\n", __func__, V4L2_CID_HFLIP);

	mira016->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &mira016_ctrl_ops,
									   V4L2_CID_HFLIP, 0, 1, 1, 0);
	//if (mira016->hflip)
	//	mira016->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	printk(KERN_INFO "[MIRA016]: %s V4L2_CID_VFLIP %X.\n", __func__, V4L2_CID_VFLIP);

	mira016->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &mira016_ctrl_ops,
									   V4L2_CID_VFLIP, 0, 1, 1, 0);
	//if (mira016->vflip)
        //		mira016->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;



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

	/* Check the number of MIPI CSI2 data lanes */
	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 1)
	{
		dev_err(dev, "only 1 data lanes are currently supported\n");
		goto error_out;
	}

	/* Check the link frequency set in device tree */
	if (!ep_cfg.nr_of_link_frequencies)
	{
		dev_err(dev, "link-frequency property not found in DT\n");
		goto error_out;
	}

	if (ep_cfg.nr_of_link_frequencies != 1 ||
		ep_cfg.link_frequencies[0] != MIRA016_DEFAULT_LINK_FREQ)
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
	if (IS_ERR(mira016->xclk))
	{
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(mira016->xclk);
	}

	mira016->xclk_freq = clk_get_rate(mira016->xclk);
	if (mira016->xclk_freq != MIRA016_SUPPORTED_XCLK_FREQ)
	{
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
				mira016->xclk_freq);
		return -EINVAL;
	}

	ret = mira016_get_regulators(mira016);
	if (ret)
	{
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	// {
	// 	printk(KERN_INFO "[MIRA016]: Init PMIC and uC and led driver.\n");
	// 	mira016->pmic_client = i2c_new_dummy_device(client->adapter,
	// 												MIRA016PMIC_I2C_ADDR);
	// 	if (IS_ERR(mira016->pmic_client))
	// 		return PTR_ERR(mira016->pmic_client);
	// 	mira016->uc_client = i2c_new_dummy_device(client->adapter,
	// 											  MIRA016UC_I2C_ADDR);
	// 	if (IS_ERR(mira016->uc_client))
	// 		return PTR_ERR(mira016->uc_client);
	// 	mira016->led_client = i2c_new_dummy_device(client->adapter,
	// 											   MIRA016LED_I2C_ADDR);
	// 	if (IS_ERR(mira016->led_client))
	// 		return PTR_ERR(mira016->led_client);
	// }

	dev_err(dev, "[MIRA016] Sleep for 1 second to let PMIC driver complete init.\n");

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


	/* Initialize default illumination trigger parameters */
	/* ILLUM_WIDTH is in unit of SEQ_TIME_BASE, equal to (8/MIRA016_DATA_RATE) us. */
	mira016->illum_width = MIRA016_ILLUM_WIDTH_DEFAULT;
	/* ILLUM_WIDTH AUTO will match illum to exposure pulse width*/
	mira016->illum_width_auto = MIRA016_ILLUM_SYNC_DEFAULT;
	/* ILLUM_ENABLE is True or False, enabling it will activate illum trig. */
	mira016->illum_enable = MIRA016_ILLUM_ENABLE_DEFAULT;
	/* ILLUM_DELAY is in unit of TIME_UNIT, equal to 1 us. In continuous stream mode, zero delay is 1<<19. */
	mira016->illum_delay = MIRA016_ILLUM_DELAY_DEFAULT;
	/* Set default mode to max resolution */
	mira016->mode = &supported_modes[1];
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
	if (ret)
	{
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	printk(KERN_INFO "[MIRA016]: Entering subdev sensor common function.\n");

	ret = v4l2_async_register_subdev_sensor(&mira016->sd);
	if (ret < 0)
	{
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
		SET_RUNTIME_PM_OPS(mira016_power_off, mira016_power_on, NULL)};

#endif // __MIRA016_INL__
