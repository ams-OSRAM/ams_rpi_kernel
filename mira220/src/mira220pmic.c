// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for ams MIRA220 cameras.
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
#include <asm/unaligned.h>

// Power on function timing
#define MIRA220PMIC_SUPPORTED_XCLK_FREQ		24000000

/* regulator supplies */
static const char * const mira220pmic_supply_name[] = {
	// TODO(jalv): Check supply names
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.8V) supply */
	"VDDL",  /* IF (1.2V) supply */
};

#define MIRA220PMIC_NUM_SUPPLIES ARRAY_SIZE(mira220pmic_supply_name)


struct mira220pmic {

	struct clk *xclk; /* system clock to MIRA220 */
	u32 xclk_freq;

	struct regulator_bulk_data supplies[MIRA220PMIC_NUM_SUPPLIES];
};

static int mira220pmic_read(struct i2c_client *client, u8 reg, u8 *val)
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

/* Power/clock management functions */
static int mira220pmic_power_on(struct device *dev, struct mira220pmic *mira220pmic)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret = -EINVAL;

	ret = regulator_bulk_enable(MIRA220PMIC_NUM_SUPPLIES, mira220pmic->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(mira220pmic->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}

	return 0;

reg_off:
	ret = regulator_bulk_disable(MIRA220PMIC_NUM_SUPPLIES, mira220pmic->supplies);
	return ret;
}

static int mira220pmic_power_off(struct device *dev)
{
	return 0;
}


static int mira220pmic_get_regulators(struct i2c_client *client, struct mira220pmic *mira220pmic)
{
	unsigned int i;

	for (i = 0; i < MIRA220PMIC_NUM_SUPPLIES; i++)
		mira220pmic->supplies[i].supply = mira220pmic_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       MIRA220PMIC_NUM_SUPPLIES,
				       mira220pmic->supplies);
}


/* Verify chip ID */
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

static int mira220pmic_check_hwcfg(struct device *dev)
{
	struct fwnode_handle *endpoint;
	int ret = -EINVAL;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	// TODO(jalv): Check device tree configuration and make sure it is supported by the driver
	ret = 0;

	fwnode_handle_put(endpoint);

	return ret;
}

static int mira220pmic_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct mira220pmic *mira220pmic;
	int ret;

	printk(KERN_INFO "[MIRA220PMIC]: probing pmic.\n");
	printk(KERN_INFO "[MIRA220PMIC]: Driver Version 0.0.\n");

	dev_err(dev, "[MIRA220PMIC] name: %s.\n", client->name);

	mira220pmic = devm_kzalloc(&client->dev, sizeof(*mira220pmic), GFP_KERNEL);
	if (!mira220pmic)
		return -ENOMEM;

	printk(KERN_INFO "[MIRA220PMIC]: Entering check hwcfg function.\n");

	/* Check the hardware configuration in device tree */
	if (mira220pmic_check_hwcfg(dev))
		return -EINVAL;

	// TODO(jalv): Get GPIO's, regulators and clocks from dts

	printk(KERN_INFO "[MIRA220PMIC]: Check xclk and freq.\n");

	/* Get system clock (xclk) */
	mira220pmic->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(mira220pmic->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(mira220pmic->xclk);
	}

	mira220pmic->xclk_freq = clk_get_rate(mira220pmic->xclk);
	if (mira220pmic->xclk_freq != MIRA220PMIC_SUPPORTED_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			mira220pmic->xclk_freq);
		return -EINVAL;
	}

	printk(KERN_INFO "[MIRA220PMIC]: Entering get regulators function.\n");

	ret = mira220pmic_get_regulators(client, mira220pmic);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}


	printk(KERN_INFO "[MIRA220PMIC]: Entering power on function.\n");

	/*
	 * The sensor must be powered for mira220_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = mira220pmic_power_on(dev, mira220pmic);
	if (ret)
		return ret;

	printk(KERN_INFO "[MIRA220PMIC]: Entering init controls function.\n");

	ret = mira220pmic_init_controls(client);
	if (ret)
		goto error_power_off;

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

error_power_off:
	return ret;
}

static int mira220pmic_remove(struct i2c_client *client)
{
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		mira220pmic_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

static const struct of_device_id mira220pmic_dt_ids[] = {
	{ .compatible = "ams,pmic" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mira220pmic_dt_ids);

static const struct i2c_device_id mira220pmic_ids[] = {
	{ "pmic", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mira220pmic_ids);


static const struct dev_pm_ops mira220pmic_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mira220pmic_suspend, mira220pmic_resume)
	//SET_RUNTIME_PM_OPS(mira220pmic_power_off, mira220pmic_power_on, NULL)
};

static struct i2c_driver mira220pmic_i2c_driver = {
	.driver = {
		.name = "mira220pmic",
		.of_match_table	= mira220pmic_dt_ids,
		.pm = &mira220pmic_pm_ops,
	},
	.probe_new = mira220pmic_probe,
	.remove = mira220pmic_remove,
	.id_table = mira220pmic_ids,
};

module_i2c_driver(mira220pmic_i2c_driver);

MODULE_AUTHOR("Javier Alvarez <javier.alvarez@ams-osram.com>");
MODULE_DESCRIPTION("ams MIRA220PMIC sensor driver");
MODULE_LICENSE("GPL v2");
