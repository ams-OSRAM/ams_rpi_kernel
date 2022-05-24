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
#include <asm/unaligned.h>

// Power on function timing
#define MIRA050PMIC_SUPPORTED_XCLK_FREQ		24000000

/* regulator supplies */
static const char * const mira050pmic_supply_name[] = {
	// TODO(jalv): Check supply names
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.8V) supply */
	"VDDL",  /* IF (1.2V) supply */
};

#define MIRA050PMIC_NUM_SUPPLIES ARRAY_SIZE(mira050pmic_supply_name)


struct mira050pmic {

	struct clk *xclk; /* system clock to MIRA050 */
	u32 xclk_freq;

	struct regulator_bulk_data supplies[MIRA050PMIC_NUM_SUPPLIES];
};

static int mira050pmic_read(struct i2c_client *client, u8 reg, u8 *val)
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

static int mira050pmic_write(struct i2c_client *client, u8 reg, u8 val)
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
static int mira050pmic_power_on(struct device *dev, struct mira050pmic *mira050pmic)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret = -EINVAL;

	ret = regulator_bulk_enable(MIRA050PMIC_NUM_SUPPLIES, mira050pmic->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(mira050pmic->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}

	return 0;

reg_off:
	ret = regulator_bulk_disable(MIRA050PMIC_NUM_SUPPLIES, mira050pmic->supplies);
	return ret;
}

static int mira050pmic_power_off(struct device *dev)
{
	return 0;
}


static int mira050pmic_get_regulators(struct i2c_client *client, struct mira050pmic *mira050pmic)
{
	unsigned int i;

	for (i = 0; i < MIRA050PMIC_NUM_SUPPLIES; i++)
		mira050pmic->supplies[i].supply = mira050pmic_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       MIRA050PMIC_NUM_SUPPLIES,
				       mira050pmic->supplies);
}


/* Verify chip ID */
static int mira050pmic_init_controls(struct i2c_client *client)
{
	int ret;
	u8 val;

	// Disable master switch //
	ret = mira050pmic_write(client, 0x62, 0x00);

	// Set all voltages to 0

	// DCDC1=0V
	ret = mira050pmic_write(client, 0x05, 0x00);
	// DCDC4=0V
	ret = mira050pmic_write(client, 0x0E, 0x0);
	// LDO1=0V VDDLO_PLL
	ret = mira050pmic_write(client, 0x11, 0x0);
	// LDO2=0.0V
	ret = mira050pmic_write(client, 0x14, 0x00);
	// LDO3=0.0V
	ret = mira050pmic_write(client, 0x17, 0x00);
	// LDO4=0V
	ret = mira050pmic_write(client, 0x1A, 0x00);
	// LDO5=0.0V
	ret = mira050pmic_write(client, 0x1C, 0x00);
	// LDO6=0.0V
	ret = mira050pmic_write(client, 0x1D, 0x00);
	// LDO7=0V
	ret = mira050pmic_write(client, 0x1E, 0x0);
	// LDO8=0.0V
	ret = mira050pmic_write(client, 0x1F, 0x00);
	// Disable LDO9 Lock
	ret = mira050pmic_write(client, 0x24, 0x48);
	// LDO9=0V VDDHI
	ret = mira050pmic_write(client, 0x20, 0x00);
	// LDO10=0V VDDLO_ANA
	ret = mira050pmic_write(client, 0x21, 0x0);

	// Enable master switch //
	usleep_range(50,60);
	ret = mira050pmic_write(client, 0x62, 0x0D);  // enable master switch
	usleep_range(50,60);

	// start PMIC
	// Keep LDOs always on
	ret = mira050pmic_write(client, 0x27, 0xFF);
	ret = mira050pmic_write(client, 0x28, 0xFF);
	ret = mira050pmic_write(client, 0x29, 0x00);
	ret = mira050pmic_write(client, 0x2A, 0x00);
	ret = mira050pmic_write(client, 0x2B, 0x00);

	// Unused LDO off //
	usleep_range(50,60);
	// set GPIO1=0
	ret = mira050pmic_write(client, 0x41, 0x04);
	// DCDC2=0.0V SPARE_PWR1
	ret = mira050pmic_write(client, 0x01, 0x00);
	ret = mira050pmic_write(client, 0x08, 0x00);
	// DCDC3=0V SPARE_PWR1
	ret = mira050pmic_write(client, 0x02, 0x00);
	ret = mira050pmic_write(client, 0x0B, 0x00);
	// LDO2=0.0V
	ret = mira050pmic_write(client, 0x14, 0x00);
	// LDO3=0.0V
	ret = mira050pmic_write(client, 0x17, 0x00);
	// LDO5=0.0V
	ret = mira050pmic_write(client, 0x1C, 0x00);
	// LDO6=0.0V
	ret = mira050pmic_write(client, 0x1D, 0x00);
	// LDO8=0.0V
	ret = mira050pmic_write(client, 0x1F, 0x00);

	ret = mira050pmic_write(client, 0x42, 4);

	// Enable 2.85V //
	usleep_range(50,60);
	// LDO4=2.85V VDDHI alternativ
	ret = mira050pmic_write(client, 0x1A, 0xB8); // Either 0x00 or 0xB8
	// Disable LDO9 Lock
	ret = mira050pmic_write(client, 0x24, 0x48);
	// LDO9=2.85V VDDHI
	ret = mira050pmic_read(client, 0x20, &val);
	dev_err(&client->dev, "Read 0x20 with val %x\n", val);
	ret = mira050pmic_write(client, 0x20, 0xB9);
	ret = mira050pmic_read(client, 0x20, &val);
	dev_err(&client->dev, "Read 0x20 with val %x\n", val);

	// VPIXH on cob = vdd25A on interposer = LDO4 on pmic
	// VPIXH should connect to VDD28 on pcb, or enable 4th supply
	ret = mira050pmic_read(client, 0x19, &val);
	dev_err(&client->dev, "Read 0x19 with val %x\n", val);
	ret = mira050pmic_write(client, 0x19, 0x38);
	ret = mira050pmic_read(client, 0x19, &val);
	dev_err(&client->dev, "Read 0x19 with val %x\n", val);


	// Enable 1.2V //
	usleep_range(700,710);
	// LDO1=1.2V VDDLO_PLL
	ret = mira050pmic_write(client, 0x12, 0x16);
	ret = mira050pmic_write(client, 0x10, 0x16);
	ret = mira050pmic_write(client, 0x11, 0x90);
	// LDO7=1.2V VDDLO_DIG
	ret = mira050pmic_write(client, 0x1E, 0x90);
	// LDO10=1.2V VDDLO_ANA
	ret = mira050pmic_write(client, 0x21, 0x90);

	// Enable 1.80V //
	usleep_range(50,60);
	// DCDC1=1.8V VINLDO1p8 >=1P8
	ret = mira050pmic_write(client, 0x00, 0x00);
	ret = mira050pmic_write(client, 0x04, 0x34);
	ret = mira050pmic_write(client, 0x06, 0xBF);
	ret = mira050pmic_write(client, 0x05, 0xB4);
	// DCDC4=1.8V VDDIO
	ret = mira050pmic_write(client, 0x03, 0x00);
	ret = mira050pmic_write(client, 0x0D, 0x34);
	ret = mira050pmic_write(client, 0x0F, 0xBF);
	ret = mira050pmic_write(client, 0x0E, 0xB4);

	// Enable green LED //
	usleep_range(50,60);
	ret = mira050pmic_write(client, 0x43, 0x40); // leda
	ret = mira050pmic_write(client, 0x44, 0x40); // ledb
	ret = mira050pmic_write(client, 0x45, 0x40); // ledc

	ret = mira050pmic_write(client, 0x47, 0x02); // leda ctrl1
	ret = mira050pmic_write(client, 0x4F, 0x02); // ledb ctrl1
	ret = mira050pmic_write(client, 0x57, 0x02); // ledc ctrl1


	ret = mira050pmic_write(client, 0x4D, 0x01); // leda ctrl1
	ret = mira050pmic_write(client, 0x55, 0x10); // ledb ctrl7
	ret = mira050pmic_write(client, 0x5D, 0x10); // ledc ctrl7
	ret = mira050pmic_write(client, 0x61, 0x10); // led seq -- use this to turn on leds. abc0000- 1110000 for all leds

	usleep_range(2000000,2001000);

	return 0;
}

static int mira050pmic_check_hwcfg(struct device *dev)
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

static int mira050pmic_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct mira050pmic *mira050pmic;
	int ret;

	printk(KERN_INFO "[MIRA050PMIC]: probing pmic.\n");
	printk(KERN_INFO "[MIRA050PMIC]: Driver Version 0.0.\n");

	dev_err(dev, "[MIRA050PMIC] name: %s.\n", client->name);

	mira050pmic = devm_kzalloc(&client->dev, sizeof(*mira050pmic), GFP_KERNEL);
	if (!mira050pmic)
		return -ENOMEM;

	printk(KERN_INFO "[MIRA050PMIC]: Entering check hwcfg function.\n");

	/* Check the hardware configuration in device tree */
	if (mira050pmic_check_hwcfg(dev))
		return -EINVAL;

	// TODO(jalv): Get GPIO's, regulators and clocks from dts

	printk(KERN_INFO "[MIRA050PMIC]: Check xclk and freq.\n");

	/* Get system clock (xclk) */
	mira050pmic->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(mira050pmic->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(mira050pmic->xclk);
	}

	mira050pmic->xclk_freq = clk_get_rate(mira050pmic->xclk);
	if (mira050pmic->xclk_freq != MIRA050PMIC_SUPPORTED_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			mira050pmic->xclk_freq);
		return -EINVAL;
	}

	printk(KERN_INFO "[MIRA050PMIC]: Entering get regulators function.\n");

	ret = mira050pmic_get_regulators(client, mira050pmic);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}


	printk(KERN_INFO "[MIRA050PMIC]: Entering power on function.\n");

	/*
	 * The sensor must be powered for mira050_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = mira050pmic_power_on(dev, mira050pmic);
	if (ret)
		return ret;

	printk(KERN_INFO "[MIRA050PMIC]: Entering init controls function.\n");

	ret = mira050pmic_init_controls(client);
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

static int mira050pmic_remove(struct i2c_client *client)
{
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		mira050pmic_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

static const struct of_device_id mira050pmic_dt_ids[] = {
	{ .compatible = "ams,mira050pmic" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mira050pmic_dt_ids);

static const struct i2c_device_id mira050pmic_ids[] = {
	{ "pmic", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mira050pmic_ids);


static const struct dev_pm_ops mira050pmic_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mira050pmic_suspend, mira050pmic_resume)
	//SET_RUNTIME_PM_OPS(mira050pmic_power_off, mira050pmic_power_on, NULL)
};

static struct i2c_driver mira050pmic_i2c_driver = {
	.driver = {
		.name = "mira050pmic",
		.of_match_table	= mira050pmic_dt_ids,
		.pm = &mira050pmic_pm_ops,
	},
	.probe_new = mira050pmic_probe,
	.remove = mira050pmic_remove,
	.id_table = mira050pmic_ids,
};

module_i2c_driver(mira050pmic_i2c_driver);

MODULE_AUTHOR("Zhenyu Ye <zhenyu.ye@ams-osram.com>");
MODULE_DESCRIPTION("ams MIRA050PMIC sensor driver");
MODULE_LICENSE("GPL v2");
