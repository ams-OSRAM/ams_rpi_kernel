// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for ams poncha110 cameras.
 * Copyright (C) 2022, ams-OSRAM
 *
 * Based on Sony IMX219 camera driver
 * Copyright (C) 2019, Raspberry Pi (Trading) Ltd
 */

#include "poncha110.inl"

static const struct of_device_id poncha110_dt_ids[] = {
	{ .compatible = "ams,poncha110color" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, poncha110_dt_ids);

static const struct i2c_device_id poncha110_ids[] = {
	{ "poncha110color", 1 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, poncha110_ids);

static struct i2c_driver poncha110_i2c_driver = {
	.driver = {
		.name = "poncha110color",
		.of_match_table	= poncha110_dt_ids,
		.pm = &poncha110_pm_ops,
	},
	.probe_new = poncha110_probe,
	.remove = poncha110_remove,
	.id_table = poncha110_ids,
};

module_i2c_driver(poncha110_i2c_driver);

MODULE_AUTHOR("Zhenyu Ye <zhenyu.ye@ams-osram.com>");
MODULE_DESCRIPTION("ams poncha110 sensor driver");
MODULE_LICENSE("GPL v2");
