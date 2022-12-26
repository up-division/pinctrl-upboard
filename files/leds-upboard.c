/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * UP Board CPLD/FPGA based LED driver
 *
 * Copyright (c) AAEON. All rights reserved.
 *
 * Author: Gary Wang <garywang@aaeon.com.tw>
 *
 */

#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "upboard-cpld.h"

struct upboard_led {
	struct regmap_field *field;
	struct led_classdev cdev;
	unsigned char bit;
};

static enum led_brightness upboard_led_brightness_get(struct led_classdev
						      *cdev)
{
	struct upboard_led *led = container_of(cdev, struct upboard_led, cdev);
	int brightness = 0;

	regmap_field_read(led->field, &brightness);

	return brightness;
};

static void upboard_led_brightness_set(struct led_classdev *cdev,
				       enum led_brightness brightness)
{
	struct upboard_led *led = container_of(cdev, struct upboard_led, cdev);

	regmap_field_write(led->field, brightness != LED_OFF);
};

static int __init upboard_led_probe(struct platform_device *pdev)
{
	struct upboard_fpga * const fpga = dev_get_drvdata(pdev->dev.parent);
	struct reg_field fldconf = {
		.reg = UPFPGA_REG_FUNC_EN0,
	};
	struct upboard_led_data * const pdata = pdev->dev.platform_data;
	struct upboard_led *led;

	if (!fpga || !pdata)
		return -EINVAL;

	led = devm_kzalloc(&pdev->dev, sizeof(*led), GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	fldconf.lsb = pdata->bit;
	fldconf.msb = pdata->bit;
	led->field = devm_regmap_field_alloc(&pdev->dev, fpga->regmap, fldconf);
	if (IS_ERR(led->field))
		return PTR_ERR(led->field);

	led->cdev.brightness_get = upboard_led_brightness_get;
	led->cdev.brightness_set = upboard_led_brightness_set;
	led->cdev.name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "upboard:%s:", pdata->colour);

	if (!led->cdev.name)
		return -ENOMEM;

	return devm_led_classdev_register(&pdev->dev, &led->cdev);
};

static struct platform_driver upboard_led_driver = {
	.driver = {
		.name = "upboard-led",
	},
};

module_platform_driver_probe(upboard_led_driver, upboard_led_probe);

MODULE_AUTHOR("Gary Wang <garywang@aaeon.com.tw>");
MODULE_DESCRIPTION("UP Board LED driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:upboard-led");
