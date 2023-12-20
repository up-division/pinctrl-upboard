/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * UP Board CPLD/FPGA based LED driver
 *
 * Copyright (c) AAEON. All rights reserved.
 *
 * Author: Gary Wang <garywang@aaeon.com.tw>
 *
 */
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "upboard-cpld.h"

struct upboard_led {
	struct regmap_field *field;
	struct led_classdev cdev;
	unsigned char bit;
};

static struct gpio_led_platform_data upboard_gpio_led_pd;

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

/* GPIO LEDs */
int upboard_led_gpio_register(struct upboard_fpga *fpga)
{
	struct gpio_led blue_led,yellow_led,green_led,red_led;
	struct gpio_desc *desc;
	static struct gpio_led upboard_gpio_leds[4];
	int leds=0;
	
	desc = devm_gpiod_get(fpga->dev, "blue", GPIOD_OUT_LOW);
	if (!IS_ERR(desc)){
		blue_led.name = "upboard:blue:";
		blue_led.gpio = desc_to_gpio(desc);
		blue_led.default_state = LEDS_GPIO_DEFSTATE_KEEP;
		upboard_gpio_leds[leds++] = blue_led;	
		devm_gpiod_put(fpga->dev,desc);	
	}
	desc = devm_gpiod_get(fpga->dev, "yellow", GPIOD_OUT_LOW);
	if (!IS_ERR(desc)){
		yellow_led.name = "upboard:yellow:";
		yellow_led.gpio = desc_to_gpio(desc);
		yellow_led.default_state = LEDS_GPIO_DEFSTATE_KEEP;
		upboard_gpio_leds[leds++] = yellow_led;
		devm_gpiod_put(fpga->dev,desc);
	}
	desc = devm_gpiod_get(fpga->dev, "green", GPIOD_OUT_LOW);
	if (!IS_ERR(desc)){
		green_led.name = "upboard:green:";
		green_led.gpio = desc_to_gpio(desc);
		green_led.default_state = LEDS_GPIO_DEFSTATE_KEEP;
		upboard_gpio_leds[leds++] = green_led;
		devm_gpiod_put(fpga->dev,desc);
	}
	desc = devm_gpiod_get(fpga->dev, "red", GPIOD_OUT_LOW);
	if (!IS_ERR(desc)){
		red_led.name = "upboard:red:";
		red_led.gpio = desc_to_gpio(desc);
		red_led.default_state = LEDS_GPIO_DEFSTATE_KEEP;
		upboard_gpio_leds[leds++] = red_led;
		devm_gpiod_put(fpga->dev,desc);
	}
	
	/* no LEDs */
	if(leds==0)
		return -ENODEV;		
	
	upboard_gpio_led_pd.num_leds = leds;
	upboard_gpio_led_pd.leds = upboard_gpio_leds;
	
	static const struct mfd_cell upboard_gpio_led_cells[] = {
		{
			.name = "leds-gpio",
			.id = 0,
			.platform_data = &upboard_gpio_led_pd,
			.pdata_size = sizeof(upboard_gpio_led_pd),
		},
	};

	return devm_mfd_add_devices(fpga->dev, 0, upboard_gpio_led_cells,ARRAY_SIZE(upboard_gpio_led_cells), NULL, 0, NULL);
}

static int upboard_led_probe(struct platform_device *pdev)
{
	struct upboard_fpga * const fpga = dev_get_drvdata(pdev->dev.parent);
	struct reg_field fldconf = {
		.reg = UPFPGA_REG_FUNC_EN0,
	};
	struct upboard_led_data * const pdata = pdev->dev.platform_data;
	struct upboard_led *led;

	if (!fpga || !pdata)
		return -EINVAL;
		
	/* check & register GPIO LEDs */
	if(strstr(pdata->colour,"gpio"))
		return upboard_led_gpio_register(fpga);	

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
