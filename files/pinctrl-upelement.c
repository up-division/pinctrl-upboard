/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * UP Element EC pin controller driver
 *
 * Copyright (c) AAEON. All rights reserved.
 *
 * Author: Gary Wang <garywang@aaeon.com.tw>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 */

#include <linux/acpi.h>
#include <linux/dmi.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>
#include <linux/string.h>

#include "upboard-ec.h"

struct upelement_pin {
	struct regmap_field *dirbit;
	struct regmap_field *valbit;
	struct regmap_field *datbit;
};

struct upelement_pinctrl {
	struct gpio_chip chip;
	struct device *dev;
	struct pinctrl_dev *pctldev;
	struct pinctrl_desc *pctldesc;
	struct upelement_pin *pins;
	struct regmap *regmap;
};

#define UPBOARD_EC_PIN_NAME(id)			\
	{					\
		.number = UPEC_UP_##id,		\
		.name = #id,			\
	}
	
enum upboard_ec_pin_num {
	UPEC_UP_GND0,
	UPEC_UP_GPIO1,
	UPEC_UP_GPIO2,
	UPEC_UP_GPIO3,
	UPEC_UP_GPIO4,
	UPEC_UP_GPIO5,
	UPEC_UP_GPIO6,
	UPEC_UP_GPIO7,
	UPEC_UP_GPIO8,
	UPEC_UP_GND9,
	UPEC_UP_GND10,
	UPEC_UP_GPIO11,
	UPEC_UP_GPIO12,
	UPEC_UP_GPIO13,
	UPEC_UP_GPIO14,
	UPEC_UP_GPIO15,	
	UPEC_UP_GPIO16,	
	UPEC_UP_GPIO17,	
	UPEC_UP_GPIO18,	
	UPEC_UP_GND19,
};


static struct pinctrl_pin_desc upboard_ec_pins[] = {
	UPBOARD_EC_PIN_NAME(GND0),
	UPBOARD_EC_PIN_NAME(GPIO1),
	UPBOARD_EC_PIN_NAME(GPIO2),
	UPBOARD_EC_PIN_NAME(GPIO3),
	UPBOARD_EC_PIN_NAME(GPIO4),
	UPBOARD_EC_PIN_NAME(GPIO5),
	UPBOARD_EC_PIN_NAME(GPIO6),
	UPBOARD_EC_PIN_NAME(GPIO7),
	UPBOARD_EC_PIN_NAME(GPIO8),
	UPBOARD_EC_PIN_NAME(GND9),
	UPBOARD_EC_PIN_NAME(GND10),
	UPBOARD_EC_PIN_NAME(GPIO11),
	UPBOARD_EC_PIN_NAME(GPIO12),
	UPBOARD_EC_PIN_NAME(GPIO13),
	UPBOARD_EC_PIN_NAME(GPIO14),
	UPBOARD_EC_PIN_NAME(GPIO15),
	UPBOARD_EC_PIN_NAME(GPIO16),
	UPBOARD_EC_PIN_NAME(GPIO17),
	UPBOARD_EC_PIN_NAME(GPIO18),
	UPBOARD_EC_PIN_NAME(GND19),
};

//UP EC reg table
struct reg_field dirconf[ARRAY_SIZE(upboard_ec_pins)] = {
	{ .reg = 0, .lsb = 0, .msb = 0 },	      //GPIO0
	{ .reg = UPEC_REG_DIR1, .lsb = 0, .msb = 0 }, //GPIO1
	{ .reg = UPEC_REG_DIR1, .lsb = 2, .msb = 2 }, //GPIO2
	{ .reg = UPEC_REG_DIR1, .lsb = 4, .msb = 4 }, //GPIO3
	{ .reg = UPEC_REG_DIR1, .lsb = 6, .msb = 6 }, //GPIO4
	{ .reg = UPEC_REG_DIR2, .lsb = 0, .msb = 0 }, //GPIO5
	{ .reg = UPEC_REG_DIR2, .lsb = 2, .msb = 2 }, //GPIO6
	{ .reg = UPEC_REG_DIR2, .lsb = 4, .msb = 4 }, //GPIO7
	{ .reg = UPEC_REG_DIR2, .lsb = 6, .msb = 6 }, //GPIO8	
	{ .reg = 0, .lsb = 0, .msb = 0 },	      //GPIO9
	{ .reg = 0, .lsb = 0, .msb = 0 },	      //GPIO10
	{ .reg = UPEC_REG_DIR1, .lsb = 1, .msb = 1 }, //GPIO11
	{ .reg = UPEC_REG_DIR1, .lsb = 3, .msb = 3 }, //GPIO12
	{ .reg = UPEC_REG_DIR1, .lsb = 5, .msb = 5 }, //GPIO13
	{ .reg = UPEC_REG_DIR1, .lsb = 7, .msb = 7 }, //GPIO14	
	{ .reg = UPEC_REG_DIR2, .lsb = 1, .msb = 1 }, //GPIO15
	{ .reg = UPEC_REG_DIR2, .lsb = 3, .msb = 3 }, //GPIO16
	{ .reg = UPEC_REG_DIR2, .lsb = 5, .msb = 5 }, //GPIO17
	{ .reg = UPEC_REG_DIR2, .lsb = 7, .msb = 7 }, //GPIO18
	{ .reg = 0, .lsb = 0, .msb = 0 },	      //GPIO19
};

struct reg_field valconf[ARRAY_SIZE(upboard_ec_pins)] = {
	{ .reg = 0, .lsb = 0, .msb = 0 },
	{ .reg = UPEC_REG_VAL1, .lsb = 0, .msb = 0 },
	{ .reg = UPEC_REG_VAL1, .lsb = 2, .msb = 2 },
	{ .reg = UPEC_REG_VAL1, .lsb = 4, .msb = 4 },
	{ .reg = UPEC_REG_VAL1, .lsb = 6, .msb = 6 },
	{ .reg = UPEC_REG_VAL2, .lsb = 0, .msb = 0 },
	{ .reg = UPEC_REG_VAL2, .lsb = 2, .msb = 2 },
	{ .reg = UPEC_REG_VAL2, .lsb = 4, .msb = 4 },
	{ .reg = UPEC_REG_VAL2, .lsb = 6, .msb = 6 },
	{ .reg = 0, .lsb = 0, .msb = 0 },
	{ .reg = 0, .lsb = 0, .msb = 0 },	
	{ .reg = UPEC_REG_VAL1, .lsb = 1, .msb = 1 },
	{ .reg = UPEC_REG_VAL1, .lsb = 3, .msb = 3 },
	{ .reg = UPEC_REG_VAL1, .lsb = 5, .msb = 5 },
	{ .reg = UPEC_REG_VAL1, .lsb = 7, .msb = 7 },	
	{ .reg = UPEC_REG_VAL2, .lsb = 1, .msb = 1 },
	{ .reg = UPEC_REG_VAL2, .lsb = 3, .msb = 3 },
	{ .reg = UPEC_REG_VAL2, .lsb = 5, .msb = 5 },
	{ .reg = UPEC_REG_VAL2, .lsb = 7, .msb = 7 },
	{ .reg = 0, .lsb = 0, .msb = 0 },
};	

struct reg_field datconf[ARRAY_SIZE(upboard_ec_pins)] = {
	{ .reg = 0, .lsb = 0, .msb = 0 },
	{ .reg = UPEC_REG_DAT1, .lsb = 0, .msb = 0 }, //KSO8
	{ .reg = UPEC_REG_DAT1, .lsb = 1, .msb = 1 }, //KSO9
	{ .reg = UPEC_REG_DAT1, .lsb = 4, .msb = 4 }, //KSO12
	{ .reg = UPEC_REG_DAT1, .lsb = 5, .msb = 5 }, //KSO13
	{ .reg = UPEC_REG_DAT2, .lsb = 0, .msb = 0 }, //KSI0
	{ .reg = UPEC_REG_DAT2, .lsb = 1, .msb = 1 }, //KSI1
	{ .reg = UPEC_REG_DAT_GP15, .lsb = 0, .msb = 0 }, //GPG0
	{ .reg = UPEC_REG_DAT_GP17, .lsb = 7, .msb = 7 }, //GPD7
	{ .reg = 0, .lsb = 0, .msb = 0 },
	{ .reg = 0, .lsb = 0, .msb = 0 },	
	{ .reg = UPEC_REG_DAT1, .lsb = 2, .msb = 2 }, //KSO10
	{ .reg = UPEC_REG_DAT1, .lsb = 3, .msb = 3 }, //KSO11
	{ .reg = UPEC_REG_DAT1, .lsb = 6, .msb = 6 }, //KSO14
	{ .reg = UPEC_REG_DAT1, .lsb = 7, .msb = 7 }, //KSO15
	{ .reg = UPEC_REG_DAT2, .lsb = 2, .msb = 2 }, //KSI2
	{ .reg = UPEC_REG_DAT2, .lsb = 3, .msb = 3 }, //KSI3
	{ .reg = UPEC_REG_DAT_GP16, .lsb = 0, .msb = 0 }, //GPA0
	{ .reg = UPEC_REG_DAT_GP18, .lsb = 7, .msb = 7 }, //GPE7
	{ .reg = 0, .lsb = 0, .msb = 0 },
};		

static int upboard_get_groups_count(struct pinctrl_dev *pctldev)
{
	return 0;
}

static const char *upboard_get_group_name(struct pinctrl_dev *pctldev,
					  unsigned int selector)
{
	return NULL;
}

static const struct pinctrl_ops upboard_pinctrl_ops = {
	.get_groups_count = upboard_get_groups_count,
	.get_group_name = upboard_get_group_name,
};

static struct pinctrl_desc upboard_ec_pinctrl_desc = {
	.pins = upboard_ec_pins,
	.npins = ARRAY_SIZE(upboard_ec_pins),
	.pctlops = &upboard_pinctrl_ops,
	.owner = THIS_MODULE,
};

static int upboard_ec_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct upelement_pinctrl *pctrl = container_of(gc, struct upelement_pinctrl, chip); 
	struct upelement_pin pin = pctrl->pins[offset];
	int val=-1;
	
	regmap_field_read(pin.datbit, &val);

	return val;
}

static void upboard_ec_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct upelement_pinctrl *pctrl = container_of(gc, struct upelement_pinctrl, chip); 
	struct upelement_pin pin = pctrl->pins[offset];
	regmap_field_write(pin.valbit, value);
}

static int upboard_ec_gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	struct upelement_pinctrl *pctrl = container_of(gc, struct upelement_pinctrl, chip); 
	struct upelement_pin pin = pctrl->pins[offset];
	return regmap_field_write(pin.dirbit, 0);
}

static int upboard_ec_gpio_direction_output(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct upelement_pinctrl *pctrl = container_of(gc, struct upelement_pinctrl, chip); 
	struct upelement_pin pin = pctrl->pins[offset];
	
	regmap_field_write(pin.dirbit, 1);
	return regmap_field_write(pin.valbit, value);
}

static int upboard_ec_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct upelement_pinctrl *pctrl = container_of(gc, struct upelement_pinctrl, chip); 
	struct upelement_pin pin = pctrl->pins[offset];
	int val=-1;
	
	regmap_field_read(pin.dirbit, &val);
	return ~val;	
}

static int upboard_ec_gpio_request(struct gpio_chip *gc, unsigned int offset)
{
	if(offset==0 || offset==9 || offset==10 || offset==19) //GND pin
		return -ENODEV;

	return 0;
}

static struct gpio_chip upboard_ec_gpio_chip = {
	.label = "UP element GPIO controller",
	.base = 0,
	.request = upboard_ec_gpio_request,
	.get = upboard_ec_gpio_get,
	.set = upboard_ec_gpio_set,
	.direction_input = upboard_ec_gpio_direction_input,
	.direction_output = upboard_ec_gpio_direction_output,
	.get_direction = upboard_ec_gpio_get_direction,
	.owner = THIS_MODULE,
};

static int __init upboard_pinctrl_probe(struct platform_device *pdev)
{
	struct upboard_ec * const ec = dev_get_drvdata(pdev->dev.parent);
	struct acpi_device * const adev = ACPI_COMPANION(&pdev->dev);
	struct pinctrl_desc *pctldesc;
	struct upelement_pinctrl *pctrl;
	struct upelement_pin *pins;
	const char *hid;
	int i,ret=0;

	if (!ec)
		return -EINVAL;

	if (!adev)
		return -ENODEV;

	hid = acpi_device_hid(adev);
	if (!strcmp(hid, "PNP0C09")) {
		pctldesc = &upboard_ec_pinctrl_desc;
	} else
		return -ENODEV;

	pctldesc->name = dev_name(&pdev->dev);

	pins = devm_kzalloc(&pdev->dev,
			    sizeof(*pins) * pctldesc->npins,
			    GFP_KERNEL);
	if (!pins)
		return -ENOMEM;
		
	/* initialise pins */
	for (i = 0; i < pctldesc->npins; i++) {
		struct upelement_pin *pin = &pins[i];
		struct pinctrl_pin_desc *pd = (struct pinctrl_pin_desc *)&pctldesc->pins[i];

		pin->dirbit = devm_regmap_field_alloc(&pdev->dev, ec->regmap, dirconf[i]);
		if (IS_ERR(pin->dirbit))
			return PTR_ERR(pin->dirbit);
				
		pin->valbit = devm_regmap_field_alloc(&pdev->dev, ec->regmap, valconf[i]);
		if (IS_ERR(pin->valbit))
			return PTR_ERR(pin->valbit);

		pin->datbit = devm_regmap_field_alloc(&pdev->dev, ec->regmap, datconf[i]);
		if (IS_ERR(pin->valbit))
			return PTR_ERR(pin->valbit);
						
		pd->drv_data = pin;	
	}
		
	/* create a new pinctrl device and register it */
	pctrl = devm_kzalloc(&pdev->dev, sizeof(*pctrl), GFP_KERNEL);
	if (!pctrl)
		return -ENOMEM;

	pctrl->chip = upboard_ec_gpio_chip;
	pctrl->regmap = ec->regmap;
	pctrl->pctldesc = pctldesc;
	pctrl->chip.parent = &pdev->dev;
	pctrl->chip.ngpio = pctldesc->npins;
	pctrl->pins = pins;

	ret = devm_gpiochip_add_data(&pdev->dev, &pctrl->chip, pctrl);
	if (ret)
		return ret;

	pctrl->pctldev = devm_pinctrl_register(&pdev->dev, pctldesc, pctrl);
	if (IS_ERR(pctrl->pctldev))
		return PTR_ERR(pctrl->pctldev);
	platform_set_drvdata(pdev, pctrl);
	
	return ret;
}

static struct platform_driver upboard_pinctrl_driver = {
	.driver = {
		.name = "upelement-pinctrl",
	},
};

module_platform_driver_probe(upboard_pinctrl_driver, upboard_pinctrl_probe);

MODULE_AUTHOR("Gary Wang <garywang@aaeon.com.tw>");
MODULE_DESCRIPTION("UP Element pin controller driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:upelement-pinctrl");
