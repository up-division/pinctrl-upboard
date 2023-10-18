// SPDX-License-Identifier: GPL-2.0-only
/*
 * UP Board multi function device driver for control CPLD/FPGA to
 * provide more GPIO driving power also provide CPLD LEDs and pin mux function
 * recognize HID AANT0F00 ~ AAANT0F04 in ACPI name space
 *
 * Copyright (c) AAEON. All rights reserved.
 *
 * Author: Gary Wang <garywang@aaeon.com.tw>
 *
 */

#include <linux/acpi.h>
#include <linux/dmi.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "upboard-cpld.h"

#define UPBOARD_LED_CELL(led_data, n)                   \
	{                                               \
		.name = "upboard-led",                  \
		.id = (n),                              \
		.platform_data = &led_data[(n)],        \
		.pdata_size = sizeof(*(led_data)),      \
	}

#define AAEON_MANUFACTURER_ID		0x01
#define SUPPORTED_FW_MAJOR		0x0
#define MENUFACTURER_ID_MASK		GENMASK(7, 0)

#define FIRMWARE_ID_BUILD_OFFSET	(12)
#define FIRMWARE_ID_MAJOR_OFFSET	(8)
#define FIRMWARE_ID_MINOR_OFFSET	(4)
#define FIRMWARE_ID_PATCH_OFFSET	(0)
#define FIRMWARE_ID_MASK		GENMASK(3, 0)

/*
 * read CPLD register on custom protocol
 * send clock and addr bit in strobe and datain pins then read from dataout pin
 */
static int upboard_cpld_read(void *context, unsigned int reg, unsigned int *val)
{
	struct upboard_fpga * const fpga = context;
	int i;
	
	if (IS_ERR(fpga->clear_gpio))	//for none fpga boards
		return 0;
		
	/* clear to start new transcation */
	gpiod_set_value(fpga->clear_gpio, 0);
	gpiod_set_value(fpga->clear_gpio, 1);

	reg |= UPFPGA_READ_FLAG;

	/* send clock and data from strobe & datain */
	for (i = UPFPGA_ADDRESS_SIZE; i >= 0; i--) {
		gpiod_set_value(fpga->strobe_gpio, 0);
		gpiod_set_value(fpga->datain_gpio, !!(reg & BIT(i)));
		gpiod_set_value(fpga->strobe_gpio, 1);
	}

	gpiod_set_value(fpga->strobe_gpio, 0);
	*val = 0;

	/* read from dataout */
	for (i = UPFPGA_REGISTER_SIZE - 1; i >= 0; i--) {
		gpiod_set_value(fpga->strobe_gpio, 1);
		gpiod_set_value(fpga->strobe_gpio, 0);
		*val |= gpiod_get_value(fpga->dataout_gpio) << i;
	}

	gpiod_set_value(fpga->strobe_gpio, 1);

	return 0;
}

/*
 * write CPLD register on custom protocol
 * send clock and addr bit in strobe and datain pins then write to datain pin
 */
static int upboard_cpld_write(void *context, unsigned int reg, unsigned int val)
{
	struct upboard_fpga * const fpga = context;
	int i;
	
	if (IS_ERR(fpga->clear_gpio))	//for none fpga boards
		return 0;

	/* clear to start new transcation */
	gpiod_set_value(fpga->clear_gpio, 0);
	gpiod_set_value(fpga->clear_gpio, 1);

	/* send clock and data from strobe & datain */
	for (i = UPFPGA_ADDRESS_SIZE; i >= 0; i--) {
		gpiod_set_value(fpga->strobe_gpio, 0);
		gpiod_set_value(fpga->datain_gpio, !!(reg & BIT(i)));
		gpiod_set_value(fpga->strobe_gpio, 1);
	}

	gpiod_set_value(fpga->strobe_gpio, 0);

	/* write to datain pin */
	for (i = UPFPGA_REGISTER_SIZE - 1; i >= 0; i--) {
		gpiod_set_value(fpga->datain_gpio, !!(val & BIT(i)));
		gpiod_set_value(fpga->strobe_gpio, 1);
		gpiod_set_value(fpga->strobe_gpio, 0);
	}

	gpiod_set_value(fpga->strobe_gpio, 1);

	return 0;
}

/* UP board */
static const struct regmap_range upboard_up_readable_ranges[] = {
	regmap_reg_range(UPFPGA_REG_PLATFORM_ID, UPFPGA_REG_FIRMWARE_ID),
	regmap_reg_range(UPFPGA_REG_FUNC_EN0, UPFPGA_REG_FUNC_EN0),
	regmap_reg_range(UPFPGA_REG_GPIO_DIR0, UPFPGA_REG_GPIO_DIR1),
};

static const struct regmap_range upboard_up_writable_ranges[] = {
	regmap_reg_range(UPFPGA_REG_FUNC_EN0, UPFPGA_REG_FUNC_EN0),
	regmap_reg_range(UPFPGA_REG_GPIO_DIR0, UPFPGA_REG_GPIO_DIR1),
};

static const struct regmap_access_table upboard_up_readable_table = {
	.yes_ranges = upboard_up_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(upboard_up_readable_ranges),
};

static const struct regmap_access_table upboard_up_writable_table = {
	.yes_ranges = upboard_up_writable_ranges,
	.n_yes_ranges = ARRAY_SIZE(upboard_up_writable_ranges),
};

static const struct regmap_config upboard_up_regmap_config = {
	.reg_bits = UPFPGA_ADDRESS_SIZE,
	.val_bits = UPFPGA_REGISTER_SIZE,
	.max_register = UPFPGA_REG_MAX,
	.reg_read = upboard_cpld_read,
	.reg_write = upboard_cpld_write,
	.fast_io = false,
	.cache_type = REGCACHE_NONE,
	.rd_table = &upboard_up_readable_table,
	.wr_table = &upboard_up_writable_table,
};

static struct upboard_led_data upboard_gpio_led_data[] = {
	{ .bit = 0, .colour = "gpio" },
};

static struct upboard_led_data upboard_up_led_data[] = {
	{ .bit = 0, .colour = "yellow" },
	{ .bit = 1, .colour = "green" },
	{ .bit = 2, .colour = "red" },
};

static const struct mfd_cell upboard_up_mfd_cells[] = {
	{ .name = "upboard-pinctrl" },
	UPBOARD_LED_CELL(upboard_up_led_data, 0),
	UPBOARD_LED_CELL(upboard_up_led_data, 1),
	UPBOARD_LED_CELL(upboard_up_led_data, 2),
	UPBOARD_LED_CELL(upboard_gpio_led_data, 0),
};

/* UP Squared 6000 EHL board */
static const struct mfd_cell upboard_pinctrl_cells[] = {
	{ .name = "upboard-pinctrl" },
	UPBOARD_LED_CELL(upboard_gpio_led_data, 0),
};

/* UP^2 board */
static const struct regmap_range upboard_up2_readable_ranges[] = {
	regmap_reg_range(UPFPGA_REG_PLATFORM_ID, UPFPGA_REG_FIRMWARE_ID),
	regmap_reg_range(UPFPGA_REG_FUNC_EN0, UPFPGA_REG_FUNC_EN1),
	regmap_reg_range(UPFPGA_REG_GPIO_EN0, UPFPGA_REG_GPIO_EN2),
	regmap_reg_range(UPFPGA_REG_GPIO_DIR0, UPFPGA_REG_GPIO_DIR2),
};

static const struct regmap_range upboard_up2_writable_ranges[] = {
	regmap_reg_range(UPFPGA_REG_FUNC_EN0, UPFPGA_REG_FUNC_EN1),
	regmap_reg_range(UPFPGA_REG_GPIO_EN0, UPFPGA_REG_GPIO_EN2),
	regmap_reg_range(UPFPGA_REG_GPIO_DIR0, UPFPGA_REG_GPIO_DIR2),
};

static const struct regmap_access_table upboard_up2_readable_table = {
	.yes_ranges = upboard_up2_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(upboard_up2_readable_ranges),
};

static const struct regmap_access_table upboard_up2_writable_table = {
	.yes_ranges = upboard_up2_writable_ranges,
	.n_yes_ranges = ARRAY_SIZE(upboard_up2_writable_ranges),
};

static const struct regmap_config upboard_up2_regmap_config = {
	.reg_bits = UPFPGA_ADDRESS_SIZE,
	.val_bits = UPFPGA_REGISTER_SIZE,
	.max_register = UPFPGA_REG_MAX,
	.reg_read = upboard_cpld_read,
	.reg_write = upboard_cpld_write,
	.fast_io = false,
	.cache_type = REGCACHE_NONE,
	.rd_table = &upboard_up2_readable_table,
	.wr_table = &upboard_up2_writable_table,
};

static struct upboard_led_data upboard_up2_led_data[] = {
	{ .bit = 0, .colour = "blue" },
	{ .bit = 1, .colour = "yellow" },
	{ .bit = 2, .colour = "green" },
	{ .bit = 3, .colour = "red" },
};

static const struct mfd_cell upboard_up2_mfd_cells[] = {
	{ .name = "upboard-pinctrl" },
 	UPBOARD_LED_CELL(upboard_up2_led_data, 0),
 	UPBOARD_LED_CELL(upboard_up2_led_data, 1),
 	UPBOARD_LED_CELL(upboard_up2_led_data, 2),
 	UPBOARD_LED_CELL(upboard_up2_led_data, 3),
	UPBOARD_LED_CELL(upboard_gpio_led_data,0),

};

static int __init upboard_cpld_gpio_init(struct upboard_fpga *fpga)
{
	enum gpiod_flags flags = fpga->uninitialised ? GPIOD_OUT_LOW : GPIOD_ASIS;
	
	fpga->enable_gpio = devm_gpiod_get(fpga->dev, "enable", flags);
	if (IS_ERR(fpga->enable_gpio))
		return PTR_ERR(fpga->enable_gpio);

	fpga->clear_gpio = devm_gpiod_get(fpga->dev, "clear", GPIOD_OUT_LOW);
	if (IS_ERR(fpga->clear_gpio))
		return PTR_ERR(fpga->clear_gpio);

	fpga->strobe_gpio = devm_gpiod_get(fpga->dev, "strobe", GPIOD_OUT_LOW);
	if (IS_ERR(fpga->strobe_gpio))
		return PTR_ERR(fpga->strobe_gpio);

	fpga->datain_gpio = devm_gpiod_get(fpga->dev, "datain", GPIOD_OUT_LOW);
	if (IS_ERR(fpga->datain_gpio))
		return PTR_ERR(fpga->datain_gpio);

	fpga->dataout_gpio = devm_gpiod_get(fpga->dev, "dataout", GPIOD_IN);
	if (IS_ERR(fpga->dataout_gpio))
		return PTR_ERR(fpga->dataout_gpio);

	/* The SoC pinctrl driver may not support reserving the GPIO line for
	 * FPGA reset without causing an undesired reset pulse. This will clear
	 * any settings on the FPGA, so only do it if we must.
	 */
	if (fpga->uninitialised) {
		fpga->reset_gpio = devm_gpiod_get(fpga->dev, "reset", GPIOD_OUT_LOW);
		if (IS_ERR(fpga->reset_gpio))
			return PTR_ERR(fpga->reset_gpio);

		gpiod_set_value(fpga->reset_gpio, 1);
	}

	gpiod_set_value(fpga->enable_gpio, 1);
	fpga->uninitialised = false;

	return 0;
}

/*
 * This function is for debugging with user for showing firmware information.
 */
static int __init upboard_fpga_show_firmware_info(struct upboard_fpga *fpga)
{
	unsigned int platform_id, manufacturer_id;
	unsigned int firmware_id, build, major, minor, patch;
	int ret;

	if (!fpga)
		return -ENOMEM;

	ret = regmap_read(fpga->regmap, UPFPGA_REG_PLATFORM_ID, &platform_id);
	if (ret)
		return ret;

	manufacturer_id = platform_id & MENUFACTURER_ID_MASK;
	if (manufacturer_id != AAEON_MANUFACTURER_ID) {
		dev_err(fpga->dev,
			"driver not compatible with custom FPGA FW from manufacturer id 0x%02x. Exiting",
			manufacturer_id);

		return -ENODEV;
	}

	ret = regmap_read(fpga->regmap, UPFPGA_REG_FIRMWARE_ID, &firmware_id);
	if (ret)
		return ret;

	build = (firmware_id >> FIRMWARE_ID_BUILD_OFFSET) & FIRMWARE_ID_MASK;
	major = (firmware_id >> FIRMWARE_ID_MAJOR_OFFSET) & FIRMWARE_ID_MASK;
	minor = (firmware_id >> FIRMWARE_ID_MINOR_OFFSET) & FIRMWARE_ID_MASK;
	patch = (firmware_id >> FIRMWARE_ID_PATCH_OFFSET) & FIRMWARE_ID_MASK;

	if (major != SUPPORTED_FW_MAJOR) {
		dev_err(fpga->dev, "unsupported FPGA FW v%u.%u.%u build 0x%02x",
			major, minor, patch, build);

		return -ENODEV;
	}

	dev_info(fpga->dev, "compatible FPGA FW v%u.%u.%u build 0x%02x",
		 major, minor, patch, build);

	return 0;
}

static const struct acpi_device_id upboard_fpga_acpi_match[] = {
	{ "AANT0000", AANT0000_ID },
	{ "AANT0F00", AANT0F00_ID },
	{ "AANT0F01", AANT0F01_ID },
	{ "AANT0F02", AANT0F02_ID },
	{ "AANT0F03", AANT0F03_ID },
	{ "AANT0F04", AANT0F04_ID },
	{ }
};
MODULE_DEVICE_TABLE(acpi, upboard_fpga_acpi_match);

static int __init upboard_fpga_probe(struct platform_device *pdev)
{
	struct upboard_fpga *fpga;
	const struct acpi_device_id *id;
	static const struct regmap_config *cpld_config;
	static const struct mfd_cell *cells;
	static size_t ncells;
	int ret;

	id = acpi_match_device(upboard_fpga_acpi_match, &pdev->dev);
	if (!id)
		return -ENODEV;

	switch(id->driver_data)
	{
		case AANT0F00_ID:
		cpld_config = &upboard_up_regmap_config;
		cells = upboard_up_mfd_cells;
		ncells = ARRAY_SIZE(upboard_up_mfd_cells);
		break;
		case AANT0F01_ID:
		cpld_config = &upboard_up2_regmap_config;
		cells = upboard_up2_mfd_cells;
		ncells = ARRAY_SIZE(upboard_up2_mfd_cells);
		break;
		case AANT0F02_ID:
		cpld_config = &upboard_up_regmap_config;
		cells = upboard_up_mfd_cells;
		ncells = ARRAY_SIZE(upboard_up_mfd_cells);
		break;
		case AANT0F03_ID:
		cpld_config = &upboard_up2_regmap_config;
		cells = upboard_up_mfd_cells;
		ncells = ARRAY_SIZE(upboard_up_mfd_cells);
		break;
		case AANT0F04_ID:
		cpld_config = &upboard_up_regmap_config;
		cells = upboard_up_mfd_cells;
		ncells = ARRAY_SIZE(upboard_up_mfd_cells);
		break;
		case AANT0000_ID:
		default:
		cpld_config = &upboard_up_regmap_config;
		cells = upboard_pinctrl_cells;
		ncells = ARRAY_SIZE(upboard_pinctrl_cells);
		break;
	}
	
	fpga = devm_kzalloc(&pdev->dev, sizeof(*fpga), GFP_KERNEL);
	if (!fpga)
		return -ENOMEM;

	//force init once
	//fpga->uninitialised = true;

	platform_set_drvdata(pdev, fpga);
	fpga->dev = &pdev->dev;
	fpga->regmap = devm_regmap_init(&pdev->dev, NULL, fpga, cpld_config);

	if (IS_ERR(fpga->regmap))
		return PTR_ERR(fpga->regmap);

	ret = upboard_cpld_gpio_init(fpga);
	if (ret) {
		/*
		 * This is for compatiable with some upboards w/o FPGA firmware,
		 * so just showing debug info and do not return directly.
		 */
		dev_warn(&pdev->dev, "Failed to initialize CPLD common GPIOs: %d", ret);
	} else {
		upboard_fpga_show_firmware_info(fpga);
	}
	
	return devm_mfd_add_devices(&pdev->dev, PLATFORM_DEVID_AUTO,
				    cells,
				    ncells,
				    NULL, 0, NULL);
}

static struct platform_driver upboard_fpga_driver = {
	.driver = {
		.name = "upboard-cpld",
		.acpi_match_table = upboard_fpga_acpi_match,
	},
};

module_platform_driver_probe(upboard_fpga_driver, upboard_fpga_probe);

MODULE_AUTHOR("Gary Wang <garywang@aaeon.com.tw>");
MODULE_DESCRIPTION("UP Board CPLD/FPGA driver");
MODULE_LICENSE("GPL v2");
