/*
 * UP Board main platform driver and FPGA configuration support
 *
 * Copyright (c) 2017, Emutex Ltd. All rights reserved.
 *
 * Author: Javier Arteaga <javier@emutex.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/acpi.h>
#include <linux/dmi.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "upboard-fpga.h"

int upboard_fpga_read(void *, unsigned int, unsigned int *);
int upboard_fpga_write(void *, unsigned int, unsigned int);

struct upboard_fpga_data {
	const struct regmap_config *regmapconf;
	const struct mfd_cell *cells;
	size_t ncells;
};

#define UPBOARD_LED_CELL(led_data, n)                   \
	{                                               \
		.name = "upboard-led",                  \
		.id = (n),                              \
		.platform_data = &led_data[(n)],        \
		.pdata_size = sizeof(*(led_data)),      \
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
	.reg_read = upboard_fpga_read,
	.reg_write = upboard_fpga_write,
	.fast_io = false,
	.cache_type = REGCACHE_RBTREE,
	.rd_table = &upboard_up_readable_table,
	.wr_table = &upboard_up_writable_table,
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
};

static const struct upboard_fpga_data upboard_up_fpga_data = {
	.regmapconf = &upboard_up_regmap_config,
	.cells = upboard_up_mfd_cells,
	.ncells = ARRAY_SIZE(upboard_up_mfd_cells),
};

//UP-EHL
static const struct mfd_cell upboard_pinctrl_cells[] = {
	{ .name = "upboard-pinctrl" },
};

static const struct upboard_fpga_data upboard_pinctrl_data = {
	.regmapconf = &upboard_up_regmap_config,
	.cells = upboard_pinctrl_cells,
	.ncells = ARRAY_SIZE(upboard_pinctrl_cells),
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
	.reg_read = upboard_fpga_read,
	.reg_write = upboard_fpga_write,
	.fast_io = false,
//	.cache_type = REGCACHE_NONE,
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
};

static const struct upboard_fpga_data upboard_up2_fpga_data = {
	.regmapconf = &upboard_up2_regmap_config,
	.cells = upboard_up2_mfd_cells,
	.ncells = ARRAY_SIZE(upboard_up2_mfd_cells),
};

/* UP-CREX carrier board for UP Core */

/* same MAXV config as UP1 (proto2 release) */
#define upboard_upcore_crex_fpga_data upboard_up_fpga_data

/* UP-CRST02 carrier board for UP Core */

/* same MAX10 config as UP2, but same LED cells as UP1 */
static const struct upboard_fpga_data upboard_upcore_crst02_fpga_data = {
	.regmapconf = &upboard_up2_regmap_config,
	.cells = upboard_up_mfd_cells,
	.ncells = ARRAY_SIZE(upboard_up_mfd_cells),
};

int upboard_fpga_read(void *context, unsigned int reg, unsigned int *val)
{
	struct upboard_fpga * const fpga = context;
	int i;
	
	if (IS_ERR(fpga->clear_gpio))	//for none fpga boards
		return 0;
		
	gpiod_set_value(fpga->clear_gpio, 0);
	gpiod_set_value(fpga->clear_gpio, 1);

	reg |= UPFPGA_READ_FLAG;

	for (i = UPFPGA_ADDRESS_SIZE; i >= 0; i--) {
		gpiod_set_value(fpga->strobe_gpio, 0);
		gpiod_set_value(fpga->datain_gpio, (reg >> i) & 0x1);
		gpiod_set_value(fpga->strobe_gpio, 1);
	}

	gpiod_set_value(fpga->strobe_gpio, 0);
	*val = 0;

	for (i = UPFPGA_REGISTER_SIZE - 1; i >= 0; i--) {
		gpiod_set_value(fpga->strobe_gpio, 1);
		gpiod_set_value(fpga->strobe_gpio, 0);
		*val |= gpiod_get_value(fpga->dataout_gpio) << i;
	}

	gpiod_set_value(fpga->strobe_gpio, 1);

	return 0;
}


int upboard_fpga_write(void *context, unsigned int reg, unsigned int val)
{
	struct upboard_fpga * const fpga = context;
	int i;
	
	if (IS_ERR(fpga->clear_gpio))	//for none fpga boards
		return 0;

	gpiod_set_value(fpga->clear_gpio, 0);
	gpiod_set_value(fpga->clear_gpio, 1);

	for (i = UPFPGA_ADDRESS_SIZE; i >= 0; i--) {
		gpiod_set_value(fpga->strobe_gpio, 0);
		gpiod_set_value(fpga->datain_gpio, (reg >> i) & 0x1);
		gpiod_set_value(fpga->strobe_gpio, 1);
	}

	gpiod_set_value(fpga->strobe_gpio, 0);

	for (i = UPFPGA_REGISTER_SIZE - 1; i >= 0; i--) {
		gpiod_set_value(fpga->datain_gpio, (val >> i) & 0x1);
		gpiod_set_value(fpga->strobe_gpio, 1);
		gpiod_set_value(fpga->strobe_gpio, 0);
	}

	gpiod_set_value(fpga->strobe_gpio, 1);

	return 0;
}

static int __init upboard_fpga_gpio_init(struct upboard_fpga *fpga)
{
	enum gpiod_flags flags;

	flags = fpga->uninitialised ? GPIOD_OUT_LOW : GPIOD_ASIS;
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
		fpga->reset_gpio = devm_gpiod_get(fpga->dev, "reset",
						  GPIOD_OUT_LOW);
		if (IS_ERR(fpga->reset_gpio))
			return PTR_ERR(fpga->reset_gpio);

		gpiod_set_value(fpga->reset_gpio, 1);
	}

	gpiod_set_value(fpga->enable_gpio, 1);
	fpga->uninitialised = false;

	return 0;
}

static int __init upboard_fpga_detect_firmware(struct upboard_fpga *fpga)
{
	const unsigned int AAEON_MANUFACTURER_ID = 0x01;
	const unsigned int SUPPORTED_FW_MAJOR = 0x0;
	unsigned int platform_id, manufacturer_id;
	unsigned int firmware_id, build, major, minor, patch;
	int ret;

	if (!fpga)
		return -ENOMEM;

	ret = regmap_read(fpga->regmap, UPFPGA_REG_PLATFORM_ID, &platform_id);
	if (ret)
		return ret;

	manufacturer_id = platform_id & 0xff;
	if (manufacturer_id != AAEON_MANUFACTURER_ID) {
		dev_dbg(fpga->dev,
			"driver not compatible with custom FPGA FW from manufacturer id 0x%02x. Exiting",
			manufacturer_id);
		return -ENODEV;
	}

	ret = regmap_read(fpga->regmap, UPFPGA_REG_FIRMWARE_ID, &firmware_id);
	if (ret)
		return ret;

	build = (firmware_id >> 12) & 0xf;
	major = (firmware_id >> 8) & 0xf;
	minor = (firmware_id >> 4) & 0xf;
	patch = firmware_id & 0xf;
	if (major != SUPPORTED_FW_MAJOR) {
		dev_dbg(fpga->dev, "unsupported FPGA FW v%u.%u.%u build 0x%02x",
			major, minor, patch, build);
		return -ENODEV;
	}

	dev_info(fpga->dev, "compatible FPGA FW v%u.%u.%u build 0x%02x",
		 major, minor, patch, build);
		 			 
	return 0;
}

void upboard_led_gpio_init(struct upboard_fpga *fpga)
{
	struct gpio_led blue_led,yellow_led,green_led,red_led;
	struct gpio_desc *desc;
	int blue_gpio=-1,yellow_gpio=-1,green_gpio=-1,red_gpio=-1,leds=0;
	
	desc = devm_gpiod_get(fpga->dev, "blue", GPIOD_OUT_LOW);
	if (!IS_ERR(desc)){
		blue_gpio = desc_to_gpio(desc);
		leds++;
		devm_gpiod_put(fpga->dev,desc);
	}
	desc = devm_gpiod_get(fpga->dev, "yellow", GPIOD_OUT_LOW);
	if (!IS_ERR(desc)){
		yellow_gpio = desc_to_gpio(desc);
		leds++;
		devm_gpiod_put(fpga->dev,desc);
	}
	desc = devm_gpiod_get(fpga->dev, "green", GPIOD_OUT_LOW);
	if (!IS_ERR(desc)){
		green_gpio = desc_to_gpio(desc);
		leds++;
		devm_gpiod_put(fpga->dev,desc);
	}
	desc = devm_gpiod_get(fpga->dev, "red", GPIOD_OUT_LOW);
	if (!IS_ERR(desc)){
		red_gpio = desc_to_gpio(desc);
		leds++;
		devm_gpiod_put(fpga->dev,desc);
	}
	
	if(leds==0)	//no leds 
		return;		
		
	static struct gpio_led upboard_gpio_leds[8];
	
	leds=0;
	if(blue_gpio>-1){
		blue_led.name = "upboard:blue:";
		blue_led.gpio = blue_gpio;
		blue_led.default_state = LEDS_GPIO_DEFSTATE_KEEP;
		upboard_gpio_leds[leds++] = blue_led;
	}
	if(yellow_gpio>-1){
		yellow_led.name = "upboard:yellow:";
		yellow_led.gpio = yellow_gpio;
		yellow_led.default_state = LEDS_GPIO_DEFSTATE_KEEP;
		upboard_gpio_leds[leds++] = yellow_led;
	}
	if(green_gpio>-1){
		green_led.name = "upboard:green:";
		green_led.gpio = green_gpio;
		green_led.default_state = LEDS_GPIO_DEFSTATE_KEEP;
		upboard_gpio_leds[leds++] = green_led;
	}
	if(red_gpio>-1){
		red_led.name = "upboard:red:";
		red_led.gpio = red_gpio;
		red_led.default_state = LEDS_GPIO_DEFSTATE_KEEP;
		upboard_gpio_leds[leds++] = red_led;
	}
	
	static struct gpio_led_platform_data upboard_gpio_led_platform_data;
	upboard_gpio_led_platform_data.num_leds = leds;
	upboard_gpio_led_platform_data.leds = upboard_gpio_leds;
	
	static const struct mfd_cell upboard_gpio_led_cells[] = {
		{
			.name = "leds-gpio",
			.id = 0,
			.platform_data = &upboard_gpio_led_platform_data,
			.pdata_size = sizeof(upboard_gpio_led_platform_data),
		},
	};

	if(devm_mfd_add_devices(fpga->dev, 0, upboard_gpio_led_cells,ARRAY_SIZE(upboard_gpio_led_cells), NULL, 0, NULL)){
		dev_info(fpga->dev, "Failed to add GPIO leds");
	}	
}

static const struct acpi_device_id upboard_fpga_acpi_match[] = {
	{ "AANT0000", (kernel_ulong_t)&upboard_pinctrl_data },
	{ "AANT0F00", (kernel_ulong_t)&upboard_up_fpga_data },
	{ "AANT0F01", (kernel_ulong_t)&upboard_up2_fpga_data },
	{ "AANT0F02", (kernel_ulong_t)&upboard_upcore_crex_fpga_data },
	{ "AANT0F03", (kernel_ulong_t)&upboard_upcore_crst02_fpga_data },
	{ "AANT0F04", (kernel_ulong_t)&upboard_up_fpga_data },
	{ }
};
MODULE_DEVICE_TABLE(acpi, upboard_fpga_acpi_match);

#define UPFPGA_QUIRK_UNINITIALISED  BIT(0)
#define UPFPGA_QUIRK_HRV1_IS_PROTO2 BIT(1)
#define UPFPGA_QUIRK_GPIO_LED       BIT(2)

static const struct dmi_system_id upboard_dmi_table[] __initconst = {
	{
		.matches = { /* UP */
			DMI_EXACT_MATCH(DMI_SYS_VENDOR, "AAEON"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "UP-CHT01"),
			DMI_EXACT_MATCH(DMI_BOARD_VERSION, "V0.4"),
		},
		.driver_data = (void *)UPFPGA_QUIRK_UNINITIALISED,
	},
	{
		.matches = { /* UP2 */
			DMI_EXACT_MATCH(DMI_SYS_VENDOR, "AAEON"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "UP-APL01"),
			DMI_EXACT_MATCH(DMI_BOARD_VERSION, "V0.3"),
		},
		.driver_data = (void *)(UPFPGA_QUIRK_UNINITIALISED |
			UPFPGA_QUIRK_HRV1_IS_PROTO2),
	},
	{
		.matches = { /* UP2 Pro*/
			DMI_EXACT_MATCH(DMI_SYS_VENDOR, "AAEON"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "UPN-APL01"),
			DMI_EXACT_MATCH(DMI_BOARD_VERSION, "V1.0"),
		},
		.driver_data = (void *)UPFPGA_QUIRK_HRV1_IS_PROTO2,
	},
	{
		.matches = { /* UP2 */
			DMI_EXACT_MATCH(DMI_SYS_VENDOR, "AAEON"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "UP-APL01"),
			DMI_EXACT_MATCH(DMI_BOARD_VERSION, "V0.4"),
		},
		.driver_data = (void *)UPFPGA_QUIRK_HRV1_IS_PROTO2,
	},
	{
		.matches = { /* UP APL03 */
			DMI_EXACT_MATCH(DMI_SYS_VENDOR, "AAEON"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "UP-APL03"),
			DMI_EXACT_MATCH(DMI_BOARD_VERSION, "V1.0"),
		},
		.driver_data = (void *)(UPFPGA_QUIRK_HRV1_IS_PROTO2 |
			UPFPGA_QUIRK_GPIO_LED),
	},
	{
		.matches = { /* UP Xtreme */
			DMI_EXACT_MATCH(DMI_SYS_VENDOR, "AAEON"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "UP-WHL01"),
			DMI_EXACT_MATCH(DMI_BOARD_VERSION, "V0.1"),
		},
	},
	{
		.matches = { /* UP Xtreme i11 */
			DMI_EXACT_MATCH(DMI_SYS_VENDOR, "AAEON"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "UPX-TGL01"),
		},
	},
	{
		.matches = { /* UP Xtreme i12 */
			DMI_EXACT_MATCH(DMI_SYS_VENDOR, "AAEON"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "UPX-ADLP01"),
		},		
	},
	{
		.matches = { /* UP Squared 6000*/
			DMI_EXACT_MATCH(DMI_SYS_VENDOR, "AAEON"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "UPN-EHL01"),
		},	
	},
	{
		.matches = { /* UPS 6000 */
			DMI_EXACT_MATCH(DMI_SYS_VENDOR, "AAEON"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "UPS-EHL01"),
		},		
	},
	{ },
};

static int __init upboard_fpga_probe(struct platform_device *pdev)
{
	struct upboard_fpga *fpga;
	const struct acpi_device_id *id;
	const struct upboard_fpga_data *fpga_data;
	const struct dmi_system_id *system_id;
	acpi_handle handle;
	acpi_status status;
	unsigned long long hrv;
	unsigned long quirks = 0;
	int ret;

	id = acpi_match_device(upboard_fpga_acpi_match, &pdev->dev);
	if (!id)
		return -ENODEV;

	handle = ACPI_HANDLE(&pdev->dev);
	status = acpi_evaluate_integer(handle, "_HRV", NULL, &hrv);
	if (ACPI_FAILURE(status)) {
		dev_err(&pdev->dev, "failed to get PCTL revision");
		//return -ENODEV;
	}

	system_id = dmi_first_match(upboard_dmi_table);
	if (system_id)
		quirks = (unsigned long)system_id->driver_data;

	if (hrv == 1 && (quirks & UPFPGA_QUIRK_HRV1_IS_PROTO2))
		hrv = UPFPGA_PROTOCOL_V2_HRV;

	if (hrv != UPFPGA_PROTOCOL_V2_HRV) {
		dev_info(&pdev->dev, "unsupported PCTL revision: %llu", hrv);
		//return -ENODEV;
	}

	fpga_data = (const struct upboard_fpga_data *) id->driver_data;

	fpga = devm_kzalloc(&pdev->dev, sizeof(*fpga), GFP_KERNEL);
	if (!fpga)
		return -ENOMEM;

	if (quirks & UPFPGA_QUIRK_UNINITIALISED) {
		dev_info(&pdev->dev, "FPGA not initialised by this BIOS");
		fpga->uninitialised = true;
	}

	dev_set_drvdata(&pdev->dev, fpga);
	fpga->dev = &pdev->dev;
	fpga->regmap = devm_regmap_init(&pdev->dev, NULL, fpga, fpga_data->regmapconf);
	fpga->regmapconf = fpga_data->regmapconf;
	
	if (IS_ERR(fpga->regmap))
		return PTR_ERR(fpga->regmap);

	ret = upboard_fpga_gpio_init(fpga);
	if (ret) {
		dev_err(&pdev->dev, "failed to init FPGA comm GPIOs: %d", ret);
		//return ret;	//ignore fpga error
	}
	else
	{
		ret = upboard_fpga_detect_firmware(fpga);
		//if (ret)
		//	return ret;	//ignore fpga error
	}
	
	//gpio leds init
	upboard_led_gpio_init(fpga);
	
	if (quirks & UPFPGA_QUIRK_GPIO_LED) {
#define APL_GPIO_218	507
		static struct gpio_led upboard_gpio_leds[] = {
			{
				.name = "upboard:blue:",
				.gpio = APL_GPIO_218,
				.default_state = LEDS_GPIO_DEFSTATE_KEEP,
			},
		};
		static struct gpio_led_platform_data upboard_gpio_led_platform_data = {
			.num_leds = ARRAY_SIZE(upboard_gpio_leds),
			.leds = upboard_gpio_leds,
		};
		static const struct mfd_cell upboard_gpio_led_cells[] = {
			{
				.name = "leds-gpio",
				.id = 0,
				.platform_data = &upboard_gpio_led_platform_data,
				.pdata_size = sizeof(upboard_gpio_led_platform_data),
			},
		};

		ret =  devm_mfd_add_devices(&pdev->dev, 0, upboard_gpio_led_cells,
                                    ARRAY_SIZE(upboard_gpio_led_cells), NULL, 0, NULL);
		if (ret) {
			dev_err(&pdev->dev, "Failed to add GPIO leds");
			return ret;
		}

	}
		
	return devm_mfd_add_devices(&pdev->dev, 0, fpga_data->cells,
				    fpga_data->ncells, NULL, 0, NULL);
}

static struct platform_driver upboard_fpga_driver = {
	.driver = {
		.name = "upboard-fpga",
		.acpi_match_table = upboard_fpga_acpi_match,
	},
};

module_platform_driver_probe(upboard_fpga_driver, upboard_fpga_probe);

MODULE_AUTHOR("Gary Wang <garywang@aaeon.com.tw>");
MODULE_AUTHOR("Javier Arteaga <javier@emutex.com>");
MODULE_DESCRIPTION("UP Board FPGA/EC driver");
MODULE_LICENSE("GPL v2");
