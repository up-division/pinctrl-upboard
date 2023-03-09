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

struct upboard_fpga_data {
	const struct regmap_config *cpld_config;
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

#define AAEON_MANUFACTURER_ID		0x01
#define SUPPORTED_FW_MAJOR		0x0
#define MENUFACTURER_ID_MASK		GENMASK(7, 0)

#define FIRMWARE_ID_BUILD_OFFSET	(12)
#define FIRMWARE_ID_MAJOR_OFFSET	(8)
#define FIRMWARE_ID_MINOR_OFFSET	(4)
#define FIRMWARE_ID_PATCH_OFFSET	(0)
#define FIRMWARE_ID_MASK		GENMASK(3, 0)

/*
 * For UP Board Series FPGA register read/write protocols
 * EMUTEX specs:
 * D0   D1  D2  D3  D4  D5  D6  D7  D8  D9 .... D22  D23
 * [RW][        address           ][        DATA        ]
 *
 * Read Sequence:
 *      ___   ____________________________________________________   _________
 * clr:    \_/ <--low-pulse does start the write-readback         \_/<--start
 *              sequence with partital reset of internal          new sequence
 *              registers but the CONF-REG.
 *        ____________________________________________________________________
 * rst: _/       _   _   _        _   _   _   __       __   __   _
 * stb: STB#1->_/1\_/2\_/3\_...._/7\_/8\_/9\_/10\_..../23\_/24\_/<-STB#25 edge
 *                                                                   is needed
 *                                                                   to ACK
 *             (D0 - D7 stb rising latch)
 * data_in:     D0  D1  D2  .... D6  D7  don't ........ care(DC)
 * data_out:    don't ...........care(DC)  D8   D9 ....  D22  D23
 *                                   (D8 - D23 stb falling latch)
 * flag_Read:                             _________...._________
 *      __DC_   ____________...._________/                      \_
 * counter:
 *    [00]DC[00][01][02] ............[08][9][10]............[24][00]
 * CONF-REG:
 *    [00] [                     CONF-REG                       ]
 * wreg:
 *    [00]DC[00][  wreg=SHFT(wreg)  ][ADR][DATA][wreg=SHFT(wreg)]
 * wreg2:
 *                                        [    (COPY)=ADDR      ]
 */
static int upboard_fpga_read(void *context, unsigned int reg, unsigned int *val)
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
		gpiod_set_value(fpga->datain_gpio, !!(reg & BIT(i)));
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

/*
 * Write Sequence:
 *      ___   ____________________________________________________   _________
 * clr:    \_/ <--low-pulse does start the write-readback         \_/<--start
 *              sequence with partital reset of internal          new sequence
 *              registers but the CONF-REG.
 *        ____________________________________________________________________
 * rst: _/       _   _   _        _   _   _   __       __   __   _
 * stb: STB#1->_/1\_/2\_/3\_...._/7\_/8\_/9\_/10\_..../23\_/24\_/<-STB#25 edge
 *                                                                   is needed
 *                                                                   to ACK
 *             (D0 - D23 stb rising latch)
 * data_in:     D0  D1  D2  .... D6  D7  D8  D9 ....  D22  D23
 * data_out:    don't ................................care (DC)
 * flag_Read:
 *      __DC_   ____________....__________________________________
 * counter:
 *    [00]DC[00][01][02] ............[08][9][10]............[24][00]
 * wreg:
 *    [00]DC[00][wreg=SHFT(wreg)&dat_in ][SHFT(wreg)&dat_in][DAT]
 * wreg2:
 *                                   [     (COPY)=ADDR          ]
 * CONF-REG:
 *    [00] [                 CONF-REG = OLD VALUE               ][CONF-REG=DAT]
 */
static int upboard_fpga_write(void *context, unsigned int reg, unsigned int val)
{
	struct upboard_fpga * const fpga = context;
	int i;
	
	if (IS_ERR(fpga->clear_gpio))	//for none fpga boards
		return 0;

	gpiod_set_value(fpga->clear_gpio, 0);
	gpiod_set_value(fpga->clear_gpio, 1);

	for (i = UPFPGA_ADDRESS_SIZE; i >= 0; i--) {
		gpiod_set_value(fpga->strobe_gpio, 0);
		gpiod_set_value(fpga->datain_gpio, !!(reg & BIT(i)));
		gpiod_set_value(fpga->strobe_gpio, 1);
	}

	gpiod_set_value(fpga->strobe_gpio, 0);

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
	.reg_read = upboard_fpga_read,
	.reg_write = upboard_fpga_write,
	.fast_io = false,
	.cache_type = REGCACHE_NONE,
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
	.cpld_config = &upboard_up_regmap_config,
	.cells = upboard_up_mfd_cells,
	.ncells = ARRAY_SIZE(upboard_up_mfd_cells),
};

/* UP Squared 6000 EHL board */
static const struct mfd_cell upboard_pinctrl_cells[] = {
	{ .name = "upboard-pinctrl" },
};

static const struct upboard_fpga_data upboard_pinctrl_data = {
	.cpld_config = &upboard_up_regmap_config,
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
};

static const struct upboard_fpga_data upboard_up2_fpga_data = {
	.cpld_config = &upboard_up2_regmap_config,
	.cells = upboard_up2_mfd_cells,
	.ncells = ARRAY_SIZE(upboard_up2_mfd_cells),
};

/* UP-CREX carrier board for UP Core */

/* same MAXV config as UP1 (proto2 release) */
#define upboard_upcore_crex_fpga_data upboard_up_fpga_data

/* UP-CRST02 carrier board for UP Core */

/* same MAX10 config as UP2, but same LED cells as UP1 */
static const struct upboard_fpga_data upboard_upcore_crst02_fpga_data = {
	.cpld_config = &upboard_up2_regmap_config,
	.cells = upboard_up_mfd_cells,
	.ncells = ARRAY_SIZE(upboard_up_mfd_cells),
};

static int __init upboard_fpga_gpio_init(struct upboard_fpga *fpga)
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

/*
 * MFD upboard-fpga is ACPI driver and can recognize the AANT ID from different
 * kind of upboards. We get the LED GPIO initialized information from this
 * then add led-upboard driver.
 */
int upboard_led_gpio_register(struct upboard_fpga *fpga)
{
	struct gpio_led blue_led,yellow_led,green_led,red_led;
	struct gpio_desc *desc;
	struct gpio_led upboard_gpio_leds[4];
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
		return 0;		
	
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
		dev_err(fpga->dev, "Failed to add GPIO leds");
	}	
	
	return 0;
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

static int __init upboard_fpga_probe(struct platform_device *pdev)
{
	struct upboard_fpga *fpga;
	const struct acpi_device_id *id;
	const struct upboard_fpga_data *fpga_data;
	int ret;
	struct device *dev = &pdev->dev;

	id = acpi_match_device(upboard_fpga_acpi_match, &pdev->dev);
	if (!id)
		return -ENODEV;

	fpga_data = (const struct upboard_fpga_data *) id->driver_data;

	fpga = devm_kzalloc(&pdev->dev, sizeof(*fpga), GFP_KERNEL);
	if (!fpga)
		return -ENOMEM;

	//force init once
	//fpga->uninitialised = true;

	platform_set_drvdata(pdev, fpga);
	fpga->dev = &pdev->dev;
	fpga->regmap = devm_regmap_init(&pdev->dev, NULL, fpga, fpga_data->cpld_config);
	fpga->cpld_config = fpga_data->cpld_config;
	
	if (IS_ERR(fpga->regmap))
		return PTR_ERR(fpga->regmap);

	ret = upboard_fpga_gpio_init(fpga);
	if (ret) {
		/*
		 * This is for compatiable with some upboards w/o FPGA firmware,
		 * so just showing debug info and do not return directly.
		 */
		dev_warn(dev, "Failed to initialize FPGA common GPIOs: %d", ret);
	} else {
		upboard_fpga_show_firmware_info(fpga);
	}
	
	/* register GPIO LEDs */
	ret = upboard_led_gpio_register(fpga);
	if (ret) {
		/*
		 * This is for compatiable with some upboards w/o LEDs.
		 */
		dev_warn(dev, "Failed to register LEDs: %d", ret);
	}

	return devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO,
				    fpga_data->cells,
				    fpga_data->ncells,
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
