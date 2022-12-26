/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * UP Element EC driver to contorl LEDs and GPIOs 
 *
 * Copyright (c) AAEON. All rights reserved.
 *
 * Author: Gary Wang <garywang@aaeon.com.tw>
 *
 */

#include <linux/acpi.h>
#include <linux/dmi.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/ioport.h>

#include "upboard-ec.h"
#include "upboard-cpld.h"

#define UPBOARD_LED_CELL(led_data, n)                   \
	{                                               \
		.name = "upboard-led",                  \
		.id = (n),                              \
		.platform_data = &led_data[(n)],        \
		.pdata_size = sizeof(*(led_data)),      \
	}

struct upboard_ec_data {
	const struct regmap_config *regmapconf;
	const struct mfd_cell *cells;
	size_t ncells;
};

static const struct regmap_range upboard_ec_readable_ranges[] = {
	regmap_reg_range(UPEC_REG_LED, UPEC_REG_VAL2),
	regmap_reg_range(UPFPGA_REG_FUNC_EN0, UPFPGA_REG_FUNC_EN0),
	regmap_reg_range(UPEC_REG_DAT_GP16, UPEC_REG_DAT_GP15),
	regmap_reg_range(UPEC_REG_DAT2, UPEC_REG_DAT1),
};

static const struct regmap_range upboard_ec_writable_ranges[] = {
	regmap_reg_range(UPEC_REG_LED, UPEC_REG_VAL2),
	regmap_reg_range(UPFPGA_REG_FUNC_EN0, UPFPGA_REG_FUNC_EN0),
};

static const struct regmap_access_table upboard_ec_readable_table = {
	.yes_ranges = upboard_ec_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(upboard_ec_readable_ranges),
};

static const struct regmap_access_table upboard_ec_writable_table = {
	.yes_ranges = upboard_ec_writable_ranges,
	.n_yes_ranges = ARRAY_SIZE(upboard_ec_writable_ranges),
};

static const struct regmap_config upboard_ec_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = UPEC_REG_DAT1,
	.reg_read = upboard_ec_read,
	.reg_write = upboard_ec_write,
	.fast_io = false,
	.cache_type = REGCACHE_NONE,
	.rd_table = &upboard_ec_readable_table,
	.wr_table = &upboard_ec_writable_table,
};

//UP ELEMENT
static struct upboard_led_data up_element_led_data[] = {
	{ .bit = 0, .colour = "0" },
	{ .bit = 1, .colour = "1" },
	{ .bit = 2, .colour = "2" },
	{ .bit = 3, .colour = "3" },
	{ .bit = 4, .colour = "4" },
	{ .bit = 5, .colour = "5" },
	{ .bit = 6, .colour = "6" },
	{ .bit = 7, .colour = "7" },
};

static const struct mfd_cell up_element_mfd_cells[] = {
	{ .name = "upelement-pinctrl" },
	UPBOARD_LED_CELL(up_element_led_data, 0),
	UPBOARD_LED_CELL(up_element_led_data, 1),
	UPBOARD_LED_CELL(up_element_led_data, 2),
	UPBOARD_LED_CELL(up_element_led_data, 3),
	UPBOARD_LED_CELL(up_element_led_data, 4),
	UPBOARD_LED_CELL(up_element_led_data, 5),
	UPBOARD_LED_CELL(up_element_led_data, 6),
	UPBOARD_LED_CELL(up_element_led_data, 7),
};

static const struct upboard_ec_data up_element_ec_data = {
	.regmapconf = &upboard_ec_regmap_config,
	.cells = up_element_mfd_cells,
	.ncells = ARRAY_SIZE(up_element_mfd_cells),
};

// output buffer full
static inline void wait_obf (void)
{
        while (!(inb(UPEC_STATUS_PORT) & UPEC_OBF));
}
// output buffer empty
static inline void wait_obe (void)
{
        while (inb(UPEC_STATUS_PORT) & UPEC_OBF)
    inb(UPEC_DATA_PORT);
}
// input buffer empty
static inline void wait_ibe (void)
{
        while (inb(UPEC_STATUS_PORT) & UPEC_IBF);
}

static void send_cmd (unsigned char cmd)
{
        wait_obe();
        wait_ibe();
        outb(cmd, UPEC_CMD_PORT);
        wait_ibe();
}

static void send_data(unsigned char data)
{
        wait_obe();
        wait_ibe();
        outb(data, UPEC_DATA_PORT);
        wait_ibe();
}

static unsigned char read_data(void)
{
        wait_obf();
        return(inb(UPEC_DATA_PORT));
}

static void write_lpc( unsigned char index, unsigned char data )
{
    outb(index, UPEC_CFG1_PORT);
    outb(data, UPEC_CFG1_PORT+1);
}

static unsigned char read_lpc( unsigned char index )
{
    outb(index, UPEC_CFG1_PORT);
    return inb(UPEC_CFG1_PORT+1);
}

static unsigned char get_ram(int addr)
{
        unsigned char temp_data;
        unsigned char addr_high_byte, addr_low_byte;

        addr_high_byte=(addr>>8)&0xff;
        addr_low_byte=addr&0xff;

        write_lpc(UPEC_CFG0_PORT, 0x11);
        write_lpc(UPEC_CFG0_PORT+1, addr_high_byte);

        write_lpc(UPEC_CFG0_PORT, 0x10);
        write_lpc(UPEC_CFG0_PORT+1, addr_low_byte);

        write_lpc(UPEC_CFG0_PORT, 0x12);
        temp_data =  read_lpc(UPEC_CFG0_PORT+1);

	return temp_data;
}

void set_ram(int addr, unsigned char data)
{
        unsigned char addr_high_byte, addr_low_byte;

        addr_high_byte=(addr>>8)&0xff;
        addr_low_byte=addr&0xff;

        write_lpc(0x2e, 0x11);
        write_lpc(0x2f, addr_high_byte);

        write_lpc(0x2e, 0x10);
        write_lpc(0x2f, addr_low_byte);

        write_lpc(0x2e, 0x12);
        write_lpc(0x2f, data);
}


int upboard_ec_read(void *context, unsigned int reg, unsigned int *val)
{

	if(reg==UPFPGA_REG_FUNC_EN0)
	{
		send_cmd(UPEC_CMD_READ);
		send_data(UPEC_REG_LED);
		*val = read_data();
		return 0;
	}
	if(reg > UPEC_REG_MAX)
	{
	    *val = get_ram(reg);
	}
	else
	{
	    send_cmd(UPEC_CMD_READ);
	    send_data(reg);
	    *val = read_data();
	}
	//dev_info(NULL, "upboard_ec_read:reg:%X, val:%X", reg,*val);  
	return 0;
}

int upboard_ec_write(void *context, unsigned int reg, unsigned int val)
{
	send_cmd(UPEC_CMD_WRITE);
	if(reg==UPFPGA_REG_FUNC_EN0)	//fpga fun0 reg for leds
	    send_data(UPEC_REG_LED);
	else
	    send_data(reg);
	send_data(val);
	
	//dev_info(NULL, "upboard_ec_write:reg:%X, val:%X", reg,val);  
	return 0;
}

static int upboard_ec_init(void)
{
	int i;
	char name[16];

	//release before request
	release_region(UPEC_CFG0_PORT, 2);
	release_region(UPEC_CFG1_PORT, 2);
	release_region(UPEC_CMD_READ, 2);
	release_region(UPEC_DATA_PORT, 8);
	
	if(request_region(UPEC_CFG0_PORT, 2, KBUILD_MODNAME)<0)
	    return -EBUSY;
	if(request_region(UPEC_CFG1_PORT, 2, KBUILD_MODNAME)<0)
	    return -EBUSY;
	if(request_region(UPEC_CMD_READ, 2, KBUILD_MODNAME)<0)
	    return -EBUSY;
	if(request_region(UPEC_DATA_PORT, 8, KBUILD_MODNAME)<0)
	    return -EBUSY;

        for(i=0; i < 16;i++){  
	    name[i] = get_ram(0x810+i);
	}
	//add if more boards support in the feature
	if(strncmp(name,NUC_AMR,strlen(NUC_AMR))==0)
		return 0;
		
	return -ENODEV;
}

static const struct acpi_device_id upboard_ec_acpi_match[] = {
	{ "PNP0C09", (kernel_ulong_t)&up_element_ec_data },
	{ }
};
MODULE_DEVICE_TABLE(acpi, upboard_ec_acpi_match);

static int __init upboard_ec_probe(struct platform_device *pdev)
{
	struct upboard_ec *ec;
	const struct acpi_device_id *id;
	const struct dmi_system_id *system_id;
	const struct upboard_ec_data *ec_data;
	unsigned long quirks = 0;
	int ret;

	id = acpi_match_device(upboard_ec_acpi_match, &pdev->dev);
	if (!id)
		return -ENODEV;

	ret = upboard_ec_init();
	if(ret)
		return ret;

	ec_data = (const struct upboard_ec_data *) id->driver_data;

	ec = devm_kzalloc(&pdev->dev, sizeof(*ec), GFP_KERNEL);
	if (!ec)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, ec);
	ec->dev = &pdev->dev;
	ec->regmap = devm_regmap_init(&pdev->dev, NULL, ec, ec_data->regmapconf);
	if (IS_ERR(ec->regmap))
		return PTR_ERR(ec->regmap);
	
	return devm_mfd_add_devices(&pdev->dev, 0, ec_data->cells, ec_data->ncells, NULL, 0, NULL);
}

static struct platform_driver upboard_ec_driver = {
	.driver = {
		.name = "upboard-ec",
		.acpi_match_table = upboard_ec_acpi_match,
	},
};

module_platform_driver_probe(upboard_ec_driver, upboard_ec_probe);

MODULE_AUTHOR("Gary Wang <garywang@aaeon.com.tw>");
MODULE_DESCRIPTION("UP Element EC driver");
MODULE_LICENSE("GPL v2");
