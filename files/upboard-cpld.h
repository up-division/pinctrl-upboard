/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * UP Board CPLD/FPGA driver
 *
 * Copyright (c) AAEON. All rights reserved.
 *
 * Author: Gary Wang <garywang@aaeon.com.tw>
 *
 */

#ifndef __LINUX_MFD_UPBOARD_FPGA_H
#define __LINUX_MFD_UPBOARD_FPGA_H

//CPLD/FPGA protocol version
#define UPFPGA_PROTOCOL_V1_HRV 1
#define UPFPGA_PROTOCOL_V2_HRV 2

#define UPFPGA_ADDRESS_SIZE  7
#define UPFPGA_REGISTER_SIZE 16

#define UPFPGA_READ_FLAG     (1 << UPFPGA_ADDRESS_SIZE)

enum upcpld_ids {
	AANT0000_ID = 255,
	AANT0F00_ID = 0,
	AANT0F01_ID = 1,
	AANT0F02_ID = 2,
	AANT0F03_ID = 3,
	AANT0F04_ID = 4,
};

enum upboard_fpgareg {
	UPFPGA_REG_PLATFORM_ID   = 0x10,
	UPFPGA_REG_FIRMWARE_ID   = 0x11,
	UPFPGA_REG_FUNC_EN0      = 0x20,
	UPFPGA_REG_FUNC_EN1      = 0x21,
	UPFPGA_REG_GPIO_EN0      = 0x30,
	UPFPGA_REG_GPIO_EN1      = 0x31,
	UPFPGA_REG_GPIO_EN2      = 0x32,
	UPFPGA_REG_GPIO_DIR0     = 0x40,
	UPFPGA_REG_GPIO_DIR1     = 0x41,
	UPFPGA_REG_GPIO_DIR2     = 0x42,
	UPFPGA_REG_MAX,
};

struct upboard_fpga {
	struct device *dev;
	struct regmap *regmap;
	//struct regmap_config *cpld_config;
	struct gpio_desc *enable_gpio;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *clear_gpio;
	struct gpio_desc *strobe_gpio;
	struct gpio_desc *datain_gpio;
	struct gpio_desc *dataout_gpio;
	bool uninitialised;
};

struct upboard_led_data {
	unsigned int bit;
	const char *colour;
};

static int upboard_cpld_read(void *, unsigned int, unsigned int *);
static int upboard_cpld_write(void *, unsigned int, unsigned int);
void upboard_pwm_register(void);

#endif /*  __LINUX_MFD_UPBOARD_FPGA_H */
