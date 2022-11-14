/*
 * UP Board FPGA-based pin controller driver
 *
 * Copyright (c) 2017, Emutex Ltd. All rights reserved.
 *
 * Authors: Javier Arteaga <javier@emutex.com>
 *          Dan O'Donovan <dan@emutex.com>
 *          Nicola Lunghi <nicolal@emutex.com>
 	    Gary Wang 	<garywang@aaeon.com.tw>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
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

#include "core.h"
#include "upboard-fpga.h"

/* Offset from regs */
#define REVID				0x000
#define REVID_SHIFT			16
#define REVID_MASK			GENMASK(31, 16)

#define CAPLIST				0x004
#define CAPLIST_ID_SHIFT		16
#define CAPLIST_ID_MASK			GENMASK(23, 16)
#define CAPLIST_ID_GPIO_HW_INFO		1
#define CAPLIST_ID_PWM			2
#define CAPLIST_ID_BLINK		3
#define CAPLIST_ID_EXP			4
#define CAPLIST_NEXT_SHIFT		0
#define CAPLIST_NEXT_MASK		GENMASK(15, 0)

#define PADBAR				0x00c

#define PADOWN_BITS			4
#define PADOWN_SHIFT(p)			((p) % 8 * PADOWN_BITS)
#define PADOWN_MASK(p)			(GENMASK(3, 0) << PADOWN_SHIFT(p))
#define PADOWN_GPP(p)			((p) / 8)

/* Offset from pad_regs */
#define PADCFG0				0x000
#define PADCFG0_RXEVCFG_SHIFT		25
#define PADCFG0_RXEVCFG_MASK		GENMASK(26, 25)
#define PADCFG0_RXEVCFG_LEVEL		0
#define PADCFG0_RXEVCFG_EDGE		1
#define PADCFG0_RXEVCFG_DISABLED	2
#define PADCFG0_RXEVCFG_EDGE_BOTH	3
#define PADCFG0_PREGFRXSEL		BIT(24)
#define PADCFG0_RXINV			BIT(23)
#define PADCFG0_GPIROUTIOXAPIC		BIT(20)
#define PADCFG0_GPIROUTSCI		BIT(19)
#define PADCFG0_GPIROUTSMI		BIT(18)
#define PADCFG0_GPIROUTNMI		BIT(17)
#define PADCFG0_PMODE_SHIFT		10
#define PADCFG0_PMODE_MASK		GENMASK(13, 10)
#define PADCFG0_PMODE_GPIO		0
#define PADCFG0_GPIORXDIS		BIT(9)
#define PADCFG0_GPIOTXDIS		BIT(8)
#define PADCFG0_GPIORXSTATE		BIT(1)
#define PADCFG0_GPIOTXSTATE		BIT(0)

#define PADCFG1				0x004
#define PADCFG1_TERM_UP			BIT(13)
#define PADCFG1_TERM_SHIFT		10
#define PADCFG1_TERM_MASK		GENMASK(12, 10)
#define PADCFG1_TERM_20K		BIT(2)
#define PADCFG1_TERM_5K			BIT(1)
#define PADCFG1_TERM_1K			BIT(0)
#define PADCFG1_TERM_833		(BIT(1) | BIT(0))

#define PADCFG2				0x008
#define PADCFG2_DEBEN			BIT(0)
#define PADCFG2_DEBOUNCE_SHIFT		1
#define PADCFG2_DEBOUNCE_MASK		GENMASK(4, 1)

#define DEBOUNCE_PERIOD_NSEC		31250

/* Additional features supported by the hardware */
#define PINCTRL_FEATURE_DEBOUNCE	BIT(0)
#define PINCTRL_FEATURE_1K_PD		BIT(1)

#define pin_to_padno(c, p)	((p) - (c)->pin_base)
#define padgroup_offset(g, p)	((p) - (g)->base)

struct intel_pinctrl {
	struct device *dev;
	raw_spinlock_t lock;
	struct pinctrl_desc pctldesc;
	struct pinctrl_dev *pctldev;
	struct gpio_chip chip;
	struct irq_chip irqchip;
	void *soc;
	void *communities;
	size_t ncommunities;
};

struct upboard_pin {
	struct regmap_field *funcbit;
	struct regmap_field *enbit;
	struct regmap_field *dirbit;
	unsigned int gpio;	//native pin controller number
	unsigned int base;	//native pin controller base
	int irq;		//native pin controller irq
	void __iomem *regs; 
};

struct upboard_bios {
	const struct reg_sequence *patches;
	size_t npatches;
};

struct upboard_pinctrl {
	struct gpio_chip chip;
	struct device *dev;
	struct pinctrl_dev *pctldev;
	struct pinctrl_desc *pctldesc;
	struct upboard_pin *pins;
	struct regmap *regmap;
	const unsigned int *rpi_mapping;
	int ident;	
};

enum upboard_func0_fpgabit {
	UPFPGA_I2C0_EN = 8,
	UPFPGA_I2C1_EN = 9,
	UPFPGA_CEC0_EN = 12,
	UPFPGA_ADC0_EN = 14,
};

static const struct reg_field upboard_i2c0_reg =
	REG_FIELD(UPFPGA_REG_FUNC_EN0, UPFPGA_I2C0_EN, UPFPGA_I2C0_EN);

static const struct reg_field upboard_i2c1_reg =
	REG_FIELD(UPFPGA_REG_FUNC_EN0, UPFPGA_I2C1_EN, UPFPGA_I2C1_EN);

static const struct reg_field upboard_adc0_reg =
	REG_FIELD(UPFPGA_REG_FUNC_EN0, UPFPGA_ADC0_EN, UPFPGA_ADC0_EN);


/* Pin group information */
struct upboard_pingroup {
	const char *name;
	const unsigned *pins;
	size_t npins;
};

/* Pin function information */
struct upboard_function {
	const char *name;
	const char * const *groups;
	size_t ngroups;
};

#define UPBOARD_BIT_TO_PIN(r, bit) \
	((r) * UPFPGA_REGISTER_SIZE + (bit))

/*
 * UP board data
 */

#define UPBOARD_UP_BIT_TO_PIN(r, id) (UPBOARD_BIT_TO_PIN(r, UPFPGA_UP_##id))

#define UPBOARD_UP_PIN_ANON(r, bit)					\
	{								\
		.number = UPBOARD_BIT_TO_PIN(r, bit),			\
	}

#define UPBOARD_UP_PIN_NAME(r, id)					\
	{								\
		.number = UPBOARD_UP_BIT_TO_PIN(r, id),			\
		.name = #id,						\
	}

#define UPBOARD_UP_PIN_FUNC(r, id, data)				\
	{								\
		.number = UPBOARD_UP_BIT_TO_PIN(r, id),			\
		.name = #id,						\
		.drv_data = (void *)(data),				\
	}
	
#define PIN_GROUP(n, p)				\
	{					\
		.name = (n),			\
		.pins = (p),			\
		.npins = ARRAY_SIZE((p)),	\
	}

#define FUNCTION(n, g)				\
	{					\
		.name = (n),			\
		.groups = (g),			\
		.ngroups = ARRAY_SIZE((g)),	\
	}
	

enum upboard_up_reg1_fpgabit {
	UPFPGA_UP_I2C1_SDA,
	UPFPGA_UP_I2C1_SCL,
	UPFPGA_UP_ADC0,
	UPFPGA_UP_GPIO17,
	UPFPGA_UP_GPIO27,
	UPFPGA_UP_GPIO22,
	UPFPGA_UP_SPI_MOSI,
	UPFPGA_UP_SPI_MISO,
	UPFPGA_UP_SPI_CLK,
	UPFPGA_UP_I2C0_SDA,
	UPFPGA_UP_GPIO5,
	UPFPGA_UP_GPIO6,
	UPFPGA_UP_PWM1,
	UPFPGA_UP_I2S_FRM,
	UPFPGA_UP_GPIO26,
	UPFPGA_UP_UART1_TX,
};

enum upboard_up_reg2_fpgabit {
	UPFPGA_UP_UART1_RX,
	UPFPGA_UP_I2S_CLK,
	UPFPGA_UP_GPIO23,
	UPFPGA_UP_GPIO24,
	UPFPGA_UP_GPIO25,
	UPFPGA_UP_SPI_CS0,
	UPFPGA_UP_SPI_CS1,
	UPFPGA_UP_I2C0_SCL,
	UPFPGA_UP_PWM0,
	UPFPGA_UP_GPIO16,
	UPFPGA_UP_I2S_DIN,
	UPFPGA_UP_I2S_DOUT,
};

#define UPFPGA_UP_UART1_RTS UPFPGA_UP_GPIO17
#define UPFPGA_UP_UART1_CTS UPFPGA_UP_GPIO16

static struct pinctrl_pin_desc upboard_up_pins[] = {
	UPBOARD_UP_PIN_FUNC(0, I2C1_SDA, &upboard_i2c1_reg),
	UPBOARD_UP_PIN_FUNC(0, I2C1_SCL, &upboard_i2c1_reg),
	UPBOARD_UP_PIN_FUNC(0, ADC0, &upboard_adc0_reg),
	UPBOARD_UP_PIN_NAME(0, UART1_RTS),
	UPBOARD_UP_PIN_NAME(0, GPIO27),
	UPBOARD_UP_PIN_NAME(0, GPIO22),
	UPBOARD_UP_PIN_NAME(0, SPI_MOSI),
	UPBOARD_UP_PIN_NAME(0, SPI_MISO),
	UPBOARD_UP_PIN_NAME(0, SPI_CLK),
	UPBOARD_UP_PIN_FUNC(0, I2C0_SDA, &upboard_i2c0_reg),
	UPBOARD_UP_PIN_NAME(0, GPIO5),
	UPBOARD_UP_PIN_NAME(0, GPIO6),
	UPBOARD_UP_PIN_NAME(0, PWM1),
	UPBOARD_UP_PIN_NAME(0, I2S_FRM),
	UPBOARD_UP_PIN_NAME(0, GPIO26),
	UPBOARD_UP_PIN_NAME(0, UART1_TX),
	/* register 1 */
	UPBOARD_UP_PIN_NAME(1, UART1_RX),
	UPBOARD_UP_PIN_NAME(1, I2S_CLK),
	UPBOARD_UP_PIN_NAME(1, GPIO23),
	UPBOARD_UP_PIN_NAME(1, GPIO24),
	UPBOARD_UP_PIN_NAME(1, GPIO25),
	UPBOARD_UP_PIN_NAME(1, SPI_CS0),
	UPBOARD_UP_PIN_NAME(1, SPI_CS1),
	UPBOARD_UP_PIN_FUNC(1, I2C0_SCL, &upboard_i2c0_reg),
	UPBOARD_UP_PIN_NAME(1, PWM0),
	UPBOARD_UP_PIN_NAME(1, UART1_CTS),
	UPBOARD_UP_PIN_NAME(1, I2S_DIN),
	UPBOARD_UP_PIN_NAME(1, I2S_DOUT),
};

static const unsigned int upboard_up_rpi_mapping[] = {
	UPBOARD_UP_BIT_TO_PIN(0, I2C0_SDA),
	UPBOARD_UP_BIT_TO_PIN(1, I2C0_SCL),
	UPBOARD_UP_BIT_TO_PIN(0, I2C1_SDA),
	UPBOARD_UP_BIT_TO_PIN(0, I2C1_SCL),
	UPBOARD_UP_BIT_TO_PIN(0, ADC0),
	UPBOARD_UP_BIT_TO_PIN(0, GPIO5),
	UPBOARD_UP_BIT_TO_PIN(0, GPIO6),
	UPBOARD_UP_BIT_TO_PIN(1, SPI_CS1),
	UPBOARD_UP_BIT_TO_PIN(1, SPI_CS0),
	UPBOARD_UP_BIT_TO_PIN(0, SPI_MISO),
	UPBOARD_UP_BIT_TO_PIN(0, SPI_MOSI),
	UPBOARD_UP_BIT_TO_PIN(0, SPI_CLK),
	UPBOARD_UP_BIT_TO_PIN(1, PWM0),
	UPBOARD_UP_BIT_TO_PIN(0, PWM1),
	UPBOARD_UP_BIT_TO_PIN(0, UART1_TX),
	UPBOARD_UP_BIT_TO_PIN(1, UART1_RX),
	UPBOARD_UP_BIT_TO_PIN(1, GPIO16),
	UPBOARD_UP_BIT_TO_PIN(0, GPIO17),
	UPBOARD_UP_BIT_TO_PIN(1, I2S_CLK),
	UPBOARD_UP_BIT_TO_PIN(0, I2S_FRM),
	UPBOARD_UP_BIT_TO_PIN(1, I2S_DIN),
	UPBOARD_UP_BIT_TO_PIN(1, I2S_DOUT),
	UPBOARD_UP_BIT_TO_PIN(0, GPIO22),
	UPBOARD_UP_BIT_TO_PIN(1, GPIO23),
	UPBOARD_UP_BIT_TO_PIN(1, GPIO24),
	UPBOARD_UP_BIT_TO_PIN(1, GPIO25),
	UPBOARD_UP_BIT_TO_PIN(0, GPIO26),
	UPBOARD_UP_BIT_TO_PIN(0, GPIO27),
};

static const unsigned uart1_pins[] = { 14, 15, 16, 17 };
static const unsigned uart2_pins[] = { 25, 27 };
static const unsigned i2c0_pins[]  = { 0, 1 };
static const unsigned i2c1_pins[]  = { 2, 3 };
static const unsigned spi2_pins[]  = { 8, 9, 10, 11 };
static const unsigned i2s0_pins[]  = { 18, 19, 20, 21 };
static const unsigned pwm0_pins[]  = { 12 };
static const unsigned pwm1_pins[]  = { 13 };
static const unsigned adc0_pins[]  = { 4 };

static const struct upboard_pingroup pin_groups[] = {
	PIN_GROUP("uart1_grp", uart1_pins),
	PIN_GROUP("uart2_grp", uart2_pins),
	PIN_GROUP("i2c0_grp", i2c0_pins),
	PIN_GROUP("i2c1_grp", i2c1_pins),
	PIN_GROUP("spi2_grp", spi2_pins),
	PIN_GROUP("i2s0_grp", i2s0_pins),
	PIN_GROUP("pwm0_grp", pwm0_pins),
	PIN_GROUP("pwm1_grp", pwm1_pins),
	PIN_GROUP("adc0_grp", adc0_pins),
};

static const char * const uart1_groups[] = { "uart1_grp" };
static const char * const uart2_groups[] = { "uart2_grp" };
static const char * const i2c0_groups[]  = { "i2c0_grp" };
static const char * const i2c1_groups[]  = { "i2c1_grp" };
static const char * const spi2_groups[]  = { "spi2_grp" };
static const char * const i2s0_groups[]  = { "i2s0_grp" };
static const char * const pwm0_groups[]  = { "pwm0_grp" };
static const char * const pwm1_groups[]  = { "pwm1_grp" };
static const char * const adc0_groups[]  = { "adc0_grp" };

static const struct upboard_function pin_functions[] = {
	FUNCTION("uart1", uart1_groups),
	FUNCTION("uart2", uart2_groups),
	FUNCTION("i2c0",  i2c0_groups),
	FUNCTION("i2c1",  i2c1_groups),
	FUNCTION("spi2",  spi2_groups),
	FUNCTION("i2s0",  i2s0_groups),
	FUNCTION("pwm0",  pwm0_groups),
	FUNCTION("pwm1",  pwm1_groups),
	FUNCTION("adc0",  adc0_groups),
};


/*
 * Init patches applied to the registers until the BIOS sets proper defaults
 */
static const struct reg_sequence upboard_up_reg_patches[] __initconst = {
	{ UPFPGA_REG_FUNC_EN0,
		// enable I2C voltage-level shifters
		BIT(UPFPGA_I2C0_EN) |
		BIT(UPFPGA_I2C1_EN) |
		// enable adc
		BIT(UPFPGA_ADC0_EN)
	},
	/* HAT function pins initially set as inputs */
	{ UPFPGA_REG_GPIO_DIR0,
		BIT(UPFPGA_UP_I2C1_SDA)	    |
		BIT(UPFPGA_UP_I2C1_SCL)	    |
		BIT(UPFPGA_UP_ADC0)	    |
		BIT(UPFPGA_UP_GPIO27)	    |
		BIT(UPFPGA_UP_GPIO22)	    |
		BIT(UPFPGA_UP_SPI_MISO)	    |
		BIT(UPFPGA_UP_I2C0_SDA)	    |
		BIT(UPFPGA_UP_GPIO5)	    |
		BIT(UPFPGA_UP_GPIO6)	    |
		BIT(UPFPGA_UP_GPIO26)
	},
	{ UPFPGA_REG_GPIO_DIR1,
		BIT(UPFPGA_UP_UART1_RX)	|
		BIT(UPFPGA_UP_GPIO23)	|
		BIT(UPFPGA_UP_GPIO24)	|
		BIT(UPFPGA_UP_GPIO25)	|
		BIT(UPFPGA_UP_I2C0_SCL)	|
		BIT(UPFPGA_UP_GPIO16)	|
		BIT(UPFPGA_UP_I2S_DIN)
	},
};

static const struct upboard_bios upboard_up_bios_info_dvt __initconst = {
	.patches = upboard_up_reg_patches,
	.npatches = ARRAY_SIZE(upboard_up_reg_patches),
};

/*
 * UP^2 board data
 */

#define UPBOARD_UP2_BIT_TO_PIN(r, id) (UPBOARD_BIT_TO_PIN(r, UPFPGA_UP2_##id))

#define UPBOARD_UP2_PIN_ANON(r, bit)					\
	{								\
		.number = UPBOARD_BIT_TO_PIN(r, bit),			\
		.name = "NONAME",					\
	}

#define UPBOARD_UP2_PIN_NAME(r, id)					\
	{								\
		.number = UPBOARD_UP2_BIT_TO_PIN(r, id),		\
		.name = #id,						\
	}

#define UPBOARD_UP2_PIN_FUNC(r, id, data)				\
	{								\
		.number = UPBOARD_UP2_BIT_TO_PIN(r, id),		\
		.name = #id,						\
		.drv_data = (void *)(data),				\
	}

enum upboard_up2_reg0_fpgabit {
	UPFPGA_UP2_UART1_TXD,
	UPFPGA_UP2_UART1_RXD,
	UPFPGA_UP2_UART1_RTS,
	UPFPGA_UP2_UART1_CTS,
	UPFPGA_UP2_GPIO3,
	UPFPGA_UP2_GPIO5,
	UPFPGA_UP2_GPIO6,
	UPFPGA_UP2_GPIO11,
	UPFPGA_UP2_EXHAT_LVDS1n,
	UPFPGA_UP2_EXHAT_LVDS1p,
	UPFPGA_UP2_SPI2_TXD,
	UPFPGA_UP2_SPI2_RXD,
	UPFPGA_UP2_SPI2_FS1,
	UPFPGA_UP2_SPI2_FS0,
	UPFPGA_UP2_SPI2_CLK,
	UPFPGA_UP2_SPI1_TXD,
};

enum upboard_up2_reg1_fpgabit {
	UPFPGA_UP2_SPI1_RXD,
	UPFPGA_UP2_SPI1_FS1,
	UPFPGA_UP2_SPI1_FS0,
	UPFPGA_UP2_SPI1_CLK,
	UPFPGA_UP2_BIT20,
	UPFPGA_UP2_BIT21,
	UPFPGA_UP2_BIT22,
	UPFPGA_UP2_BIT23,
	UPFPGA_UP2_PWM1,
	UPFPGA_UP2_PWM0,
	UPFPGA_UP2_EXHAT_LVDS0n,
	UPFPGA_UP2_EXHAT_LVDS0p,
	UPFPGA_UP2_I2C0_SCL,
	UPFPGA_UP2_I2C0_SDA,
	UPFPGA_UP2_I2C1_SCL,
	UPFPGA_UP2_I2C1_SDA,
};

enum upboard_up2_reg2_fpgabit {
	UPFPGA_UP2_EXHAT_LVDS3n,
	UPFPGA_UP2_EXHAT_LVDS3p,
	UPFPGA_UP2_EXHAT_LVDS4n,
	UPFPGA_UP2_EXHAT_LVDS4p,
	UPFPGA_UP2_EXHAT_LVDS5n,
	UPFPGA_UP2_EXHAT_LVDS5p,
	UPFPGA_UP2_I2S_SDO,
	UPFPGA_UP2_I2S_SDI,
	UPFPGA_UP2_I2S_WS_SYNC,
	UPFPGA_UP2_I2S_BCLK,
	UPFPGA_UP2_EXHAT_LVDS6n,
	UPFPGA_UP2_EXHAT_LVDS6p,
	UPFPGA_UP2_EXHAT_LVDS7n,
	UPFPGA_UP2_EXHAT_LVDS7p,
	UPFPGA_UP2_EXHAT_LVDS2n,
	UPFPGA_UP2_EXHAT_LVDS2p,
};

static struct pinctrl_pin_desc upboard_up2_pins[] = {
	UPBOARD_UP2_PIN_NAME(0, UART1_TXD),
	UPBOARD_UP2_PIN_NAME(0, UART1_RXD),
	UPBOARD_UP2_PIN_NAME(0, UART1_RTS),
	UPBOARD_UP2_PIN_NAME(0, UART1_CTS),
	UPBOARD_UP2_PIN_NAME(0, GPIO3),
	UPBOARD_UP2_PIN_NAME(0, GPIO5),
	UPBOARD_UP2_PIN_NAME(0, GPIO6),
	UPBOARD_UP2_PIN_NAME(0, GPIO11),
	UPBOARD_UP2_PIN_NAME(0, EXHAT_LVDS1n),
	UPBOARD_UP2_PIN_NAME(0, EXHAT_LVDS1p),
	UPBOARD_UP2_PIN_NAME(0, SPI2_TXD),
	UPBOARD_UP2_PIN_NAME(0, SPI2_RXD),
	UPBOARD_UP2_PIN_NAME(0, SPI2_FS1),
	UPBOARD_UP2_PIN_NAME(0, SPI2_FS0),
	UPBOARD_UP2_PIN_NAME(0, SPI2_CLK),
	UPBOARD_UP2_PIN_NAME(0, SPI1_TXD),
	UPBOARD_UP2_PIN_NAME(1, SPI1_RXD),
	UPBOARD_UP2_PIN_NAME(1, SPI1_FS1),
	UPBOARD_UP2_PIN_NAME(1, SPI1_FS0),
	UPBOARD_UP2_PIN_NAME(1, SPI1_CLK),
	UPBOARD_UP2_PIN_ANON(1, 4),
	UPBOARD_UP2_PIN_ANON(1, 5),
	UPBOARD_UP2_PIN_ANON(1, 6),
	UPBOARD_UP2_PIN_ANON(1, 7),
	UPBOARD_UP2_PIN_NAME(1, PWM1),
	UPBOARD_UP2_PIN_NAME(1, PWM0),
	UPBOARD_UP2_PIN_NAME(1, EXHAT_LVDS0n),
	UPBOARD_UP2_PIN_NAME(1, EXHAT_LVDS0p),
	UPBOARD_UP2_PIN_FUNC(1, I2C0_SCL, &upboard_i2c0_reg),
	UPBOARD_UP2_PIN_FUNC(1, I2C0_SDA, &upboard_i2c0_reg),
	UPBOARD_UP2_PIN_FUNC(1, I2C1_SCL, &upboard_i2c1_reg),
	UPBOARD_UP2_PIN_FUNC(1, I2C1_SDA, &upboard_i2c1_reg),
	UPBOARD_UP2_PIN_NAME(2, EXHAT_LVDS3n),
	UPBOARD_UP2_PIN_NAME(2, EXHAT_LVDS3p),
	UPBOARD_UP2_PIN_NAME(2, EXHAT_LVDS4n),
	UPBOARD_UP2_PIN_NAME(2, EXHAT_LVDS4p),
	UPBOARD_UP2_PIN_NAME(2, EXHAT_LVDS5n),
	UPBOARD_UP2_PIN_NAME(2, EXHAT_LVDS5p),
	UPBOARD_UP2_PIN_NAME(2, I2S_SDO),
	UPBOARD_UP2_PIN_NAME(2, I2S_SDI),
	UPBOARD_UP2_PIN_NAME(2, I2S_WS_SYNC),
	UPBOARD_UP2_PIN_NAME(2, I2S_BCLK),
	UPBOARD_UP2_PIN_NAME(2, EXHAT_LVDS6n),
	UPBOARD_UP2_PIN_NAME(2, EXHAT_LVDS6p),
	UPBOARD_UP2_PIN_NAME(2, EXHAT_LVDS7n),
	UPBOARD_UP2_PIN_NAME(2, EXHAT_LVDS7p),
	UPBOARD_UP2_PIN_NAME(2, EXHAT_LVDS2n),
	UPBOARD_UP2_PIN_NAME(2, EXHAT_LVDS2p),
};

static const unsigned int upboard_up2_rpi_mapping[] = {
	UPBOARD_UP2_BIT_TO_PIN(1, I2C0_SDA),
	UPBOARD_UP2_BIT_TO_PIN(1, I2C0_SCL),
	UPBOARD_UP2_BIT_TO_PIN(1, I2C1_SDA),
	UPBOARD_UP2_BIT_TO_PIN(1, I2C1_SCL),
	UPBOARD_UP2_BIT_TO_PIN(0, GPIO3),
	UPBOARD_UP2_BIT_TO_PIN(0, GPIO11),
	UPBOARD_UP2_BIT_TO_PIN(0, SPI2_CLK),
	UPBOARD_UP2_BIT_TO_PIN(1, SPI1_FS1),
	UPBOARD_UP2_BIT_TO_PIN(1, SPI1_FS0),
	UPBOARD_UP2_BIT_TO_PIN(1, SPI1_RXD),
	UPBOARD_UP2_BIT_TO_PIN(0, SPI1_TXD),
	UPBOARD_UP2_BIT_TO_PIN(1, SPI1_CLK),
	UPBOARD_UP2_BIT_TO_PIN(1, PWM0),
	UPBOARD_UP2_BIT_TO_PIN(1, PWM1),
	UPBOARD_UP2_BIT_TO_PIN(0, UART1_TXD),
	UPBOARD_UP2_BIT_TO_PIN(0, UART1_RXD),
	UPBOARD_UP2_BIT_TO_PIN(0, UART1_CTS),
	UPBOARD_UP2_BIT_TO_PIN(0, UART1_RTS),
	UPBOARD_UP2_BIT_TO_PIN(2, I2S_BCLK),
	UPBOARD_UP2_BIT_TO_PIN(2, I2S_WS_SYNC),
	UPBOARD_UP2_BIT_TO_PIN(2, I2S_SDI),
	UPBOARD_UP2_BIT_TO_PIN(2, I2S_SDO),
	UPBOARD_UP2_BIT_TO_PIN(0, GPIO6),
	UPBOARD_UP2_BIT_TO_PIN(0, SPI2_FS1),
	UPBOARD_UP2_BIT_TO_PIN(0, SPI2_RXD),
	UPBOARD_UP2_BIT_TO_PIN(0, SPI2_TXD),
	UPBOARD_UP2_BIT_TO_PIN(0, SPI2_FS0),
	UPBOARD_UP2_BIT_TO_PIN(0, GPIO5),
};

/*
 * Init patches applied to the registers until the BIOS sets proper defaults
 */
static const struct reg_sequence upboard_up2_reg_patches[] __initconst = {
	// enable I2C voltage-level shifters
	{ UPFPGA_REG_FUNC_EN0,
		BIT(UPFPGA_I2C0_EN) |
		BIT(UPFPGA_I2C1_EN)
	},
	// HAT function pins initially set as inputs
	{ UPFPGA_REG_GPIO_DIR0,
		BIT(UPFPGA_UP2_UART1_RXD) |
		BIT(UPFPGA_UP2_UART1_CTS)
	},
	{ UPFPGA_REG_GPIO_DIR1,
		BIT(UPFPGA_UP2_SPI1_RXD)
	},
	// HAT function pins initially enabled (i.e. not hi-Z)
	{ UPFPGA_REG_GPIO_EN0,
		BIT(UPFPGA_UP2_UART1_TXD) |
		BIT(UPFPGA_UP2_UART1_RXD) |
		BIT(UPFPGA_UP2_UART1_RTS) |
		BIT(UPFPGA_UP2_UART1_CTS) |
		BIT(UPFPGA_UP2_SPI1_TXD)
	},
	{ UPFPGA_REG_GPIO_EN1,
		BIT(UPFPGA_UP2_SPI1_RXD) |
		BIT(UPFPGA_UP2_SPI1_FS1) |
		BIT(UPFPGA_UP2_SPI1_FS0) |
		BIT(UPFPGA_UP2_SPI1_CLK) |
		BIT(UPFPGA_UP2_PWM1) |
		BIT(UPFPGA_UP2_PWM0)
	},
};

static const struct upboard_bios upboard_up2_bios_info_v0_3 __initconst = {
	.patches = upboard_up2_reg_patches,
	.npatches = ARRAY_SIZE(upboard_up2_reg_patches),
};

/*
 * UP Core board + CREX carrier board data
 */

#define UPBOARD_UPCORE_CREX_BIT_TO_PIN(r, id)				\
	(UPBOARD_BIT_TO_PIN(r, UPFPGA_UPCORE_CREX_##id))

#define UPBOARD_UPCORE_CREX_PIN_ANON(r, bit)				\
	{								\
		.number = UPBOARD_BIT_TO_PIN(r, bit),			\
	}

#define UPBOARD_UPCORE_CREX_PIN_NAME(r, id)				\
	{								\
		.number = UPBOARD_UPCORE_CREX_BIT_TO_PIN(r, id),	\
		.name = #id,						\
	}

#define UPBOARD_UPCORE_CREX_PIN_FUNC(r, id, data)			\
	{								\
		.number = UPBOARD_UPCORE_CREX_BIT_TO_PIN(r, id),	\
		.name = #id,						\
		.drv_data = (void *)(data),				\
	}

enum upboard_upcore_crex_reg1_fpgabit {
	UPFPGA_UPCORE_CREX_I2C0_SDA,
	UPFPGA_UPCORE_CREX_I2C0_SCL,
	UPFPGA_UPCORE_CREX_I2C1_SDA,
	UPFPGA_UPCORE_CREX_I2C1_SCL,
	UPFPGA_UPCORE_CREX_SPI2_CS0,
	UPFPGA_UPCORE_CREX_SPI2_CS1,
	UPFPGA_UPCORE_CREX_SPI2_MOSI,
	UPFPGA_UPCORE_CREX_SPI2_MISO,
	UPFPGA_UPCORE_CREX_SPI2_CLK,
	UPFPGA_UPCORE_CREX_UART1_TXD,
	UPFPGA_UPCORE_CREX_UART1_RXD,
	UPFPGA_UPCORE_CREX_PWM0,
	UPFPGA_UPCORE_CREX_PWM1,
	UPFPGA_UPCORE_CREX_I2S2_FRM,
	UPFPGA_UPCORE_CREX_I2S2_CLK,
	UPFPGA_UPCORE_CREX_I2S2_RX,
};

enum upboard_upcore_crex_reg2_fpgabit {
	UPFPGA_UPCORE_CREX_I2S2_TX,
	UPFPGA_UPCORE_CREX_GPIO0,
	UPFPGA_UPCORE_CREX_GPIO2,
	UPFPGA_UPCORE_CREX_GPIO3,
	UPFPGA_UPCORE_CREX_GPIO4,
	UPFPGA_UPCORE_CREX_GPIO9,
};

static struct pinctrl_pin_desc upboard_upcore_crex_pins[] = {
	UPBOARD_UPCORE_CREX_PIN_FUNC(0, I2C0_SDA, &upboard_i2c0_reg),
	UPBOARD_UPCORE_CREX_PIN_FUNC(0, I2C0_SCL, &upboard_i2c0_reg),
	UPBOARD_UPCORE_CREX_PIN_FUNC(0, I2C1_SDA, &upboard_i2c1_reg),
	UPBOARD_UPCORE_CREX_PIN_FUNC(0, I2C1_SCL, &upboard_i2c1_reg),
	UPBOARD_UPCORE_CREX_PIN_NAME(0, SPI2_CS0),
	UPBOARD_UPCORE_CREX_PIN_NAME(0, SPI2_CS1),
	UPBOARD_UPCORE_CREX_PIN_NAME(0, SPI2_MOSI),
	UPBOARD_UPCORE_CREX_PIN_NAME(0, SPI2_MISO),
	UPBOARD_UPCORE_CREX_PIN_NAME(0, SPI2_CLK),
	UPBOARD_UPCORE_CREX_PIN_NAME(0, UART1_TXD),
	UPBOARD_UPCORE_CREX_PIN_NAME(0, UART1_RXD),
	UPBOARD_UPCORE_CREX_PIN_NAME(0, PWM0),
	UPBOARD_UPCORE_CREX_PIN_NAME(0, PWM1),
	UPBOARD_UPCORE_CREX_PIN_NAME(0, I2S2_FRM),
	UPBOARD_UPCORE_CREX_PIN_NAME(0, I2S2_CLK),
	UPBOARD_UPCORE_CREX_PIN_NAME(0, I2S2_RX),
	/* register 1 */
	UPBOARD_UPCORE_CREX_PIN_NAME(1, I2S2_TX),
	UPBOARD_UPCORE_CREX_PIN_NAME(1, GPIO0),
	UPBOARD_UPCORE_CREX_PIN_FUNC(1, GPIO2, &upboard_adc0_reg),
	UPBOARD_UPCORE_CREX_PIN_NAME(1, GPIO3),
	UPBOARD_UPCORE_CREX_PIN_NAME(1, GPIO4),
	UPBOARD_UPCORE_CREX_PIN_NAME(1, GPIO9),
};

static unsigned int upboard_upcore_crex_rpi_mapping[] = {
	UPBOARD_UPCORE_CREX_BIT_TO_PIN(0, I2C0_SDA),
	UPBOARD_UPCORE_CREX_BIT_TO_PIN(0, I2C0_SCL),
	UPBOARD_UPCORE_CREX_BIT_TO_PIN(0, I2C1_SDA),
	UPBOARD_UPCORE_CREX_BIT_TO_PIN(0, I2C1_SCL),
	UPBOARD_UPCORE_CREX_BIT_TO_PIN(1, GPIO0),
	UPBOARD_UPCORE_CREX_BIT_TO_PIN(1, GPIO2),
	UPBOARD_UPCORE_CREX_BIT_TO_PIN(1, GPIO3),
	UPBOARD_UPCORE_CREX_BIT_TO_PIN(0, SPI2_CS1),
	UPBOARD_UPCORE_CREX_BIT_TO_PIN(0, SPI2_CS0),
	UPBOARD_UPCORE_CREX_BIT_TO_PIN(0, SPI2_MISO),
	UPBOARD_UPCORE_CREX_BIT_TO_PIN(0, SPI2_MOSI),
	UPBOARD_UPCORE_CREX_BIT_TO_PIN(0, SPI2_CLK),
	UPBOARD_UPCORE_CREX_BIT_TO_PIN(0, PWM0),
	UPBOARD_UPCORE_CREX_BIT_TO_PIN(0, PWM1),
	UPBOARD_UPCORE_CREX_BIT_TO_PIN(0, UART1_TXD),
	UPBOARD_UPCORE_CREX_BIT_TO_PIN(0, UART1_RXD),
	UPBOARD_UPCORE_CREX_BIT_TO_PIN(1, GPIO9),
	UPBOARD_UPCORE_CREX_BIT_TO_PIN(1, GPIO4),
	UPBOARD_UPCORE_CREX_BIT_TO_PIN(0, I2S2_CLK),
	UPBOARD_UPCORE_CREX_BIT_TO_PIN(0, I2S2_FRM),
	UPBOARD_UPCORE_CREX_BIT_TO_PIN(0, I2S2_RX),
	UPBOARD_UPCORE_CREX_BIT_TO_PIN(1, I2S2_TX),
};

/*
 * Init patches applied to the registers until the BIOS sets proper defaults
 */
static const struct reg_sequence upboard_upcore_crex_reg_patches[] __initconst = {
	// enable I2C voltage-level shifters
	{ UPFPGA_REG_FUNC_EN0,
		BIT(UPFPGA_I2C0_EN) |
		BIT(UPFPGA_I2C1_EN)
	},
	// HAT function pins initially set as inputs
	{ UPFPGA_REG_GPIO_DIR0,
		BIT(UPFPGA_UPCORE_CREX_SPI2_MISO) |
		BIT(UPFPGA_UPCORE_CREX_UART1_RXD) |
		BIT(UPFPGA_UPCORE_CREX_I2S2_FRM) |
		BIT(UPFPGA_UPCORE_CREX_I2S2_CLK) |
		BIT(UPFPGA_UPCORE_CREX_I2S2_RX)
	},
};

static const struct upboard_bios upboard_upcore_crex_bios_info __initconst = {
	.patches = upboard_upcore_crex_reg_patches,
	.npatches = ARRAY_SIZE(upboard_upcore_crex_reg_patches),
};

/*
 * UP Core board + CRST02 carrier board data
 */

#define upboard_upcore_crst02_pins        upboard_upcore_crex_pins
#define upboard_upcore_crst02_rpi_mapping upboard_upcore_crex_rpi_mapping

/*
 * Init patches applied to the registers until the BIOS sets proper defaults
 */
static const struct reg_sequence upboard_upcore_crst02_reg_patches[] __initconst = {
	// enable I2C voltage-level shifters
	{ UPFPGA_REG_FUNC_EN0,
		BIT(UPFPGA_I2C0_EN) |
		BIT(UPFPGA_I2C1_EN)
	},
	// HAT function pins initially set as inputs
	{ UPFPGA_REG_GPIO_DIR0,
		BIT(UPFPGA_UPCORE_CREX_SPI2_MISO) |
		BIT(UPFPGA_UPCORE_CREX_UART1_RXD) |
		BIT(UPFPGA_UPCORE_CREX_I2S2_FRM) |
		BIT(UPFPGA_UPCORE_CREX_I2S2_CLK) |
		BIT(UPFPGA_UPCORE_CREX_I2S2_RX)
	},
	// HAT function pins initially enabled (i.e. not hi-Z)
	{ UPFPGA_REG_GPIO_EN0,
		BIT(UPFPGA_UPCORE_CREX_SPI2_CS0) |
		BIT(UPFPGA_UPCORE_CREX_SPI2_MOSI) |
		BIT(UPFPGA_UPCORE_CREX_SPI2_MISO) |
		BIT(UPFPGA_UPCORE_CREX_SPI2_CLK) |
		BIT(UPFPGA_UPCORE_CREX_UART1_TXD) |
		BIT(UPFPGA_UPCORE_CREX_UART1_RXD) |
		BIT(UPFPGA_UPCORE_CREX_PWM0) |
		BIT(UPFPGA_UPCORE_CREX_PWM1) |
		BIT(UPFPGA_UPCORE_CREX_I2S2_FRM) |
		BIT(UPFPGA_UPCORE_CREX_I2S2_CLK) |
		BIT(UPFPGA_UPCORE_CREX_I2S2_RX)
	},
	{ UPFPGA_REG_GPIO_EN1,
		BIT(UPFPGA_UPCORE_CREX_I2S2_TX)
	},
};

static const struct upboard_bios upboard_upcore_crst02_bios_info __initconst = {
	.patches = upboard_upcore_crst02_reg_patches,
	.npatches = ARRAY_SIZE(upboard_upcore_crst02_reg_patches),
};

static int upboard_set_mux(struct pinctrl_dev *pctldev, unsigned int function,
			   unsigned int group)
{
	return 0;
};

static int upboard_fpga_request_enable(struct pinctrl_dev *pctldev,
				       struct pinctrl_gpio_range *range,
				       unsigned int pin)
{
	const struct pin_desc * const pd = pin_desc_get(pctldev, pin);
	const struct upboard_pin *p;
	int ret=0;

	if (!pd)
		return -EINVAL;
	p = pd->drv_data;

	if (p->funcbit) {
		ret = regmap_field_write(p->funcbit, 0);
		if (ret)
			return ret;
	}

	if (p->enbit) {
		ret = regmap_field_write(p->enbit, 1);
		if (ret)
			return ret;
	}

	return 0;
};

static int upboard_fpga_request_free(struct pinctrl_dev *pctldev,
				       unsigned int pin)
{
	const struct pin_desc * const pd = pin_desc_get(pctldev, pin);
	const struct upboard_pin *p;
	int ret=0;

	if (!pd)
		return -EINVAL;
	p = pd->drv_data;

	if (p->funcbit) {
		ret = regmap_field_write(p->funcbit, 1);
		if (ret)
			return ret;
	}

	if (p->enbit) {
		ret = regmap_field_write(p->enbit, 0);
		if (ret)
			return ret;
	}

	return 0;
};

static int upboard_fpga_set_direction(struct pinctrl_dev *pctldev,
				      struct pinctrl_gpio_range *range,
				      unsigned int pin, bool input)
{
	const struct pin_desc * const pd = pin_desc_get(pctldev, pin);
	const struct upboard_pin *p;

	if (!pd)
		return -EINVAL;
	p = pd->drv_data;
	upboard_fpga_request_enable(pctldev,range,pin);		
	return regmap_field_write(p->dirbit, input);
};

static int upboard_get_functions_count(struct pinctrl_dev *pctldev)
{
	//dev_info(pctldev->dev,"upboard_get_functions_count");
	return 0;
}

static const char *upboard_get_function_name(struct pinctrl_dev *pctldev,
				     unsigned int selector)
{
	//dev_info(pctldev->dev,"upboard_get_function_name:%d",selector);
	return NULL;
}

static int upboard_get_function_groups(struct pinctrl_dev *pctldev,
			       unsigned int selector,
			       const char * const **groups,
			       unsigned int *num_groups)
{
	*groups = NULL;
	*num_groups = 0;
	return 0;
}

static const struct pinmux_ops upboard_pinmux_ops = {
	.get_functions_count = upboard_get_functions_count,
	.get_function_groups = upboard_get_function_groups,
	.get_function_name = upboard_get_function_name,
	.set_mux = upboard_set_mux,
	.gpio_request_enable = upboard_fpga_request_enable,
	.gpio_set_direction = upboard_fpga_set_direction,
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

static struct pinctrl_desc upboard_up_pinctrl_desc = {
	.pins = upboard_up_pins,
	.npins = ARRAY_SIZE(upboard_up_pins),
	.pctlops = &upboard_pinctrl_ops,
	.pmxops = &upboard_pinmux_ops,
	.owner = THIS_MODULE,
};

static struct pinctrl_desc upboard_up2_pinctrl_desc = {
	.pins = upboard_up2_pins,
	.npins = ARRAY_SIZE(upboard_up2_pins),
	.pctlops = &upboard_pinctrl_ops,
	.pmxops = &upboard_pinmux_ops,
	.owner = THIS_MODULE,
};

static struct pinctrl_desc upboard_upcore_crex_pinctrl_desc = {
	.pins = upboard_upcore_crex_pins,
	.npins = ARRAY_SIZE(upboard_upcore_crex_pins),
	.pctlops = &upboard_pinctrl_ops,
	.pmxops = &upboard_pinmux_ops,
	.owner = THIS_MODULE,
};

static struct pinctrl_desc upboard_upcore_crst02_pinctrl_desc = {
	.pins = upboard_upcore_crst02_pins,
	.npins = ARRAY_SIZE(upboard_upcore_crst02_pins),
	.pctlops = &upboard_pinctrl_ops,
	.pmxops = &upboard_pinmux_ops,
	.owner = THIS_MODULE,
};

static void upboard_alt_func_enable(struct gpio_chip *gc, const char* name)
{
	struct upboard_pinctrl *pctrl = container_of(gc, struct upboard_pinctrl, chip);
	int offset[pctrl->pctldesc->npins];
	int i,cnt;
	bool input = false;
	
	//find all pins
	for(i=0,cnt=0;i<pctrl->pctldesc->npins;i++){
		if(strstr(pctrl->pctldesc->pins[i].name,name)){
			offset[cnt++] = i;
		}
	}
	for(i=0;i<cnt;i++){
		if(gpio_request(pctrl->pins[offset[i]].gpio,module_name(THIS_MODULE)))
			return; //used
		else
			gpio_free(pctrl->pins[offset[i]].gpio);
	}
	//change to alternate function
	for(i=0;i<cnt;i++){
		if(pctrl->pins[offset[i]].regs==NULL)
			continue;

		if(strstr(pctrl->pctldesc->pins[offset[i]].name,"I2C")){
			int mode=1;
			switch(pctrl->ident)
			{
				case 15:
					mode=2;
				break;
			}
			unsigned int val = readl(pctrl->pins[offset[i]].regs) | mode<<PADCFG0_PMODE_SHIFT; 
			writel(val,pctrl->pins[offset[i]].regs);
			upboard_fpga_request_free(pctrl->pctldev,offset[i]);
			continue;
		}
		else if(strstr(pctrl->pctldesc->pins[offset[i]].name,"UART")){
			int mode=1;
			switch(pctrl->ident)
			{
				case 13:
				case 14:
					mode=4;
				break;
				case 15:
					mode=2;
				break;
			}
			unsigned int val = readl(pctrl->pins[offset[i]].regs) | mode<<PADCFG0_PMODE_SHIFT;
			writel(val,pctrl->pins[offset[i]].regs);	
		}
		else if(strstr(pctrl->pctldesc->pins[offset[i]].name,"SPI")){
			int mode=1;
			switch(pctrl->ident)
			{
				case 15:
					mode=7;
				break;
			}
			unsigned int val = readl(pctrl->pins[offset[i]].regs) | mode<<PADCFG0_PMODE_SHIFT;
			writel(val,pctrl->pins[offset[i]].regs);	
		}
		else if(strstr(pctrl->pctldesc->pins[offset[i]].name,"I2S")){
			int mode=1;
			switch(pctrl->ident)
			{
				case 15:
					mode=4;
				break;
			}
			unsigned int val = readl(pctrl->pins[offset[i]].regs) | mode<<PADCFG0_PMODE_SHIFT;
			writel(val,pctrl->pins[offset[i]].regs);	
		}
		else {
			unsigned int val = readl(pctrl->pins[offset[i]].regs) | 1<<PADCFG0_PMODE_SHIFT;
			writel(val,pctrl->pins[offset[i]].regs);	
		
		}
		
		//input pins
		if(strstr(pctrl->pctldesc->pins[offset[i]].name,"RX")){
			input = true;
		}
		if(strstr(pctrl->pctldesc->pins[offset[i]].name,"CTS")){
			input = true;
		}
		if(strstr(pctrl->pctldesc->pins[offset[i]].name,"ADC")){
			input = true;
		}
		if(strstr(pctrl->pctldesc->pins[offset[i]].name,"MISO")){
			input = true;
		}
		if(strstr(pctrl->pctldesc->pins[offset[i]].name,"DIN")){
			input = true;
		}
		if(strstr(pctrl->pctldesc->pins[offset[i]].name,"SDI")){
			input = true;
		}
		upboard_fpga_set_direction(pctrl->pctldev,NULL, offset[i], input);
	}
}

static int upboard_rpi_to_native_gpio(struct gpio_chip *gc, unsigned int gpio)
{
	struct upboard_pinctrl *pctrl = container_of(gc, struct upboard_pinctrl, chip);
	unsigned int pin = pctrl->rpi_mapping[gpio];

	struct pinctrl_gpio_range *range =
		pinctrl_find_gpio_range_from_pin(pctrl->pctldev, pin);


	if (!range)
		return -ENODEV;

	return range->base;
}

static int upboard_gpio_request(struct gpio_chip *gc, unsigned int offset)
{
	struct upboard_pinctrl *pctrl = container_of(gc, struct upboard_pinctrl, chip);
	int gpio = upboard_rpi_to_native_gpio(gc, offset);

	if (gpio < 0)
		return gpio;

	return gpio_request(gpio, module_name(THIS_MODULE));
}

static void upboard_gpio_free(struct gpio_chip *gc, unsigned int offset)
{
	struct upboard_pinctrl *pctrl = container_of(gc, struct upboard_pinctrl, chip);
	int gpio = upboard_rpi_to_native_gpio(gc, offset);
	unsigned int pin = pctrl->rpi_mapping[offset];
	char name[strlen(pctrl->pctldesc->pins[pin].name)]; 
	char *p;

	if (gpio < 0)
		return;

	gpio_free(gpio);

	strcpy(name,pctrl->pctldesc->pins[pin].name);
	p = name;
	upboard_alt_func_enable(gc,strsep(&p,"_"));		
}

static int upboard_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	int gpio = upboard_rpi_to_native_gpio(gc, offset);

	if (gpio < 0)
		return gpio;
	
	return gpio_get_value(gpio);
}

static void upboard_gpio_set(struct gpio_chip *gc, unsigned int offset, int
			     value)
{
	struct upboard_pinctrl *pctrl = container_of(gc, struct upboard_pinctrl, chip);
	unsigned int pin = pctrl->rpi_mapping[offset];
	int gpio = upboard_rpi_to_native_gpio(gc, offset);

	if (gpio < 0)
		return;

	gpio_set_value(gpio, value);

	//APL03 open drain GPIO
	if(pctrl->ident == 9) {
		if(pin==0 || pin ==1) { 
			int val = readl(pctrl->pins[pin].regs);
			if(value)
				val |= PADCFG0_GPIOTXDIS;
			else
				val &= ~ PADCFG0_GPIOTXDIS;
			writel(val,pctrl->pins[pin].regs);
		}
	}	
}

static int upboard_gpio_direction_input(struct gpio_chip *gc,
					unsigned int offset)
{
	int gpio = upboard_rpi_to_native_gpio(gc, offset);
	struct upboard_pinctrl *pctrl = container_of(gc, struct upboard_pinctrl, chip);
	unsigned int pin = pctrl->rpi_mapping[offset];
	struct pinctrl_gpio_range *range = pinctrl_find_gpio_range_from_pin(pctrl->pctldev, pin);

	upboard_fpga_set_direction(pctrl->pctldev,range,pin,true);
	
	if (gpio < 0)
		return gpio;

	return gpio_direction_input(gpio);
}

static int upboard_gpio_direction_output(struct gpio_chip *gc,
					 unsigned int offset, int value)
{
	int gpio = upboard_rpi_to_native_gpio(gc, offset);
	struct upboard_pinctrl *pctrl = container_of(gc, struct upboard_pinctrl, chip); 
	unsigned int pin = pctrl->rpi_mapping[offset];
	struct pinctrl_gpio_range *range = pinctrl_find_gpio_range_from_pin(pctrl->pctldev, pin);

	upboard_fpga_set_direction(pctrl->pctldev,range,pin,false);

	if (gpio < 0)
		return gpio;

	return gpio_direction_output(gpio, value);
}

static struct gpio_chip upboard_gpio_chip = {
	.label = "Raspberry Pi compatible UP GPIO",
	.base = 0,
	.request = upboard_gpio_request,
	.free = upboard_gpio_free,
	.get = upboard_gpio_get,
	.set = upboard_gpio_set,
	.direction_input = upboard_gpio_direction_input,
	.direction_output = upboard_gpio_direction_output,
	.owner = THIS_MODULE,
};

/* DMI Matches for older bios without fpga initialization */
static const struct dmi_system_id upboard_dmi_table[] __initconst = {
	{
		.matches = { /* UP */
			DMI_EXACT_MATCH(DMI_SYS_VENDOR, "AAEON"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "UP-CHT01"),
			DMI_EXACT_MATCH(DMI_BOARD_VERSION, "V0.4"),
		},
		.driver_data = (void *)&upboard_up_bios_info_dvt,
	},
	{
		.matches = { /* UP2 */
			DMI_EXACT_MATCH(DMI_SYS_VENDOR, "AAEON"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "UP-APL01"),
			DMI_EXACT_MATCH(DMI_BOARD_VERSION, "V0.3"),
		},
		.driver_data = (void *)&upboard_up2_bios_info_v0_3,
	},
	{
		.ident = 9,
		.matches = { /* UP2 */
			DMI_EXACT_MATCH(DMI_SYS_VENDOR, "AAEON"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "UP-APL03"),
		},		
	},	
	{
		.ident = 13,
		.matches = { /* UP 6000 */
			DMI_EXACT_MATCH(DMI_SYS_VENDOR, "AAEON"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "UPN-EHL01"),
		},		
	},	
	{
		.ident = 14,
		.matches = { /* UP squared v2 */
			DMI_EXACT_MATCH(DMI_SYS_VENDOR, "AAEON"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "UPS-EHL01"),
		},		
	},		
	{
		.ident = 15,
		.matches = { /* UP Xtreme i12 */
			DMI_EXACT_MATCH(DMI_SYS_VENDOR, "AAEON"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "UPX-ADLP01"),
		},		
	},		
	{ },
};

bool regmap_check_writeable(struct upboard_fpga *fpga, unsigned int reg)
{

	if (!fpga)
		return -ENOMEM;

	if (fpga->regmapconf->max_register && reg > fpga->regmapconf->max_register)
		return false;

	if (fpga->regmapconf->writeable_reg)
		return fpga->regmapconf->writeable_reg(fpga->dev, reg);

	if (fpga->regmapconf->wr_table)
		return regmap_check_range_table(fpga->regmap, reg, fpga->regmapconf->wr_table);
	
	return true;
}

static void __iomem *upboard_get_regs(struct gpio_chip *gc, unsigned int pin, unsigned int reg)
{
	struct platform_device *pdev = to_platform_device(gc->parent);
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	size_t nregs;

	//UP ADL/TGL
	if(strstr(dev_name(&pdev->dev),"INTC1055:00") || strstr(dev_name(&pdev->dev),"INT34C5:00")) {
		struct intel_pinctrl *intel_pctrl = gpiochip_get_data(gc);
		struct pinctrl_dev *pctldev = intel_pctrl->pctldev; //of_pinctrl_get(gc->parent->of_node);
	
		if(pctldev==NULL)
			return NULL;
		struct pinctrl_gpio_range *range = pinctrl_find_gpio_range_from_pin(pctldev, pin);
		
		if(range)
			pin = pin-range->pin_base;
		else
			return NULL;
	
		if(range->pin_base < 67){
			res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		}
		else if(range->pin_base < 171){
			res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		}
		else if(range->pin_base < 260){
			res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
		}
		else {
			res = platform_get_resource(pdev, IORESOURCE_MEM, 3);
		}
	}	
	if (IS_ERR(res)) {
		dev_info(gc->parent,"upboard resource get failed");	
		return NULL;
	}

	void __iomem *base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	u32 offset = readl(base+PADBAR);
	
	u32 value = readl(base + REVID);

	if (((value & REVID_MASK) >> REVID_SHIFT) >= 0x94)
		nregs=4;
	else
		nregs=2;
		
	return base+offset+reg+pin*nregs*4;
}

int upboard_acpi_node_pin_mapping(struct upboard_fpga *fpga,
				struct upboard_pinctrl *pctrl,
			  	const char *propname,
			  	const char *pinctl_name,
			  	unsigned int pin_offset)
{
	struct gpio_descs *descs = devm_gpiod_get_array(fpga->dev, propname, GPIOD_ASIS);
	int ret=0, i;

	if (IS_ERR(descs)) {
		ret = PTR_ERR(descs);
		if (ret != -EPROBE_DEFER)
			dev_err(fpga->dev, "Failed to get %s gpios", propname);
		return ret;
	}

	for (i = 0; i < descs->ndescs; i++) {
		struct gpio_desc *desc = descs->desc[i];
		struct gpio_chip *gc = gpiod_to_chip(desc);

		//if (IS_ERR(desc))
		//	return PTR_ERR(desc);
		pctrl->pins[i].gpio = desc_to_gpio(desc);
		pctrl->pins[i].base = gc->base;
		pctrl->pins[i].irq = gpiod_to_irq(desc);
		pctrl->pins[i].regs = upboard_get_regs(gc,desc_to_gpio(desc)-gc->base,PADCFG0);
		
		/* The GPIOs may not be contiguous, so add them 1-by-1 */
		ret = gpiochip_add_pin_range(gpiod_to_chip(desc), pinctl_name,
					     desc_to_gpio(desc)-gc->base,
					     pin_offset+i, 1);
		
		if (ret)
			return ret;

	}

	//dispose acpi resource
	devm_gpiod_put_array(fpga->dev,descs);

	return ret;
}

static void upboard_irq_ack(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	dev_info(NULL,"upboard_irq_ack");
}

static void upboard_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	dev_info(NULL,"upboard_irq_mask");
}

static void upboard_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	switch (irqd_get_trigger_type(d)) {
	case IRQ_TYPE_LEVEL_HIGH:
		//value |= BYT_TRIG_LVL;
		//fallthrough;
	case IRQ_TYPE_EDGE_RISING:
		//value |= BYT_TRIG_POS;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		//value |= BYT_TRIG_LVL;
		//fallthrough;
	case IRQ_TYPE_EDGE_FALLING:
		//value |= BYT_TRIG_NEG;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		//value |= (BYT_TRIG_NEG | BYT_TRIG_POS);
		break;
	}	
	
	dev_info(NULL,"upboard_irq_unmask");
}

static int upboard_irq_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	dev_info(NULL,"upboard_irq_type");
	
	return 0;
}

static irqreturn_t upboard_gpio_irq_handler(int irq, void *data)
{
	//struct up_pin_info *pin = (struct up_pin_info *)data;

	generic_handle_irq(irq);
	return IRQ_HANDLED;
}

static int upboard_irq_chip_set_type(struct irq_data *data, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct upboard_pinctrl *pctrl = container_of(gc, struct upboard_pinctrl, chip);
	unsigned offset = irqd_to_hwirq(data);

	data->parent_data = irq_get_irq_data(pctrl->pins[offset].irq);


	if(data->parent_data) {
		printk("%s: no NULL\n",__func__);
		return irq_chip_set_type_parent(data, type);
	} else {
		printk("%s: NULL\n",__func__);
		return 0;
	}
}

static struct irq_chip upboard_gpio_irqchip = {
	.name = "upboard-gpio-irq",
	.irq_ack = upboard_irq_ack,
	.irq_mask = upboard_irq_mask,
	.irq_unmask = upboard_irq_unmask,
	.irq_set_type = upboard_irq_chip_set_type,
	.flags = IRQCHIP_MASK_ON_SUSPEND | IRQCHIP_SKIP_SET_WAKE,
};

static int __init upboard_pinctrl_probe(struct platform_device *pdev)
{
	struct upboard_fpga * const fpga = dev_get_drvdata(pdev->dev.parent);
	struct acpi_device * const adev = ACPI_COMPANION(&pdev->dev);
	struct pinctrl_desc *pctldesc;
	const struct upboard_bios *bios_info = NULL;
	struct upboard_pinctrl *pctrl;
	struct upboard_pin *pins;
	const struct dmi_system_id *system_id;
	const char *hid;
	const unsigned int *rpi_mapping;
	unsigned ngpio;
	int ret;
	int i;

	if (!fpga)
		return -EINVAL;

	if (!adev)
		return -ENODEV;

	hid = acpi_device_hid(adev);
	if (!strcmp(hid, "AANT0F00") || !strcmp(hid, "AANT0F04") || !strcmp(hid, "AANT0000")) {
		pctldesc = &upboard_up_pinctrl_desc;
		rpi_mapping = upboard_up_rpi_mapping;
		ngpio  = ARRAY_SIZE(upboard_up_rpi_mapping);
	} else if (!strcmp(hid, "AANT0F01")) {
		pctldesc = &upboard_up2_pinctrl_desc;
		rpi_mapping = upboard_up2_rpi_mapping;
		ngpio  = ARRAY_SIZE(upboard_up2_rpi_mapping);
	} else if (!strcmp(hid, "AANT0F02")) {
		pctldesc = &upboard_upcore_crex_pinctrl_desc;
		rpi_mapping = upboard_upcore_crex_rpi_mapping;
		ngpio  = ARRAY_SIZE(upboard_upcore_crex_rpi_mapping);
		bios_info = &upboard_upcore_crex_bios_info;
	} else if (!strcmp(hid, "AANT0F03")) {
		pctldesc = &upboard_upcore_crst02_pinctrl_desc;
		rpi_mapping = upboard_upcore_crst02_rpi_mapping;
		ngpio  = ARRAY_SIZE(upboard_upcore_crst02_rpi_mapping);
		bios_info = &upboard_upcore_crst02_bios_info;
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
		struct upboard_pin *pin = &pins[i];
		struct pinctrl_pin_desc *pd = (struct pinctrl_pin_desc *)
			&pctldesc->pins[i];
		struct reg_field fldconf = {0};
		unsigned int regoff = (pd->number / UPFPGA_REGISTER_SIZE);
		unsigned int lsb = pd->number % UPFPGA_REGISTER_SIZE;

		pin->funcbit = NULL;
		if (pd->drv_data) {
			fldconf = *(struct reg_field *)pd->drv_data;
			if (!regmap_check_writeable(fpga, fldconf.reg))
				return -EINVAL;

			pin->funcbit = devm_regmap_field_alloc(&pdev->dev,
							       fpga->regmap,
							       fldconf);
			if (IS_ERR(pin->funcbit))
				return PTR_ERR(pin->funcbit);
		}

		pin->enbit = NULL;
		fldconf.reg = UPFPGA_REG_GPIO_EN0 + regoff;
		fldconf.lsb = lsb;
		fldconf.msb = lsb;

		/* some platform don't have enable bit, ignore if not present */
		if (regmap_check_writeable(fpga, fldconf.reg)) {
			pin->enbit = devm_regmap_field_alloc(&pdev->dev,
							     fpga->regmap,
							     fldconf);
			if (IS_ERR(pin->enbit))
				return PTR_ERR(pin->enbit);
		}

		fldconf.reg = UPFPGA_REG_GPIO_DIR0 + regoff;
		fldconf.lsb = lsb;
		fldconf.msb = lsb;

		if (!regmap_check_writeable(fpga, fldconf.reg))
			return -EINVAL;

		pin->dirbit = devm_regmap_field_alloc(&pdev->dev,
						      fpga->regmap,
						      fldconf);
		if (IS_ERR(pin->dirbit))
			return PTR_ERR(pin->dirbit);

		pd->drv_data = pin;
	}

	/* create a new pinctrl device and register it */
	pctrl = devm_kzalloc(&pdev->dev, sizeof(*pctrl), GFP_KERNEL);
	if (!pctrl)
		return -ENOMEM;

	pctrl->regmap = fpga->regmap;
	pctrl->rpi_mapping = rpi_mapping;
	pctrl->chip = upboard_gpio_chip;
	pctrl->pctldesc = pctldesc;
	pctrl->chip.parent = &pdev->dev;
	pctrl->chip.ngpio = ngpio;
	pctrl->pins = pins;
	pctrl->chip.irq.chip = &upboard_gpio_irqchip;
	
	ret = devm_gpiochip_add_data(&pdev->dev, &pctrl->chip, pctrl);
	if (ret)
		return ret;

	pctrl->pctldev = devm_pinctrl_register(&pdev->dev, pctldesc, pctrl);
	if (IS_ERR(pctrl->pctldev))
		return PTR_ERR(pctrl->pctldev);

	/* add acpi pin mapping according to external-gpios key */
	ret = upboard_acpi_node_pin_mapping(fpga, pctrl, 
					"external",
					dev_name(&pdev->dev),
					0);
					
	if (ret)
		return ret;

	if (! bios_info) {
		/* check for special board versions that require register patches */
		system_id = dmi_first_match(upboard_dmi_table);
		if (system_id) {
			bios_info = system_id->driver_data;
			pctrl->ident = system_id->ident;
		}
		
	}

	if (bios_info && bios_info->patches) {
		ret = regmap_register_patch(pctrl->regmap,
					    bios_info->patches,
					    bios_info->npatches);
		if (ret)
			return ret;
	}
	
	upboard_alt_func_enable(&pctrl->chip,"I2C");
	upboard_alt_func_enable(&pctrl->chip,"SPI");
	upboard_alt_func_enable(&pctrl->chip,"SPI1");
	upboard_alt_func_enable(&pctrl->chip,"SPI2");
	upboard_alt_func_enable(&pctrl->chip,"UART");
	upboard_alt_func_enable(&pctrl->chip,"I2S");
	upboard_alt_func_enable(&pctrl->chip,"PWM");
	upboard_alt_func_enable(&pctrl->chip,"ADC");
	upboard_alt_func_enable(&pctrl->chip,"NONAME");	//for UP2 I2C pins
	
	//display mapping info.
	//for(i=0;i<pctldesc->npins;i++){
	//	dev_info(&pdev->dev,"Name:%s, GPIO:%d, IRQ:%d, regs:0x%08x",
	//	pctldesc->pins[i].name,pins[i].gpio, pins[i].irq, pins[i].regs);
	//	if(pins[i].regs)
	//		dev_info(&pdev->dev,"val:%pS", readl(pins[i].regs));
	//}
		
	return ret;
}

static struct platform_driver upboard_pinctrl_driver = {
	.driver = {
		.name = "upboard-pinctrl",
	},
};

module_platform_driver_probe(upboard_pinctrl_driver, upboard_pinctrl_probe);

MODULE_AUTHOR("Gary Wang <garywang@aaeon.com.tw>");
MODULE_AUTHOR("Javier Arteaga <javier@emutex.com>");
MODULE_AUTHOR("Dan O'Donovan <dan@emutex.com>");
MODULE_AUTHOR("Nicola Lunghi <nicolal@emutex.com>");
MODULE_DESCRIPTION("UP Board HAT pin controller driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:upboard-pinctrl");
