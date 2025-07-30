// SPDX-License-Identifier: GPL-2.0-only
/*
 * check kernel driver proto types 
 *
 * Copyright (c) AAEON. All rights reserved.
 *
 * Authors: Gary Wang <garywang@aaeon.com.tw>
 *
 */
#define CHECK_PLATFORM_DRIVER

#ifdef CHECK_PWM
#include <linux/pwm.h>
#include <linux/types.h>
static void _check_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
			       struct pwm_state *state)
{
    return;
}

static const struct pwm_ops pwm_lpss_ops = {
	.get_state = _check_get_state,
};
#endif //CHECK_PWM

#ifdef CHECK_GPIO
#include <linux/pinctrl/consumer.h>
void func(void)
{
    pinctrl_gpio_request(0);
}
#endif //CHECK_GPIO

#ifdef CHECK_SPI_CHIP_SELECT
#include <linux/spi/spi.h>
void func(void)
{
	struct spi_device *spi;
	int cs = *spi->chip_select;
}
#endif //CHECK_SPI_CHIP_SELECT

#ifdef CHECK_SPI_NAMING
#include <linux/spi/pxa2xx_spi.h>
void foo(void)
{
	struct pxa2xx_spi_controller ctrl;
	ctrl.is_slave=false;
}
#endif //CHECK_SPI_NAMING

#ifdef CHECK_SPI_SSP
#include <linux/pxa2xx_ssp.h>
void foo(void)
{
	struct ssp_device ssp;
	ssp.pdev = NULL;
}
#endif //CHECK_SPI_SSP

#ifdef CHECK_PWM_PDEV
#include <linux/device.h>
#include <linux/pwm.h>
void foo(struct device *dev)
{
	struct pwm_chip chip;
	chip.dev = dev;
}
#endif //CHECK_PWM_DEV_POINT
 
#ifdef CHECK_PLATFORM_DRIVER
#include <linux/platform_device.h>
static void platform_remove_new(struct platform_device *pdev)
{
    return; 
}
static struct platform_driver driver = {
	.remove = platform_remove_new,
};
#endif //CHECK_PLATFORM_DRIVER

