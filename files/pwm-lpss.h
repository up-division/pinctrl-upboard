/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Intel Low Power Subsystem PWM controller driver
 *
 * Copyright (C) 2014, Intel Corporation
 *
 * Derived from the original pwm-lpss.c
 */

#ifndef __PWM_LPSS_H
#define __PWM_LPSS_H

#include <linux/device.h>
#include <linux/pwm.h>
#include <linux/version.h>

#define MAX_PWMS			4
#define LPSS_MAX_PWMS			MAX_PWMS


struct pwm_lpss_chip {
	void __iomem *regs;
	const struct pwm_lpss_boardinfo *info;
	struct pwm_chip chip;
};

struct pwm_lpss_boardinfo {
	unsigned long clk_rate;
	unsigned int npwm;
	unsigned long base_unit_bits;
	bool bypass;
	/*
	 * On some devices the _PS0/_PS3 AML code of the GPU (GFX0) device
	 * messes with the PWM0 controllers state,
	 */
	bool other_devices_aml_touches_pwm_regs;
};

/* BayTrail */
const struct pwm_lpss_boardinfo pwm_lpss_byt_info = {
	.clk_rate = 25000000,
	.npwm = 1,
	.base_unit_bits = 16,
};
//EXPORT_SYMBOL_GPL(pwm_lpss_byt_info);

/* Braswell */
const struct pwm_lpss_boardinfo pwm_lpss_bsw_info = {
	.clk_rate = 19200000,
	.npwm = 1,
	.base_unit_bits = 16,
	.other_devices_aml_touches_pwm_regs = true,
};
//EXPORT_SYMBOL_GPL(pwm_lpss_bsw_info);

/* Broxton */
const struct pwm_lpss_boardinfo pwm_lpss_bxt_info = {
	.clk_rate = 19200000,
	.npwm = 4,
	.base_unit_bits = 22,
	.bypass = true,
};
//EXPORT_SYMBOL_GPL(pwm_lpss_bxt_info);

/* Tangier */
const struct pwm_lpss_boardinfo pwm_lpss_tng_info = {
	.clk_rate = 19200000,
	.npwm = 4,
	.base_unit_bits = 22,
};
//EXPORT_SYMBOL_GPL(pwm_lpss_tng_info);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 9, 0)
struct pwm_chip *upboard_pwm_lpss_probe(struct device *dev, void __iomem *base,
				     const struct pwm_lpss_boardinfo *info);
#else
struct pwm_lpss_chip *upboard_pwm_lpss_probe(struct device *dev, void __iomem *base,
					  const struct pwm_lpss_boardinfo *info);
#endif
#endif	/* __PWM_LPSS_H */
