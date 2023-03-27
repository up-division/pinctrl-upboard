// SPDX-License-Identifier: GPL-2.0-only
/*
 * UP Board HAT PWM driver 
 *
 * Copyright (c) AAEON. All rights reserved.
 *
 * Authors: Gary Wang <garywang@aaeon.com.tw>
 *
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "pwm-lpss.h"

#define PWM				0x00000000
#define PWM_ENABLE			BIT(31)
#define PWM_SW_UPDATE			BIT(30)
#define PWM_BASE_UNIT_SHIFT		8
#define PWM_ON_TIME_DIV_MASK		0x000000ff

/* Size of each PWM register space if multiple */
#define PWM_SIZE			0x400

static struct platform_device *up_pwm;

//PWM resource & info for UP WHL/TGL/APL/APLN
static struct resource pwm_res[] = { 
	{
	.name = "upboard-pwmctrl",
	.start = 0xFD6D0204,
	.end = 0xFD6D0214,
	.flags = IORESOURCE_MEM|IORESOURCE_MEM_WRITEABLE,
	},
};

static struct pwm_lpss_boardinfo up_pwm_info = {
	.clk_rate = 32768,
	.npwm = 1,
	.base_unit_bits = 22,
};

static inline struct pwm_lpss_chip *to_lpwm(struct pwm_chip *chip)
{
	return container_of(chip, struct pwm_lpss_chip, chip);
}

static inline u32 upboard_pwm_read(const struct pwm_device *pwm)
{
	struct pwm_lpss_chip *lpwm = to_lpwm(pwm->chip);

	return readl(lpwm->regs);
}

static inline void upboard_pwm_write(const struct pwm_device *pwm, u32 value)
{
	struct pwm_lpss_chip *lpwm = to_lpwm(pwm->chip);

	dev_info(NULL,"pwm value:%X",value);
	writel(value, lpwm->regs);
}

static void upboard_pwm_prepare(struct pwm_lpss_chip *lpwm, struct pwm_device *pwm,
			     int duty_ns, int period_ns)
{
	unsigned long long on_time_div;
	unsigned long c = lpwm->info->clk_rate, base_unit_range;
	unsigned long long base_unit, freq = NSEC_PER_SEC;
	u32 ctrl;

	do_div(freq, period_ns);

	/*
	 * The equation is:
	 * base_unit = round(base_unit_range * freq / c)
	 */
	base_unit_range = BIT(lpwm->info->base_unit_bits);
	freq *= base_unit_range;

	base_unit = DIV_ROUND_CLOSEST_ULL(freq, c);
	/* base_unit must not be 0 and we also want to avoid overflowing it */
	base_unit = clamp_val(base_unit, 1, base_unit_range - 1);

	on_time_div = 255ULL * duty_ns;
	do_div(on_time_div, period_ns);
	on_time_div = 255ULL - on_time_div;

	ctrl = upboard_pwm_read(pwm);
	ctrl &= ~PWM_ON_TIME_DIV_MASK;
	ctrl &= ~((base_unit_range - 1) << PWM_BASE_UNIT_SHIFT);
	ctrl |= (u32) base_unit << PWM_BASE_UNIT_SHIFT;
	ctrl |= on_time_div;

	//upboard_pwm_write(pwm, ctrl);
	upboard_pwm_write(pwm, ctrl | PWM_SW_UPDATE);
}

static int upboard_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			  const struct pwm_state *state)
{
	struct pwm_lpss_chip *lpwm = to_lpwm(chip);

	if (state->enabled) {
		upboard_pwm_write(pwm, upboard_pwm_read(pwm) & ~PWM_ENABLE & ~PWM_SW_UPDATE);
		upboard_pwm_prepare(lpwm, pwm, state->duty_cycle, state->period);
		upboard_pwm_write(pwm, upboard_pwm_read(pwm) | PWM_ENABLE);
	} else if (pwm_is_enabled(pwm)) {
		upboard_pwm_write(pwm, upboard_pwm_read(pwm) & ~PWM_ENABLE & ~PWM_SW_UPDATE);
	}

	return 0;
}

static void upboard_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
			       struct pwm_state *state)
{
	struct pwm_lpss_chip *lpwm = to_lpwm(chip);
	unsigned long base_unit_range;
	unsigned long long base_unit, freq, on_time_div;
	u32 ctrl;

	base_unit_range = BIT(lpwm->info->base_unit_bits);

	ctrl = upboard_pwm_read(pwm);
	on_time_div = 255 - (ctrl & PWM_ON_TIME_DIV_MASK);
	base_unit = (ctrl >> PWM_BASE_UNIT_SHIFT) & (base_unit_range - 1);

	freq = base_unit * lpwm->info->clk_rate;
	do_div(freq, base_unit_range);
	if (freq == 0)
		state->period = NSEC_PER_SEC;
	else
		state->period = NSEC_PER_SEC / (unsigned long)freq;

	on_time_div *= state->period;
	do_div(on_time_div, 255);
	state->duty_cycle = on_time_div;

	state->polarity = PWM_POLARITY_NORMAL;
	state->enabled = !!(ctrl & PWM_ENABLE);
}

static const struct pwm_ops pwm_lpss_ops = {
	.apply = upboard_pwm_apply,
	.get_state = upboard_pwm_get_state,
	.owner = THIS_MODULE,
};

struct pwm_lpss_chip *upboard_pwm_probe(struct device *dev, struct resource *r,
				     const struct pwm_lpss_boardinfo *info)
{
	struct pwm_lpss_chip *lpwm;
	unsigned long c;
	int i, ret;
	u32 ctrl;

	lpwm = devm_kzalloc(dev, sizeof(*lpwm), GFP_KERNEL);
	if (!lpwm)
		return ERR_PTR(-ENOMEM);

	lpwm->regs = devm_ioremap(&up_pwm->dev, pwm_res[0].start, resource_size(pwm_res));
	if (IS_ERR(lpwm->regs))
		return ERR_CAST(lpwm->regs);

	lpwm->info = info;

	c = lpwm->info->clk_rate;
	if (!c)
		return ERR_PTR(-EINVAL);

	lpwm->chip.dev = dev;
	lpwm->chip.ops = &pwm_lpss_ops;
	lpwm->chip.npwm = info->npwm;

	ret = pwmchip_add(&lpwm->chip);

	if (ret) {
		dev_err(dev, "failed to add PWM chip: %d\n", ret);
		return ERR_PTR(ret);
	}

	for (i = 0; i < lpwm->info->npwm; i++) {
		ctrl = upboard_pwm_read(&lpwm->chip.pwms[i]);
	}

	return lpwm;
}

static void __exit
upboard_pwm_exit(void)
{
	platform_device_unregister(up_pwm);	
}

module_exit(upboard_pwm_exit);

void upboard_pwm_register(void)
{
	up_pwm = platform_device_alloc("upboard-pwm", -1);
	if (IS_ERR(up_pwm)) {
		platform_device_unregister(up_pwm);	
		return;
	}	
	if(platform_device_add(up_pwm)) {
		platform_device_unregister(up_pwm);	
		return;
	}			
	if(IS_ERR(upboard_pwm_probe(&up_pwm->dev,pwm_res,&up_pwm_info))) {
		platform_device_unregister(up_pwm);
		return;				
	}
}

EXPORT_SYMBOL_GPL(upboard_pwm_register);

MODULE_DESCRIPTION("PWM driver for UP Boards");
MODULE_AUTHOR("Gary Wang <garywang@aaeon.com.tw>");
MODULE_LICENSE("GPL v2");
