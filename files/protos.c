// SPDX-License-Identifier: GPL-2.0-only
/*
 * UP Board HAT PWM driver 
 *
 * Copyright (c) AAEON. All rights reserved.
 *
 * Authors: Gary Wang <garywang@aaeon.com.tw>
 *
 */
 
#include <linux/pwm.h>
#include <linux/types.h>

//testing build pwm prototypes 
static void _check_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
			       struct pwm_state *state)
{
  return;
}

static const struct pwm_ops pwm_lpss_ops = {
	.get_state = _check_get_state,
};


