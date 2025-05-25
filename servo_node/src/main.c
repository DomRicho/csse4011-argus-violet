/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Sample app to demonstrate PWM-based servomotor control
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

static const struct pwm_dt_spec tilt_servo = PWM_DT_SPEC_GET(DT_NODELABEL(tilt_servo));
static const struct pwm_dt_spec pan_servo = PWM_DT_SPEC_GET(DT_NODELABEL(pan_servo));
static const uint32_t min_pulse = DT_PROP(DT_NODELABEL(pan_servo), min_pulse);
static const uint32_t max_pulse = DT_PROP(DT_NODELABEL(pan_servo), max_pulse);

#define STEP PWM_USEC(100)

enum direction {
	DOWN,
	UP,
};

int main(void)
{
	uint32_t pulse_width = min_pulse;
	enum direction dir = UP;
	int ret;

	printk("Servomotor control\n");

	if (!pwm_is_ready_dt(&pan_servo)) {
		printk("Error: PWM device %s is not ready\n", pan_servo.dev->name);
		return 0;
	}
	if (!pwm_is_ready_dt(&tilt_servo)) {
		printk("Error: PWM device %s is not ready\n", tilt_servo.dev->name);
		return 0;
	}

	while (1) {
		/*ret = pwm_set_pulse_dt(&servo, pulse_width);*/
        /*printk("%s :: chl %d, T=%u, PW=%u\n", pan_servo.dev->name, pan_servo.channel, pan_servo.period, pulse_width);*/
        ret = pwm_set_pulse_dt(&pan_servo, pulse_width);
        ret = pwm_set_pulse_dt(&tilt_servo, pulse_width);
		if (ret < 0) {
			printk("Error %d: failed to set pulse width\n", ret);
			return 0;
		}
        /*printk("%s :: chl %d, T=%u, PW=%u\n", tilt_servo.dev->name, tilt_servo.channel, tilt_servo.period, pulse_width);*/

		if (dir == DOWN) {
			if (pulse_width <= min_pulse) {
				dir = UP;
				pulse_width = min_pulse;
			} else {
				pulse_width -= STEP;
			}
		} else {
			pulse_width += STEP;

			if (pulse_width >= max_pulse) {
				dir = DOWN;
				pulse_width = max_pulse;
			}
		}
        printk("Pulse Width: %d\n", pulse_width);

		/*k_sleep(K_SECONDS(1));*/
        k_msleep(100);
	}
	return 0;
}

