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
#include <zephyr/logging/log.h>

void network_thread(void);
int servo_thread(void);

#define SERVO_STACK 2048
#define SERVO_PRIORITY 5
#define NETWORK_STACK 2048
#define NETWORK_PRIORITY 5

#define STEP PWM_USEC(100)

K_FIFO_DEFINE(servo_cmd_q);

LOG_MODULE_REGISTER(servo_node, LOG_LEVEL_DBG);

static const struct pwm_dt_spec tilt_servo = PWM_DT_SPEC_GET(DT_NODELABEL(tilt_servo));
static const struct pwm_dt_spec pan_servo = PWM_DT_SPEC_GET(DT_NODELABEL(pan_servo));
static const uint32_t min_pulse = DT_PROP(DT_NODELABEL(pan_servo), min_pulse);
static const uint32_t max_pulse = DT_PROP(DT_NODELABEL(pan_servo), max_pulse);

enum direction {
	DOWN,
	UP,
    LEFT,
    RIGHT
};

struct servo_cmd_t {
    void *res;
    enum direction dir;
    int mag; 
};

K_THREAD_DEFINE(servo_tid, SERVO_STACK, servo_thread, NULL, NULL, NULL, SERVO_PRIORITY, 0, 0);
K_THREAD_DEFINE(network_tid, NETWORK_STACK, network_thread, NULL, NULL, NULL, NETWORK_PRIORITY, 0, 0);

void network_thread(void)
{
    int dir_change = 0;
    int count = 0;
    while(1) {
        struct servo_cmd_t *cmd = k_malloc(sizeof(struct servo_cmd_t));
        if (cmd == NULL) {
            LOG_ERR("CMD NULL P");
            continue;
        }
        cmd->mag = 1;
        switch (dir_change) {
            case 0:
                cmd->dir = UP;
                break;
            case 1:
                cmd->dir = RIGHT;
                break;
            case 2:
                cmd->dir = DOWN;
                break;
            case 3:
                cmd->dir = LEFT;
                break;
            default:
                dir_change = 0;
                break;
        }
        count++;
        if (count >= 25) {
            dir_change++;
            count = 0;
        }
        k_fifo_put(&servo_cmd_q, cmd);
        k_msleep(100);
    }
}

int servo_thread(void)
{
    int ret = 0; 
	/*uint32_t pan_pw = (min_pulse + max_pulse) / 2;*/
	/*uint32_t tilt_pw = (min_pulse + max_pulse) / 2;*/
	uint32_t pan_pw = min_pulse;
	uint32_t tilt_pw = min_pulse;
    struct servo_cmd_t *cmd; 
	if (!pwm_is_ready_dt(&pan_servo)) {
		printk("Error: PWM device %s is not ready\n", pan_servo.dev->name);
		return 0;
	}
	if (!pwm_is_ready_dt(&tilt_servo)) {
		printk("Error: PWM device %s is not ready\n", tilt_servo.dev->name);
		return 0;
	}
    ret = pwm_set_pulse_dt(&pan_servo, pan_pw);
    if (ret < 0) {
        printk("Error %d: failed to set pulse width\n", ret);
        return 0;
    }
    ret = pwm_set_pulse_dt(&tilt_servo, tilt_pw);
    if (ret < 0) {
        printk("Error %d: failed to set pulse width\n", ret);
        return 0;
    }
    k_msleep(2500);
    while(1) {
        cmd = k_fifo_get(&servo_cmd_q, K_FOREVER);
        if (cmd == NULL) {
            LOG_ERR("NULL cmd");
            continue;
        }
        switch (cmd->dir) {
            case UP:
                if (!(tilt_pw >= max_pulse)) {
                    tilt_pw += cmd->mag * STEP; 
                    ret = pwm_set_pulse_dt(&tilt_servo, tilt_pw);
                    if (ret < 0) {
                        printk("Error %d: failed to set pulse width\n", ret);
                    }
                }
                break;
            case RIGHT:
                if (!(pan_pw >= max_pulse)) {
                    pan_pw += cmd->mag * STEP; 
                    ret = pwm_set_pulse_dt(&pan_servo, pan_pw);
                    if (ret < 0) {
                        printk("Error %d: failed to set pulse width\n", ret);
                    }
                }
                break;
            case LEFT:
                if (!(pan_pw <= min_pulse)) {
                    pan_pw -= cmd->mag * STEP; 
                    ret = pwm_set_pulse_dt(&pan_servo, pan_pw);
                    if (ret < 0) {
                        printk("Error %d: failed to set pulse width\n", ret);
                    }
                }
                break;
            case DOWN:
                if (!(tilt_pw <= min_pulse)) {
                    tilt_pw -= cmd->mag * STEP; 
                    ret = pwm_set_pulse_dt(&tilt_servo, tilt_pw);
                    if (ret < 0) {
                        printk("Error %d: failed to set pulse width\n", ret);
                    }
                }
                break;
            default:
                break;
        }
        k_free(cmd);
    }
}
