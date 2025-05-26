/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Sample app to demonstrate PWM-based servomotor control
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/socket.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/shell/shell.h>

#include <zephyr/logging/log.h>

#include "common/my_network.h" // For wifi credentials
#include "zephyr/net/wifi.h"
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

void network_thread(void);
int servo_thread(void);

static struct net_mgmt_event_callback wifi_cb;
static bool wifi_connected = false;
 
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

static void wifi_connect(void);

static ssize_t sendall(int sock, const void *buf, size_t len)
{
    while (len) {
        ssize_t out_len = zsock_send(sock, buf, len, 0);
        if (out_len < 0) {
            LOG_ERR("Error %zd %d: %p", out_len, errno, buf);
            k_msleep(100);
            return out_len;
        }
        buf = (const char *)buf + out_len;
        len -= out_len;
    }
    return 0;
}

static void wifi_connect_handler(struct net_mgmt_event_callback *cb,
    uint32_t mgmt_event, struct net_if *iface)
{
    if (mgmt_event == NET_EVENT_WIFI_CONNECT_RESULT) {
        struct wifi_status *status = (struct wifi_status *)cb->info;
        if (status && status->status == 0) {
            wifi_connected = true;
            LOG_INF("Wi-Fi connected");
        } else {
            LOG_ERR("Wi-Fi connection failed\n");
        }
    } else if (mgmt_event == NET_EVENT_WIFI_DISCONNECT_RESULT) {
        wifi_connected = false;
        LOG_WRN("Wi-Fi disconnected, attempting reconnect...");
        k_sleep(K_SECONDS(2)); 
        wifi_connect();
    } else {
        LOG_ERR("Unknown Wi-Fi event: %d", mgmt_event);
    }
} 

static void wifi_connect(void)
{
    LOG_INF("Connecting to Wi-Fi...");
    struct net_if *iface = net_if_get_default();
    struct wifi_connect_req_params params = {
        .ssid = WIFI_SSID,
        .ssid_length = strlen(WIFI_SSID),
        .psk = WIFI_PASS,
        .psk_length = strlen(WIFI_PASS),
        .band = WIFI_FREQ_BAND_2_4_GHZ,
        .security = WIFI_SECURITY_TYPE_PSK,
        .channel = WIFI_CHANNEL_ANY,
        .timeout = SYS_FOREVER_MS,
    };
    
    net_mgmt_init_event_callback(&wifi_cb, wifi_connect_handler,
        NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT);
    net_mgmt_add_event_callback(&wifi_cb);

    int ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &params, sizeof(params));
    if (ret) {
        printk("Wi-Fi connect request failed: %d\n", ret);
    }
}
void network_thread(void)
{
    wifi_connect();
    while (!wifi_connected) {
        k_sleep(K_SECONDS(1));
    }
    struct net_if *iface = net_if_get_default();

    struct in_addr addr, netmask, gw;

    net_addr_pton(AF_INET, SERVO_IP, &addr);
    net_addr_pton(AF_INET, "255.255.255.0", &netmask);
    net_addr_pton(AF_INET, GW_IP, &gw);  // Gateway, probably your PC/router

    net_if_ipv4_addr_add(iface, &addr, NET_ADDR_MANUAL, 0);
    net_if_ipv4_set_netmask_by_addr(iface, &addr, &netmask);
    net_if_ipv4_set_gw(iface, &gw);

    while (!net_if_is_up(iface)) {
        k_sleep(K_MSEC(100));
    }

    int sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        LOG_ERR("Failed to create socket");
        return;
    }

    struct sockaddr_in sock_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SERVO_SERVER_PORT),
        .sin_addr.s_addr = INADDR_ANY
    };
    if (zsock_bind(sock, (struct sockaddr *)&sock_addr, sizeof(sock_addr)) < 0) {
        LOG_ERR("Bind failed");
        return;
    }

    // Set up destination address (Python server)
    struct sockaddr_in dest_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SERVO_SERVER_PORT),
    };
    zsock_inet_pton(AF_INET, BRIDGE_IP, &dest_addr.sin_addr);
    if (zsock_connect(sock, (const struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0) {
        LOG_ERR("Connect failed");
        zsock_close(sock);
        return;
    }

    char buf[16];  // Make it a bit larger than needed
    int size = 0;
    while (1) {
        struct servo_cmd_t *cmd = k_malloc(sizeof(struct servo_cmd_t));
        if (!cmd) {
            LOG_ERR("Failed to allocate servo_cmd_t");
            k_sleep(K_MSEC(100));
            continue;
        }

        size = zsock_recv(sock, buf, sizeof(buf) - 1, 0);
        if (size <= 0) {
            LOG_ERR("recv failed: %d", size);
            k_free(cmd);
            continue;
        }

        buf[size] = '\0';  // Safe now

        char dir[8] = {0};  // Ensure null-termination
        int mag = 0;
        char *colon = strchr(buf, ':');
        if (colon) {
            size_t len = colon - buf;
            if (len < sizeof(dir)) {
                strncpy(dir, buf, len);
                dir[len] = '\0';
                mag = atoi(colon + 1);

            } else {
                LOG_ERR("Direction too long");
            }
        } else {
            LOG_ERR("Invalid format, missing ':'");
        }        

        cmd->mag = mag;

        if (strcmp(dir, "left") == 0) {
            cmd->dir = LEFT;
        } else if (strcmp(dir, "right") == 0) {
            cmd->dir = RIGHT;
        } else if (strcmp(dir, "up") == 0) {
            cmd->dir = UP;
        } else if (strcmp(dir, "down") == 0) {
            cmd->dir = DOWN;
        } else {
            LOG_ERR("Invalid direction: %s", dir);
            k_free(cmd);
            continue;
        }

        k_fifo_put(&servo_cmd_q, cmd);
        k_sleep(K_MSEC(100));
    }
}

int servo_thread(void)
{
    int ret = 0; 
	uint32_t pan_pw = (min_pulse + max_pulse) / 2;
	uint32_t tilt_pw = (min_pulse + max_pulse) / 2;
	/*uint32_t pan_pw = min_pulse;*/
	/*uint32_t tilt_pw = min_pulse;*/
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
    while(1) {
        cmd = k_fifo_get(&servo_cmd_q, K_FOREVER);
        if (cmd == NULL) {
            LOG_ERR("NULL cmd");
            continue;
        }
        switch (cmd->dir) {
            case DOWN:
                if (!(tilt_pw >= max_pulse)) {
                    tilt_pw += cmd->mag * STEP; 
                    ret = pwm_set_pulse_dt(&tilt_servo, tilt_pw);
                    if (ret < 0) {
                        printk("Error %d: failed to set pulse width\n", ret);
                    }
                }
                break;
            case LEFT:
                if (!(pan_pw >= max_pulse)) {
                    pan_pw += cmd->mag * STEP; 
                    ret = pwm_set_pulse_dt(&pan_servo, pan_pw);
                    if (ret < 0) {
                        printk("Error %d: failed to set pulse width\n", ret);
                    }
                }
                break;
            case RIGHT:
                if (!(pan_pw <= min_pulse)) {
                    pan_pw -= cmd->mag * STEP; 
                    ret = pwm_set_pulse_dt(&pan_servo, pan_pw);
                    if (ret < 0) {
                        printk("Error %d: failed to set pulse width\n", ret);
                    }
                }
                break;
            case UP:
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
