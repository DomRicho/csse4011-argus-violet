#ifndef MY_NETWORK_H
#define MY_NETWORK_H

#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/wifi.h>

// defines a WIFI_SSID and WIFI_PASS
#include "secrets.h"

#define CAM_SERVER_PORT 5000
#define SERVO_SERVER_PORT 5050
#define CAM_IP "10.78.174.50"
#define SERVO_IP "10.78.174.51"
#define BRIDGE_IP "10.78.174.52"
#define PC_IP "10.78.174.170"
#define GW_IP "10.78.174.89"

int init_ip_net(const char *my_addr);
void init_wifi(void);
int start_udp(int port, const char *dest_addr);

#endif
