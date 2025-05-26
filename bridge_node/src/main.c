#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/socket.h>

#include <zephyr/logging/log.h>

#include "common/my_network.h" // For wifi credentials
#include "zephyr/net/wifi.h"

#define CAM_STACKSIZE 2048
#define CAM_PRIORITY 5
#define SERVO_STACKSIZE 2048
#define SERVO_PRIORITY 5

#define LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(bridge_node, LOG_LEVEL);

static struct net_mgmt_event_callback wifi_cb;
static bool wifi_connected = false;

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

void network_thread(void) {
    /*int ret = 0;*/
    /*struct net_if_addr *addr_test;*/

    // Connect to Wi-Fi
    wifi_connect();
    while (!wifi_connected) {
        k_sleep(K_SECONDS(1));
    }
    struct net_if *iface = net_if_get_default();

    struct in_addr addr, netmask, gw;

    net_addr_pton(AF_INET, ESP32_IP, &addr);
    net_addr_pton(AF_INET, "255.255.255.0", &netmask);
    net_addr_pton(AF_INET, GW_IP, &gw);  // Gateway, probably your PC/router

    net_if_ipv4_addr_add(iface, &addr, NET_ADDR_MANUAL, 0);
    net_if_ipv4_set_netmask_by_addr(iface, &addr, &netmask);
    net_if_ipv4_set_gw(iface, &gw);

    while (!net_if_is_up(iface)) {
        k_sleep(K_MSEC(100));
    }

    // Set up destination address (Python server)
    struct sockaddr_in dest_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SERVER_PORT),
    };
    zsock_inet_pton(AF_INET, PC_IP, &dest_addr.sin_addr);

    int sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        LOG_ERR("Failed to create socket");
        return;
    }

    struct sockaddr_in sock_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SERVER_PORT),
        .sin_addr.s_addr = INADDR_ANY
    };

    if (zsock_bind(sock, (struct sockaddr *)&sock_addr, sizeof(sock_addr)) < 0) {
        LOG_ERR("Bind failed");
        return;
    }

    if (zsock_connect(sock, (const struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0) {
        LOG_ERR("Connect failed");
        zsock_close(sock);
        return;
    }

    int connected = 0;
    char buf[32];

    while (!connected) {
        zsock_send(sock, "CONN?", sizeof("CONN?"), 0);

        int r = zsock_recv(sock, buf, sizeof(buf) - 1, ZSOCK_MSG_DONTWAIT);
        printk("%d %32s\n", r, buf);
        if (r > 0) {
            buf[r] = '\0';
            LOG_INF("Received: %s", buf);
            if (strncmp(buf, "OK", 2) == 0) {
                connected = 1;
            }
        }

        k_msleep(1000);
    }

    while(1) k_msleep(2000);
    return;
}


void cam_client(void) 
{
    wifi_connect();
    while (!wifi_connected) {
        k_sleep(K_SECONDS(1));
    }
    struct net_if *iface = net_if_get_default();

    struct in_addr addr, netmask, gw;

    net_addr_pton(AF_INET, ESP32_IP, &addr);
    net_addr_pton(AF_INET, "255.255.255.0", &netmask);
    net_addr_pton(AF_INET, GW_IP, &gw);  // Gateway, probably your PC/router

    net_if_ipv4_addr_add(iface, &addr, NET_ADDR_MANUAL, 0);
    net_if_ipv4_set_netmask_by_addr(iface, &addr, &netmask);
    net_if_ipv4_set_gw(iface, &gw);

    while (!net_if_is_up(iface)) {
        k_sleep(K_MSEC(100));
    }

    // Set up destination address (Python server)
    struct sockaddr_in dest_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SERVER_PORT),
    };
    zsock_inet_pton(AF_INET, PC_IP, &dest_addr.sin_addr);

    int sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        LOG_ERR("Failed to create socket");
        return;
    }

    struct sockaddr_in sock_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SERVER_PORT),
        .sin_addr.s_addr = INADDR_ANY
    };

    if (zsock_bind(sock, (struct sockaddr *)&sock_addr, sizeof(sock_addr)) < 0) {
        LOG_ERR("Bind failed");
        return;
    }

    if (zsock_connect(sock, (const struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0) {
        LOG_ERR("Connect failed");
        zsock_close(sock);
        return;
    }

    int connected = 0;
    char buf[32];

    while (!connected) {
        zsock_send(sock, "CONN?", sizeof("CONN?"), 0);

        int r = zsock_recv(sock, buf, sizeof(buf) - 1, ZSOCK_MSG_DONTWAIT);
        printk("%d %32s\n", r, buf);
        if (r > 0) {
            buf[r] = '\0';
            LOG_INF("Received: %s", buf);
            if (strncmp(buf, "OK", 2) == 0) {
                connected = 1;
            }
        }

        k_msleep(1000);
    }
}
K_THREAD_DEFINE(cam_client_tid, CAM_STACKSIZE, cam_client, 
        NULL, NULL, NULL, CAM_PRIORITY, 0, 0);

void servo_client(void) 
{
    // Set up destination address (Python server)
    struct sockaddr_in dest_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SERVER_PORT),
    };
    zsock_inet_pton(AF_INET, PC_IP, &dest_addr.sin_addr);

    int sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        LOG_ERR("Failed to create socket");
        return;
    }

    struct sockaddr_in sock_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SERVER_PORT),
        .sin_addr.s_addr = INADDR_ANY
    };

    if (zsock_bind(sock, (struct sockaddr *)&sock_addr, sizeof(sock_addr)) < 0) {
        LOG_ERR("Bind failed");
        return;
    }

    if (zsock_connect(sock, (const struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0) {
        LOG_ERR("Connect failed");
        zsock_close(sock);
        return;
    }

    int connected = 0;
    char buf[32];

    while (!connected) {
        zsock_send(sock, "CONN?", sizeof("CONN?"), 0);

        int r = zsock_recv(sock, buf, sizeof(buf) - 1, ZSOCK_MSG_DONTWAIT);
        printk("%d %32s\n", r, buf);
        if (r > 0) {
            buf[r] = '\0';
            LOG_INF("Received: %s", buf);
            if (strncmp(buf, "OK", 2) == 0) {
                connected = 1;
            }
        }

        k_msleep(1000);
    }
}
K_THREAD_DEFINE(servo_client_tid, SERVO_STACKSIZE, servo_client, 
        NULL, NULL, NULL, SERVO_PRIORITY, 0, 0);
