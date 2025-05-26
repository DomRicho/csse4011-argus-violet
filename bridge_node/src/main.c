#include <zephyr/device.h>
#include <zephyr/kernel.h>

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

#define CAM_STACKSIZE 2048
#define CAM_PRIORITY 5
#define SERVO_STACKSIZE 2048
#define SERVO_PRIORITY 5

#define LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(bridge_node, LOG_LEVEL);

static struct net_mgmt_event_callback wifi_cb;
static bool wifi_connected = false;
static bool ip_done = false;

K_FIFO_DEFINE(servo_cmd_fifo);

enum direction {
    UP,
    DOWN,
    LEFT,
    RIGHT,
};

struct servo_cmd_t {
    void *fifo_res;
    char cmd[8]; 
};

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

void cam_client(void) 
{
    wifi_connect();
    while (!wifi_connected) {
        k_sleep(K_SECONDS(1));
    }
    struct net_if *iface = net_if_get_default();

    struct in_addr addr, netmask, gw;

    net_addr_pton(AF_INET, BRIDGE_IP, &addr);
    net_addr_pton(AF_INET, "255.255.255.0", &netmask);
    net_addr_pton(AF_INET, GW_IP, &gw);  // Gateway, probably your PC/router

    net_if_ipv4_addr_add(iface, &addr, NET_ADDR_MANUAL, 0);
    net_if_ipv4_set_netmask_by_addr(iface, &addr, &netmask);
    net_if_ipv4_set_gw(iface, &gw);

    while (!net_if_is_up(iface)) {
        k_sleep(K_MSEC(100));
    }
    ip_done = true;

    int sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        LOG_ERR("Failed to create socket");
        return;
    }

    struct sockaddr_in sock_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(CAM_SERVER_PORT),
        .sin_addr.s_addr = INADDR_ANY
    };
    if (zsock_bind(sock, (struct sockaddr *)&sock_addr, sizeof(sock_addr)) < 0) {
        LOG_ERR("Bind failed");
        return;
    }

    // Set up destination address (Python server)
    struct sockaddr_in dest_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(CAM_SERVER_PORT),
    };
    zsock_inet_pton(AF_INET, CAM_IP, &dest_addr.sin_addr);
    if (zsock_connect(sock, (const struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0) {
        LOG_ERR("Connect failed");
        zsock_close(sock);
        return;
    }

    while (1) {
        zsock_send(sock, "Hello world!", strlen("Hello world!"), 0); 
        k_msleep(1000); 
    }
}
K_THREAD_DEFINE(cam_client_tid, CAM_STACKSIZE, cam_client, 
        NULL, NULL, NULL, CAM_PRIORITY, 0, 0);

void servo_client(void) 
{
    while (ip_done == false) k_msleep(100);
    LOG_INF("servo client init");
    // Set up destination address (Python server)

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

    struct sockaddr_in dest_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SERVO_SERVER_PORT),
    };
    zsock_inet_pton(AF_INET, SERVO_IP, &dest_addr.sin_addr);
    if (zsock_connect(sock, (const struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0) {
        LOG_ERR("Connect failed");
        zsock_close(sock);
        return;
    }

    struct servo_cmd_t *servo_cmd; 
    int ret = 0;
    while (true) {
        servo_cmd = k_fifo_get(&servo_cmd_fifo, K_FOREVER);
        LOG_INF("%s", servo_cmd->cmd);
        ret = zsock_send(sock, servo_cmd->cmd, strlen(servo_cmd->cmd), 0); 
        if (ret < 0) {
            LOG_ERR("Error %d: :(", errno);
        }
        k_free(servo_cmd);
    }
    zsock_close(sock);
}
K_THREAD_DEFINE(servo_client_tid, SERVO_STACKSIZE, servo_client, 
        NULL, NULL, NULL, SERVO_PRIORITY, 0, 0);

// SHELL COMMANDS
//
static int servo_move(const struct shell *shell, size_t argc, char **argv)
{
    if (argc != 3) {
        shell_print(shell, "Usage: move <direction_deg> <magnitude>");
        return -EINVAL;
    }

    char* direction = argv[1];
    int magnitude = atoi(argv[2]);
    struct servo_cmd_t *servo_cmd = k_malloc(sizeof(struct servo_cmd_t));
    if (magnitude < 1 || magnitude > 3) {
        shell_error(shell, "Magnitude must be 1 2 3");
        return -EINVAL;
    }
    // Simple validation
    if (strncmp(direction, "left", 4) == 0) {
        snprintf(servo_cmd->cmd, 8, "left:%d", magnitude);
    } else if (strncmp(direction, "right", 5) == 0) {
        snprintf(servo_cmd->cmd, 8, "right:%d", magnitude);
    } else if (strncmp(direction, "down", 4) == 0) {
        snprintf(servo_cmd->cmd, 8, "down:%d", magnitude);
    } else if (strncmp(direction, "up", 2) == 0) {
        snprintf(servo_cmd->cmd, 8, "up:%d", magnitude);
    } else {
        shell_error(shell, "wrong direction: up down left right");
        return -EINVAL;
    }

    k_fifo_put(&servo_cmd_fifo, servo_cmd);

    shell_print(shell, "Received direction: %s, magnitude: %d", direction, magnitude);

    return 0;
}

// Subcommand set
SHELL_STATIC_SUBCMD_SET_CREATE(sub_servo,
    SHELL_CMD(move, NULL, "Move with <left | right | up | down> < 1 2 3 >", servo_move),
    SHELL_SUBCMD_SET_END
);

// Root command
SHELL_CMD_REGISTER(servo, &sub_servo, "servo commands", NULL);
