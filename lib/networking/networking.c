
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/wifi.h>

#include <common/my_network.h> // For wifi credentials

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(networking, LOG_LEVEL_INF);

static struct net_mgmt_event_callback wifi_cb;
static bool wifi_connected = false;
static void wifi_connect(void);

int init_ip_net(const char *my_addr) 
{
    struct net_if *iface = net_if_get_default();

    struct in_addr addr, netmask, gw;

    net_addr_pton(AF_INET, my_addr, &addr);
    net_addr_pton(AF_INET, "255.255.255.0", &netmask);
    net_addr_pton(AF_INET, GW_IP, &gw);  // Gateway, probably your PC/router

    net_if_ipv4_addr_add(iface, &addr, NET_ADDR_MANUAL, 0);
    net_if_ipv4_set_netmask_by_addr(iface, &addr, &netmask);
    net_if_ipv4_set_gw(iface, &gw);

    while (!net_if_is_up(iface)) {
        k_sleep(K_MSEC(100));
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

void init_wifi()
{
    wifi_connect();
    while (!wifi_connected) {
        k_sleep(K_SECONDS(1));
    }
}

int start_udp(int port, const char *dest_ip)
{
    int sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        LOG_ERR("Failed to create socket");
        return -1;
    }

    struct sockaddr_in sock_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(port),
        .sin_addr.s_addr = INADDR_ANY
    };
    if (zsock_bind(sock, (struct sockaddr *)&sock_addr, sizeof(sock_addr)) < 0) {
        LOG_ERR("Bind failed");
        return -1;
    }

    // Set up destination address (Python server)
    struct sockaddr_in dest_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(port),
    };
    zsock_inet_pton(AF_INET, dest_ip, &dest_addr.sin_addr);
    if (zsock_connect(sock, (const struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0) {
        LOG_ERR("Connect failed");
        zsock_close(sock);
        return -1;
    }

    return sock;
}
