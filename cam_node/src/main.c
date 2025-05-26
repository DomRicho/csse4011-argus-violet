#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/drivers/display.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/video-controls.h>

#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/socket.h>

#include <zephyr/logging/log.h>

#include <esp_clk.h>

#include "cam_node/cam_node.h"
#include "common/my_network.h" // For wifi credentials
#include "zephyr/net/wifi.h"

#define FRAME_HEIGHT 240
#define FRAME_WIDTH 240
#define PIXEL_FORMAT VIDEO_PIX_FMT_RGB565
#define FRAME_BUF_SIZE (FRAME_WIDTH * FRAME_HEIGHT * 2)

#define LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(cam_node, LOG_LEVEL);

#define CAMERA_STACK_SIZE 4096
#define NETWORK_STACK_SIZE 4096 * 3
#define CAMERA_PRIORITY 3
#define NETWORK_PRIORITY 6 
K_THREAD_DEFINE(camera_tid, CAMERA_STACK_SIZE, camera_thread, NULL, NULL, NULL, CAMERA_PRIORITY, 0, 0);
K_THREAD_DEFINE(network_tid, NETWORK_STACK_SIZE, network_thread, NULL, NULL, NULL, NETWORK_PRIORITY, 0, 0);

struct k_sem sem_frame_ready, sem_frame_taken;
K_SEM_DEFINE(sem_frame_ready, 0, 1); // Camera gives, network takes
K_SEM_DEFINE(sem_frame_taken, 1, 1); // Network gives, camera takes

static struct net_mgmt_event_callback wifi_cb;
static bool wifi_connected = false;

uint8_t frame_seg[528];
int chunk = 0;

// Store the frame buffer in external RAM
__attribute__((section(".ext_ram.bss")))
static uint8_t frame_buf[FRAME_BUF_SIZE];

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

static ssize_t send_frame_with_header(int sock, const void *buf, size_t len)
{
    // Send magic header
    const char header[4] = {'F', 'R', 'A', 'M'};
    if (sendall(sock, header, 4) < 0) {
        LOG_ERR("Error sending FRAM");
        return -2;
    }

    // Send frame length (little-endian uint32)
    uint32_t frame_len = (uint32_t)len;
    if (sendall(sock, &frame_len, 4) < 0) {
        LOG_ERR("Error sending FRAM");
        return -3;
    }

    // fragment video frame into 225 segments (115200/512) with 16 ID header 
    uint8_t header_f[17];
    int count = 0;
    while (count < 225) {
        snprintf(header_f, 17, "F%012d%03d", chunk, count);
        memcpy(frame_seg, header_f, 16);
        memcpy(frame_seg + 16, buf, 512);
        buf = (const char *)buf + 512;
        if (sendall(sock, frame_seg, 528) < 0) {
            LOG_ERR("Error %d: Error sending frame %d", errno, count);
            return -4;
        }
        count++;
        k_usleep(2000);
    }
    LOG_INF("Frame %d sent", chunk);
    chunk++;

    // Send frame data
    /*return sendall(sock, buf, len);*/
    return 0;
}

static inline int display_setup(const struct device *const display_dev, const uint32_t pixfmt)
{
	struct display_capabilities capabilities;
	int ret = 0;

	LOG_INF("Display device: %s", display_dev->name);

	display_get_capabilities(display_dev, &capabilities);

	/* Set display pixel format to match the one in use by the camera */
	switch (pixfmt) {
	case VIDEO_PIX_FMT_RGB565:
		if (capabilities.current_pixel_format != PIXEL_FORMAT_BGR_565) {
			ret = display_set_pixel_format(display_dev, PIXEL_FORMAT_BGR_565);
		}
		break;
	case VIDEO_PIX_FMT_XRGB32:
		if (capabilities.current_pixel_format != PIXEL_FORMAT_ARGB_8888) {
			ret = display_set_pixel_format(display_dev, PIXEL_FORMAT_ARGB_8888);
		}
		break;
	default:
		return -ENOTSUP;
	}

	if (ret) {
		LOG_ERR("Unable to set display format");
		return ret;
	}

	/* Turn off blanking if driver supports it */
	ret = display_blanking_off(display_dev);
	if (ret == -ENOSYS) {
		LOG_DBG("Display blanking off not available");
		ret = 0;
	}

	return ret;
}

static inline void video_display_frame(const struct device *const display_dev,
    const struct video_buffer *const vbuf,
    const struct video_format fmt)
{
    struct display_buffer_descriptor buf_desc = {
        .buf_size = vbuf->bytesused,
        .width = fmt.width,
        .pitch = buf_desc.width,
        .height = vbuf->bytesused / fmt.pitch,
    };

    display_write(display_dev, 0, vbuf->line_offset, &buf_desc, vbuf->buffer);
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

void camera_thread(void) {
    struct video_buffer *buffers[CONFIG_VIDEO_BUFFER_POOL_NUM_MAX], *vbuf;
	struct video_format fmt;
	struct video_caps caps;
	enum video_buf_type type = VIDEO_BUF_TYPE_OUTPUT;
	size_t bsize;
	int i = 0;
    int err;
    /* is indeed running at 240 MHz */
    /*int cpu_freq = esp_clk_cpu_freq() / 1000000;*/
    /*printk("%d Mhz\n", cpu_freq);*/

    // Initialize video device
    const struct device *const video_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_camera));
	if (!device_is_ready(video_dev)) {
		LOG_ERR("%s: video device is not ready", video_dev->name);
		return;
	}
    LOG_INF("Video device: %s", video_dev->name);

    caps.type = type;
	if (video_get_caps(video_dev, &caps)) {
		LOG_ERR("Unable to retrieve video capabilities");
		return;
	}
	fmt.type = type;
	if (video_get_format(video_dev, &fmt)) {
		LOG_ERR("Unable to retrieve video format");
		return;
	}
	fmt.height = FRAME_HEIGHT;
	fmt.width = FRAME_WIDTH;
	fmt.pitch = fmt.width * 2;
    fmt.pixelformat = PIXEL_FORMAT;

    if (video_set_format(video_dev, &fmt)) {
        LOG_ERR("Unable to set format");
        return;
    }

	struct video_control ctrl = {.id = VIDEO_CID_VFLIP, .val = 1};
	if (video_set_ctrl(video_dev, &ctrl)) {
        LOG_ERR("Unable to set control");
        return;
    }

    // Initialize display device
    const struct device *const display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

	if (!device_is_ready(display_dev)) {
		LOG_ERR("%s: display device not ready.", display_dev->name);
		return;
	}

	err = display_setup(display_dev, fmt.pixelformat);
	if (err) {
		LOG_ERR("Unable to set up display");
		return;
	}

    LOG_INF("Camera format: width=%d height=%d pitch=%d pixelformat=0x%x",
        fmt.width, fmt.height, fmt.pitch, fmt.pixelformat);

    /* Alloc video buffers and enqueue for capture */
    bsize = fmt.pitch * fmt.height;
	for (i = 0; i < ARRAY_SIZE(buffers); i++) {
		/*
		* For some hardwares, such as the PxP used on i.MX RT1170 to do image rotation,
		* buffer alignment is needed in order to achieve the best performance
		*/
		buffers[i] = video_buffer_aligned_alloc(bsize, CONFIG_VIDEO_BUFFER_POOL_ALIGN,
							K_FOREVER);
		if (buffers[i] == NULL) {
			LOG_ERR("Unable to alloc video buffer");
			return;
		}
		buffers[i]->type = type;
		video_enqueue(video_dev, buffers[i]);
	}

    /* Start video capture */
	if (video_stream_start(video_dev, type)) {
		LOG_ERR("Unable to start capture (interface)");
		return;
	}

	LOG_INF("Capture started");

    while (1) {
        if (video_dequeue(video_dev, &vbuf, K_FOREVER) == 0) {
            // Display frame on the display device
            video_display_frame(display_dev, vbuf, fmt);

            // Only update buffer if network is done with it
            if (k_sem_take(&sem_frame_taken, K_NO_WAIT) == 0) {
                if (vbuf->bytesused != FRAME_BUF_SIZE) {
                    LOG_ERR("Frame size mismatch: %zu", vbuf->bytesused);
                }
                // LOG_DBG("Addr: %p, Size: %zu, Bytesused: %zu", (void*)vbuf->buffer, vbuf->size, vbuf->bytesused);
                memcpy(frame_buf, vbuf->buffer, FRAME_BUF_SIZE);
                k_sem_give(&sem_frame_ready); // Signal to network thread
            }
        }
        video_enqueue(video_dev, vbuf);
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

    LOG_INF("Client connected!");
    while (1) {
        // LOG_DBG("Waiting for new frame...");
        k_sem_take(&sem_frame_ready, K_FOREVER); 
        // LOG_DBG("Sending frame to client...");
        
        int err = send_frame_with_header(sock, frame_buf, FRAME_BUF_SIZE);


        k_sem_give(&sem_frame_taken); // Allow camera to update buffer again
        if (err < 0) {
            LOG_ERR("Error %d: Failed to send frame to client", err);
            break;
        }
    }
    /*zsock_close(client_sock);*/
    zsock_close(sock);
    while(1) k_msleep(2000);
    return;
}
