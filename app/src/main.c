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

#include "app/my_network.h" // For wifi credentials

#define LOG_LEVEL LOG_LEVEL_DBG

#define FRAME_HEIGHT 240
#define FRAME_WIDTH 240
#define PIXEL_FORMAT VIDEO_PIX_FMT_RGB565

// #define FRAME_HEIGHT 240
// #define FRAME_WIDTH 320
// #define FRAME_WIDTH 1600
// #define FRAME_HEIGHT 1200
// #define PIXEL_FORMAT VIDEO_PIX_FMT_JPEG 

LOG_MODULE_REGISTER(main);

static struct net_mgmt_event_callback wifi_cb;
static bool wifi_connected = false;

static ssize_t sendall(int sock, const void *buf, size_t len)
{
    while (len) {
        ssize_t out_len = zsock_send(sock, buf, len, 0);
        if (out_len < 0) {
            return out_len;
        }
        buf = (const char *)buf + out_len;
        len -= out_len;
    }
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
        LOG_WRN("Wi-Fi disconnected");
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

int main(void)
{
    struct video_buffer *buffers[CONFIG_VIDEO_BUFFER_POOL_NUM_MAX], *vbuf;
	struct video_format fmt;
	struct video_caps caps;
	struct video_frmival frmival;
	struct video_frmival_enum fie;
	enum video_buf_type type = VIDEO_BUF_TYPE_OUTPUT;
	unsigned int frame = 0;
	size_t bsize;
	int i = 0;
	int err;

    // Wait for 3 seconds to allow time to connect to screen
    // k_sleep(K_SECONDS(3));

    // Connect to Wi-Fi
    wifi_connect();
    while (!wifi_connected) {
        k_sleep(K_SECONDS(1));
    }

    struct net_if *iface = net_if_get_default();
    while (!net_if_is_up(iface)) {
        k_sleep(K_MSEC(100));
    }

    // Open TCP socket
    int sock = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        LOG_ERR("Failed to create socket");
        return 0;
    }

    // Create socket
    int listen_sock = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_sock < 0) {
        LOG_ERR("Failed to create listening socket");
        return 0;
    }

    // Set socket options
    struct sockaddr_in bind_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SERVER_PORT),
        .sin_addr.s_addr = INADDR_ANY
    };

    // Bind the socket to the address and port
    if (zsock_bind(listen_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        LOG_ERR("Bind failed");
        zsock_close(listen_sock);
        return 0;
    }

    // Set the socket to listen for incoming connections
    if (zsock_listen(listen_sock, 1) < 0) {
        LOG_ERR("Listen failed");
        zsock_close(listen_sock);
        return 0;
    }
    LOG_INF("Server listening on port %d", SERVER_PORT);

    // Wait to accept connection from client
    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    int client_sock = zsock_accept(listen_sock, (struct sockaddr *)&client_addr, &client_addr_len);
    if (client_sock < 0) {
        LOG_ERR("Accept failed");
        zsock_close(listen_sock);
        return 0;
    }
    LOG_INF("Client connected!");

    // Initialize video device
    const struct device *const video_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_camera));
	if (!device_is_ready(video_dev)) {
		LOG_ERR("%s: video device is not ready", video_dev->name);
		return 0;
	}
    LOG_INF("Video device: %s", video_dev->name);

    caps.type = type;
	if (video_get_caps(video_dev, &caps)) {
		LOG_ERR("Unable to retrieve video capabilities");
		return 0;
	}
	fmt.type = type;
	if (video_get_format(video_dev, &fmt)) {
		LOG_ERR("Unable to retrieve video format");
		return 0;
	}
	fmt.height = FRAME_HEIGHT;
	fmt.width = FRAME_WIDTH;
	fmt.pitch = fmt.width * 2;
    fmt.pixelformat = PIXEL_FORMAT;

    if (video_set_format(video_dev, &fmt)) {
        LOG_ERR("Unable to set format");
        return 0;
    }
    LOG_INF("Camera format: width=%d height=%d pitch=%d pixelformat=0x%x",
        fmt.width, fmt.height, fmt.pitch, fmt.pixelformat);

    if (!video_get_frmival(video_dev, &frmival)) {
        LOG_INF("- Default frame rate : %f fps",
            1.0 * frmival.denominator / frmival.numerator);
    }

	struct video_control ctrl = {.id = VIDEO_CID_VFLIP, .val = 1};

	if (video_set_ctrl(video_dev, &ctrl)) {
        LOG_ERR("Unable to set control");
        return 0;
    }

    //// Initialize display device
    // const struct device *const display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

	// if (!device_is_ready(display_dev)) {
	// 	LOG_ERR("%s: display device not ready.", display_dev->name);
	// 	return 0;
	// }

	// err = display_setup(display_dev, fmt.pixelformat);
	// if (err) {
	// 	LOG_ERR("Unable to set up display");
	// 	return err;
	// }

    /* Size to allocate for each buffer */
	if (caps.min_line_count == LINE_COUNT_HEIGHT) {
		bsize = fmt.pitch * fmt.height;
	} else {
		bsize = fmt.pitch * caps.min_line_count;
	}
    // bsize = 40 * 1024; // 40KB is usually enough for 320x240 JPEG, increase if needed

    /* Alloc video buffers and enqueue for capture */
	for (i = 0; i < ARRAY_SIZE(buffers); i++) {
		/*
		* For some hardwares, such as the PxP used on i.MX RT1170 to do image rotation,
		* buffer alignment is needed in order to achieve the best performance
		*/
		buffers[i] = video_buffer_aligned_alloc(bsize, CONFIG_VIDEO_BUFFER_POOL_ALIGN,
							K_FOREVER);
		if (buffers[i] == NULL) {
			LOG_ERR("Unable to alloc video buffer");
			return 0;
		}
		buffers[i]->type = type;
		video_enqueue(video_dev, buffers[i]);
	}

    /* Start video capture */
	if (video_stream_start(video_dev, type)) {
		LOG_ERR("Unable to start capture (interface)");
		return 0;
	}

	LOG_INF("Capture started");

    while (1) {
		err = video_dequeue(video_dev, &vbuf, K_FOREVER);
		if (err) {
			LOG_ERR("Unable to dequeue video buf");
			return 0;
		}

        // LOG_INF("Sending frame: %d bytes, line_offset: %d", vbuf->bytesused, vbuf->line_offset);
        // Send the image data
        err = sendall(client_sock, vbuf->buffer, vbuf->bytesused);
        if (err < 0) {
            LOG_ERR("Failed to send frame to client");
            break;
        }

        // Display the frame on the screen
        // video_display_frame(display_dev, vbuf, fmt);

        err = video_enqueue(video_dev, vbuf);
		if (err) {
			LOG_ERR("Unable to requeue video buf");
			return 0;
		}
	}
    zsock_close(client_sock);
    zsock_close(listen_sock);
    return 0;
}
