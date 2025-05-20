#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/drivers/display.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/video-controls.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

#define LOG_LEVEL LOG_LEVEL_DBG

#define FRAME_HEIGHT 240
#define FRAME_WIDTH 240
#define PIXEL_FORMAT VIDEO_PIX_FMT_RGB565

// #define FRAME_WIDTH 1600
// #define FRAME_HEIGHT 1200
// #define PIXEL_FORMAT VIDEO_PIX_FMT_JPEG 

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

int main(void)
{
    // Wait for 10 seconds to allow time to connect to screen
    k_sleep(K_SECONDS(5));

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
    
    /* Get default/native format */
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

    if (!video_get_frmival(video_dev, &frmival)) {
        LOG_INF("- Default frame rate : %f fps",
            1.0 * frmival.denominator / frmival.numerator);
    }

    /* Set controls */
	struct video_control ctrl = {.id = VIDEO_CID_VFLIP, .val = 1};

	if (video_set_ctrl(video_dev, &ctrl)) {
        LOG_ERR("Unable to set control");
        return 0;
    }

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

		LOG_DBG("Got frame %u! size: %u; timestamp %u ms", frame++, vbuf->bytesused,
			vbuf->timestamp);

        // video_display_frame(display_dev, vbuf, fmt);
        err = video_enqueue(video_dev, vbuf);
		if (err) {
			LOG_ERR("Unable to requeue video buf");
			return 0;
		}
	}
}
