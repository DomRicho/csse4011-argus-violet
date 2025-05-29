#include <string.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
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

static bool ip_done = false;

K_FIFO_DEFINE(servo_cmd_fifo);
K_SEM_DEFINE(tx_done, 0, 1);

const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));

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

static uint8_t frame_seg[528];
static size_t frame_size = 528;
static size_t data_sent = 0;
uint8_t cmd_buf[8];

static void uart_cb(const struct device *dev, void *user_data)
{
    /* Ensure interrupt is valid */
    if (!uart_irq_update(dev)) return;

    /* Handle RX */
    if (uart_irq_rx_ready(dev)) {
        int read = uart_fifo_read(dev, cmd_buf, 2);

        if (read > 0) {
            struct servo_cmd_t *servo_cmd = k_malloc(sizeof(struct servo_cmd_t));
            memcpy(servo_cmd->cmd, cmd_buf, read); 
            k_fifo_put(&servo_cmd_fifo, servo_cmd);
        }
    }

    /* Handle TX */
    if (uart_irq_tx_ready(dev)) {
        if (data_sent < frame_size) {
            int written = uart_fifo_fill(dev, frame_seg + data_sent, frame_size - data_sent);
            data_sent += written;
        }

        // If all data sent, disable TX IRQ
        if (data_sent >= frame_size) {
            uart_irq_tx_disable(dev);
            k_sem_give(&tx_done);
            data_sent = 0;
        }
    }
}


void cam_client(void) 
{
    init_wifi();
    init_ip_net(BRIDGE_IP);
    /*uart_callback_set(uart_dev, uart_callback, NULL);*/
    if (!device_is_ready(uart_dev)) {
        return;
    }
    uart_irq_callback_set(uart_dev, uart_cb);
    uart_irq_rx_enable(uart_dev);
    ip_done = true;
    int sock = connect_tcp_client(CAM_IP, CAM_SERVER_PORT);
    if (sock < 0) {
        LOG_ERR("Error with socket");
        return;
    }


    uint8_t frame_a[512];
    memset(frame_a, 0xAA, sizeof(frame_a));

    int id, chunk; 

    uint8_t header_f[17];
    chunk = 0;
    id = 0;
    uint8_t colours[3] = { 0xAA, 0xFF, 0x00};

    while (1) {
        while (chunk < 225) {
            snprintf(header_f, 17, "F%012d%03d", id, chunk);
            memcpy(frame_seg, header_f, 16);
            memcpy(frame_seg + 16, frame_a, 512);
            // Enable TX IRQ to send it
            uart_irq_tx_enable(uart_dev);
            k_sem_take(&tx_done, K_FOREVER);
            chunk++;
        }
        memset(frame_a, colours[id % 3], sizeof(frame_a));
        id++;
        chunk = 0;
    }
}
K_THREAD_DEFINE(cam_client_tid, CAM_STACKSIZE, cam_client, 
        NULL, NULL, NULL, CAM_PRIORITY, 0, 0);

void servo_client(void) 
{
    while (ip_done == false) k_msleep(100);
    int sock = start_udp(SERVO_SERVER_PORT, SERVO_IP);
    if (sock < 0) {
        LOG_ERR("Error with socket");
        return;
    }

    struct servo_cmd_t *servo_cmd; 
    int ret = 0;

    while (1) {
        servo_cmd = k_fifo_get(&servo_cmd_fifo, K_FOREVER);
        LOG_INF("%s", servo_cmd->cmd);
        int mag = servo_cmd->cmd[1] - '0';
        switch(servo_cmd->cmd[0]) {
        case 'U':
            snprintf(servo_cmd->cmd, 8, "up:%1d", mag);
            break;
        case 'D':
            snprintf(servo_cmd->cmd, 8, "down:%1d", mag);
            break;
        case 'L':
            snprintf(servo_cmd->cmd, 8, "left:%1d", mag);
            break;
        case 'R':
            snprintf(servo_cmd->cmd, 8, "right:%1d", mag);
            break;
        default:
            k_free(servo_cmd);
            continue;
        }
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

/*--------------------------------------------------------------------------*/
/*----------------------------- SHELL COMMANDS -----------------------------*/
/*--------------------------------------------------------------------------*/

/*static int servo_move(const struct shell *shell, size_t argc, char **argv)*/
/*{*/
/*    if (argc != 3) {*/
/*        shell_print(shell, "Usage: move <direction_deg> <magnitude>");*/
/*        return -EINVAL;*/
/*    }*/
/**/
/*    char* direction = argv[1];*/
/*    int magnitude = atoi(argv[2]);*/
/*    struct servo_cmd_t *servo_cmd = k_malloc(sizeof(struct servo_cmd_t));*/
/*    if (magnitude < 1 || magnitude > 3) {*/
/*        shell_error(shell, "Magnitude must be 1 2 3");*/
/*        return -EINVAL;*/
/*    }*/
/*    // Simple validation*/
/*    if (strncmp(direction, "left", 4) == 0) {*/
/*        snprintf(servo_cmd->cmd, 8, "left:%d", magnitude);*/
/*    } else if (strncmp(direction, "right", 5) == 0) {*/
/*        snprintf(servo_cmd->cmd, 8, "right:%d", magnitude);*/
/*    } else if (strncmp(direction, "down", 4) == 0) {*/
/*        snprintf(servo_cmd->cmd, 8, "down:%d", magnitude);*/
/*    } else if (strncmp(direction, "up", 2) == 0) {*/
/*        snprintf(servo_cmd->cmd, 8, "up:%d", magnitude);*/
/*    } else {*/
/*        shell_error(shell, "wrong direction: up down left right");*/
/*        return -EINVAL;*/
/*    }*/
/**/
/*    k_fifo_put(&servo_cmd_fifo, servo_cmd);*/
/**/
/*    shell_print(shell, "Received direction: %s, magnitude: %d", direction, magnitude);*/
/**/
/*    return 0;*/
/*}*/
/**/
/*// Subcommand set*/
/*SHELL_STATIC_SUBCMD_SET_CREATE(sub_servo,*/
/*    SHELL_CMD(move, NULL, "Move with <left | right | up | down> < 1 2 3 >", servo_move),*/
/*    SHELL_SUBCMD_SET_END*/
/*);*/
/**/
/*// Root command*/
/*SHELL_CMD_REGISTER(servo, &sub_servo, "servo commands", NULL);*/
