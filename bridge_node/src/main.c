#include <zephyr/device.h>
#include <zephyr/kernel.h>

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

void cam_client(void) 
{
    init_wifi();
    init_ip_net(BRIDGE_IP);
    ip_done = true;
    int sock = start_udp(CAM_SERVER_PORT, CAM_IP);
    if (sock < 0) {
        LOG_ERR("Error with socket");
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

// ----------------------------------------------------------------------------
// ---------------------------------------------------------- SHELL COMMANDS --
// ----------------------------------------------------------------------------

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
