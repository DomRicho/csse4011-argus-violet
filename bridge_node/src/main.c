#include <zephyr/kernel.h>

/*
 * attempt to connect to CAM server and SERVO server. 
 *
 *
 *
 */
#define CAM_STACKSIZE 2048
#define CAM_PRIORITY 5
#define SERVO_STACKSIZE 2048
#define SERVO_PRIORITY 5


void cam_client(void) 
{
}
K_THREAD_DEFINE(cam_client_tid, CAM_STACKSIZE, cam_client, 
        NULL, NULL, NULL, CAM_PRIORITY, 0, 0);

void servo_client(void) 
{
}
K_THREAD_DEFINE(servo_client_tid, SERVO_STACKSIZE, servo_client, 
        NULL, NULL, NULL, SERVO_PRIORITY, 0, 0);
