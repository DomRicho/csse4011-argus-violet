#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

/* Register logging module */
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Define the LED pin */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

int main(void)
{
    const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    int counter = 0;
    char buffer[50];
    
    /* Give serial port time to stabilize */
    k_sleep(K_MSEC(1000));
    
    /* Print startup message using all three methods */
    printk("\n\n*** METHOD 2: PRINTK TEST ***\n\n");
    LOG_INF("*** METHOD 3: LOG_INF TEST ***");
    
    /* Main application loop testing all three output methods */
    while (1) {
        
        /* Method 2: printk */
        printk("[Method 2] Loop count: %d\r\n", counter);
        
        /* Method 3: LOG */
        LOG_INF("[Method 3] Loop count: %d", counter);
        
        counter++;
        gpio_pin_toggle_dt(&led);
        k_sleep(K_MSEC(1000));  /* Slow down to 1 second for easier reading */
    }
    
    return 0;
}