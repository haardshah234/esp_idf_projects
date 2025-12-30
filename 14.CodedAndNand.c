#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#include "esp_log.h"
#define LED GPIO_NUM_15
#define BUTTON1 GPIO_NUM_13
#define BUTTON2 GPIO_NUM_25

int delay(int milliseconds) {
    return milliseconds/portTICK_PERIOD_MS;
}

void app_main(void)
{
    gpio_reset_pin(LED);
    gpio_reset_pin(BUTTON1);
    gpio_reset_pin(BUTTON2);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(BUTTON1, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON1, GPIO_PULLDOWN_ONLY);
    gpio_set_direction(BUTTON2, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON2, GPIO_PULLDOWN_ONLY);
    bool val1,val2;
    while(1)
    {
        val1 = gpio_get_level(BUTTON1);
        val2 = gpio_get_level(BUTTON2);
        gpio_set_level(LED, !(val1 && val2)); 
        //For NAND gate, the 2nd arg will be !(val1 && val2)
        vTaskDelay(delay(10));
    }
}
