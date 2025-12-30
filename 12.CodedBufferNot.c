#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#include "esp_log.h"
#define LED GPIO_NUM_15
#define BUTTON GPIO_NUM_13

int delay(int milliseconds) {
    return milliseconds/portTICK_PERIOD_MS;
}
    
void app_main(void)
{
    gpio_reset_pin(LED);
    gpio_reset_pin(BUTTON);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(BUTTON, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON, GPIO_PULLDOWN_ONLY);
    bool val;
    while(1)
    {
        val = gpio_get_level(BUTTON);
        gpio_set_level(LED,val);
        //For NOT gate, the 2nd arg will be !val1
        vTaskDelay(delay(10));
    }
}
