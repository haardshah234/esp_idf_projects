#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#define LED GPIO_NUM_2

int delay(int milliseconds) {
    return milliseconds/portTICK_PERIOD_MS;
}

void app_main(void)
{
    gpio_reset_pin(LED);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    while(1)
    {
        gpio_set_level(LED, 0);
        vTaskDelay(delay(1000));
        gpio_set_level(LED, 1);
        vTaskDelay(delay(1000));
    }
}
