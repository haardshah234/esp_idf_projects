#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#include "esp_log.h"
#define BUTTON GPIO_NUM_12

int delay(int milliseconds) {
    return milliseconds/portTICK_PERIOD_MS;
}

void app_main(void)
{
    gpio_reset_pin(BUTTON);
    gpio_set_direction(BUTTON, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON, GPIO_PULLUP_ONLY);
    int val;
    while(1)
    {
        val = gpio_get_level(BUTTON);
        ESP_LOGI("ESP32", "%d", val);
        vTaskDelay(delay(50));
    }
}
