#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#define BUFFER GPIO_NUM_13

void app_main(void)
{
    gpio_reset_pin(BUFFER);
    gpio_set_direction(BUFFER,GPIO_MODE_OUTPUT);
    gpio_set_level(BUFFER,1);
    /*
    Connections:
    Emitter - GND
    Base - Pushbutton - Resistor(330 Ohm) - D13
    Collector - LED(-), LED(1) - Resistoy(330 Ohm) - D13
    */
}