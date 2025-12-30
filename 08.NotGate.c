#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#define NOT GPIO_NUM_13

void app_main(void)
{
    gpio_reset_pin(NOT);
    gpio_set_direction(NOT,GPIO_MODE_OUTPUT);
    gpio_set_level(NOT,1);
    /*
    Connections:
    Emitter - GND
    Base - Pushbutton - Resistor(330 Ohm) - D13
    Collector - LED(+), LED(-) - GND
    Collector - Resistor (330 Ohm) - D13
    */
}