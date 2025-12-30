#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#define ANDNAND GPIO_NUM_13

void app_main(void)
{
    gpio_reset_pin(ANDNAND);
    gpio_set_direction(ANDNAND,GPIO_MODE_OUTPUT);
    gpio_set_level(ANDNAND,1);
    /*
    Connections:

    AND Gate:
    Transistor1:
    Emitter - GND
    Base - Pushbutton1 - Resistor(330 Ohm) - D13
    Collector - Transistor2(Emitter)
    Transistor2:
    Emitter - Transistor1(Collector)
    Base - Pushbutton2 - Resistor(330 Ohm) - D13
    Collector - LED(-),LED(+) - Resistor (330 Ohm) - D13

    NAND Gate:
    Transistor1:
    Emitter - GND
    Base - Pushbutton1 - Resistor(330 Ohm) - D13
    Collector - Transistor2(Emitter)
    Transistor2:
    Emitter - Transistor1(Collector)
    Base - Pushbutton2 - Resistor(330 Ohm) - D13
    Collector - LED(+),LED(-) - GND
    Collector - Resistor (330 Ohm) - D13
    */
}