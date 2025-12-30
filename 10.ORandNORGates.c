#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#define ORNOR GPIO_NUM_13

void app_main(void)
{
    gpio_reset_pin(ORNOR);
    gpio_set_direction(ORNOR,GPIO_MODE_OUTPUT);
    gpio_set_level(ORNOR,1);
    /*
    Connections:
    
    OR Gate:
    Transistor1:
    Emitter - GND
    Base - Pushbutton1 - Resistor(330 Ohm) - D13
    Collector - Transistor2(Collector)
    Transistor2:
    Emitter - GND
    Base - Pushbutton2 - Resistor(330 Ohm) - D13
    Collector - Transistor1(Collector)
    Collector - LED(-),LED(+) - Resistor (330 Ohm) - D13

    NOR Gate:
    Transistor1:
    Emitter - GND
    Base - Pushbutton1 - Resistor(330 Ohm) - D13
    Collector - Transistor2(Collector)
    Transistor2:
    Emitter - GND
    Base - Pushbutton2 - Resistor(330 Ohm) - D13
    Collector - Transistor1(Collector)
    Collector - LED(+),LED(-) - GND
    Collector - Resistor (330 Ohm) - D13
    */
}