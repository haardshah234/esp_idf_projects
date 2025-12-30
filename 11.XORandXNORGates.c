#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#define XORXNOR GPIO_NUM_13

void app_main(void)
{
    gpio_reset_pin(XORXNOR);
    gpio_set_direction(XORXNOR,GPIO_MODE_OUTPUT);
    gpio_set_level(XORXNOR,1);
    /*
    Connections:
    
    XOR Gate:

    Transistor1: 
    Emitter - GND 
    Base - Button1 
    Collector - Transistor2(Emitter) 
    ---------------
    Transistor2:
    Emitter - Transistor1(Collector)
    Base - Button2
    Collector - 330Ohm - D13
    Collector - 330Ohm - Transistor3(Base)
    ---------------
    Transistor3:
    Emitter - GND
    Base - 330Ohm - T2(Collector)
    Collector - 10kOhm - GND
    Collector - Transistor4(Emitter) - Transistor5(Emitter)
    ---------------
    Transistor4:
    Emitter - Transistor3(Collector)
    Emitter - Transistor5(Emitter)
    Base - Button1
    Collector - Transistor5(Collector)
    ---------------
    Transistor5:
    Emitter - T4(Emitter) - T3(Collector)
    Base - Button2
    Collector - Transistor4(Collector)
    Collector - Led(-), Led(+) - 330Ohm - D13

    XNOR Gate:

    Transistor1: 
    Emitter - GND 
    Base - Button1 
    Collector - Transistor2(Emitter) 
    ---------------
    Transistor2:
    Emitter - Transistor1(Collector)
    Base - Button2
    Collector - 330Ohm - D13
    Collector - 330Ohm - Transistor3(Base)
    ---------------
    Transistor3:
    Emitter - GND
    Base - 330Ohm - T2(Collector)
    Collector - 10kOhm - GND
    Collector - Transistor4(Emitter) - Transistor5(Emitter)
    ---------------
    Transistor4:
    Emitter - Transistor3(Collector)
    Emitter - Transistor5(Emitter)
    Base - Button1
    Collector - Transistor5(Collector)
    ---------------
    Transistor5:
    Emitter - T4(Emitter) - T3(Collector)
    Base - Button2
    Collector - Transistor4(Collector)
    Collector - Led(+), Led(-) - GND
    Collector - 330Ohm - D13
    */
}