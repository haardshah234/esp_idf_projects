#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#define DP GPIO_NUM_13
#define A GPIO_NUM_12
#define B GPIO_NUM_14
#define C GPIO_NUM_27
#define D GPIO_NUM_26
#define E GPIO_NUM_25
#define F GPIO_NUM_33
#define G GPIO_NUM_32

int gpio[8] = {DP,A,B,C,D,E,F,G};
int h[8] = {0,0,1,1,0,1,1,1};
int e[8] = {0,1,0,0,1,1,1,1};
int l[8] = {0,0,0,0,1,1,1,0};
int o[8] = {0,1,1,1,1,1,1,0};
int dot[8] = {1,0,0,0,0,0,0,0};
int null[8] = {0,0,0,0,0,0,0,0};

int delay(int milliseconds) {
    return milliseconds/portTICK_PERIOD_MS;
}

void display(int *l)
{
    for(int i = 0; i < 8; i++)
        {
            gpio_set_level(gpio[i],!l[i]);
        }
}

void app_main(void)
{
    gpio_reset_pin(DP);
    gpio_reset_pin(A);
    gpio_reset_pin(B);
    gpio_reset_pin(C);
    gpio_reset_pin(D);
    gpio_reset_pin(E);
    gpio_reset_pin(F);
    gpio_reset_pin(G);
    gpio_set_direction(DP, GPIO_MODE_OUTPUT);
    gpio_set_direction(A, GPIO_MODE_OUTPUT);
    gpio_set_direction(B, GPIO_MODE_OUTPUT);
    gpio_set_direction(C, GPIO_MODE_OUTPUT);
    gpio_set_direction(D, GPIO_MODE_OUTPUT);
    gpio_set_direction(E, GPIO_MODE_OUTPUT);
    gpio_set_direction(F, GPIO_MODE_OUTPUT);
    gpio_set_direction(G, GPIO_MODE_OUTPUT);
    int dt = 500;
    while(1)
    {
        display(h);
        vTaskDelay(delay(dt));
        display(null);
        vTaskDelay(delay(dt));
        display(e);
        vTaskDelay(delay(dt));
        display(null);
        vTaskDelay(delay(dt));
        display(l);
        vTaskDelay(delay(dt));
        display(null);
        vTaskDelay(delay(dt));
        display(l);
        vTaskDelay(delay(dt));
        display(null);
        vTaskDelay(delay(dt));
        display(o);
        vTaskDelay(delay(dt));
        display(null);
        vTaskDelay(delay(dt));
        display(dot);
        vTaskDelay(delay(dt));
        display(null);
        vTaskDelay(delay(dt));
    }
}