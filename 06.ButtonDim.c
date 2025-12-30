#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/projdefs.h"
#include "esp_log.h"
#define B1 GPIO_NUM_4
#define B2 GPIO_NUM_5
#define BUZZ GPIO_NUM_22
#define LED GPIO_NUM_2

int delay(int milliseconds)
{
    return milliseconds/portTICK_PERIOD_MS;
}

// value 0-255
void analogWrite(int pin, int value)
{
    static bool initialized[48] = {0};   
    int channel = pin % 8;               

    if (!initialized[pin]) {
        ledc_timer_config_t timer = {
            .speed_mode       = LEDC_LOW_SPEED_MODE,
            .timer_num        = LEDC_TIMER_0,
            .duty_resolution  = LEDC_TIMER_8_BIT,  // 0–255
            .freq_hz          = 5000,              // 5 kHz PWM
            .clk_cfg          = LEDC_AUTO_CLK
        };
        ledc_timer_config(&timer);

        ledc_channel_config_t ch = {
            .gpio_num       = pin,
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = channel,
            .intr_type      = LEDC_INTR_DISABLE,
            .timer_sel      = LEDC_TIMER_0,
            .duty           = 0,
            .hpoint         = 0
        };
        ledc_channel_config(&ch);

        initialized[pin] = true;
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, value);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

void beep()
{
    int bzdl = 300;
    ESP_LOGI("\x1b[38;2;255;128;0mBuzzer", "BUZZZZZ!!!\x1b[0m");
    analogWrite(BUZZ, 127);
    vTaskDelay(delay(bzdl));
    analogWrite(BUZZ,0);
}

void app_main(void)
{
    gpio_reset_pin(B1);
    gpio_reset_pin(B2);
    gpio_reset_pin(BUZZ);
    gpio_reset_pin(LED);
    gpio_set_direction(B1,GPIO_MODE_INPUT);
    gpio_set_direction(B2,GPIO_MODE_INPUT);
    gpio_set_direction(BUZZ,GPIO_MODE_OUTPUT);
    gpio_set_direction(LED,GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(B1,GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(B2,GPIO_PULLUP_ONLY);
    bool b1old = true, b2old = true;
    bool b1new, b2new;
    int level = 0;
    while (1) {
        b1new = gpio_get_level(B1);
        if (b1new == 1 && b1old == 0)
        {
            ESP_LOGI("\x1b[38;2;0;120;255mJARVIS", "Button 1 Released.\x1b[0m");
            if (level == 0) beep();
            else level--;
        }
        b2new = gpio_get_level(B2);
        if (b2new == 1 && b2old == 0)
        {
            ESP_LOGI("\x1b[38;2;0;120;255mJARVIS", "Button 2 Released.\x1b[0m");
            if (level == 3) beep();
            else level++;
        }
        switch (level)
        {
            case 0:
                analogWrite(LED, 0);
                break;
            
            case 1:
                analogWrite(LED, 64);
                break;

            case 2:
                analogWrite(LED, 128);
                break;

            case 3:
                analogWrite(LED, 255);
                break;
        }
        if (b1new != b1old) b1old = b1new;
        if (b2new != b2old) b2old = b2new;
        vTaskDelay(delay(10));
    }
}