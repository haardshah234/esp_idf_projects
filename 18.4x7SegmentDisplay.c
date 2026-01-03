#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_rom_sys.h"
#define LATH GPIO_NUM_5
#define D1 GPIO_NUM_21
#define D2 GPIO_NUM_2
#define D3 GPIO_NUM_4
#define D4 GPIO_NUM_19
#define MOSI GPIO_NUM_23
#define SCLK GPIO_NUM_18
#define DIGIT_ON(pin)   gpio_set_level(pin, 0)
#define DIGIT_OFF(pin)  gpio_set_level(pin, 1)

spi_device_handle_t spi;

void shiftOut(uint8_t data)
{
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &data,
    };

    spi_device_transmit(spi, &t);

    gpio_set_level(LATH, 0);
    esp_rom_delay_us(1);
    gpio_set_level(LATH, 1);
    esp_rom_delay_us(1);
    gpio_set_level(LATH, 0);
}

void app_main(void)
{
    gpio_set_direction(LATH, GPIO_MODE_OUTPUT);

    gpio_set_direction(D1, GPIO_MODE_OUTPUT);
    gpio_set_direction(D2, GPIO_MODE_OUTPUT);
    gpio_set_direction(D3, GPIO_MODE_OUTPUT);
    gpio_set_direction(D4, GPIO_MODE_OUTPUT);

    DIGIT_OFF(D1);
    DIGIT_OFF(D2);
    DIGIT_OFF(D3);
    DIGIT_OFF(D4);

    spi_bus_config_t buscfg = {
        .mosi_io_num = MOSI,
        .miso_io_num = -1,
        .sclk_io_num = SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,   // 1 MHz
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
        .flags = SPI_DEVICE_TXBIT_LSBFIRST,
    };
    spi_bus_add_device(SPI2_HOST, &devcfg, &spi);

    int H = 0b00110111;
    int E = 0b01001111;
    int L = 0b00001110;
    int P = 0b01100111;
    while (1)
    {
    int deltime = 1000;
    shiftOut(H);
    DIGIT_ON(D1);
    vTaskDelay(pdMS_TO_TICKS(deltime));
    DIGIT_OFF(D1);

    shiftOut(E);
    DIGIT_ON(D2);
    vTaskDelay(pdMS_TO_TICKS(deltime));
    DIGIT_OFF(D2);

    shiftOut(L);
    DIGIT_ON(D3);
    vTaskDelay(pdMS_TO_TICKS(deltime));
    DIGIT_OFF(D3);

    shiftOut(P);
    DIGIT_ON(D4);
    vTaskDelay(pdMS_TO_TICKS(deltime));
    DIGIT_OFF(D4);
    }
}
