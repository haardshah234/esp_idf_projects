#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_rom_sys.h"
#define LATCH GPIO_NUM_5

spi_device_handle_t spi;

int delay(int milliseconds) {
    return milliseconds/portTICK_PERIOD_MS;
}

void shiftOut(uint8_t data)
{
    int inverted_data = 255 - data;
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &inverted_data,
    };

    spi_device_transmit(spi, &t);

    gpio_set_level(LATCH, 0);
    esp_rom_delay_us(1);
    gpio_set_level(LATCH, 1);
    esp_rom_delay_us(1);
    gpio_set_level(LATCH, 0);
}

void app_main(void)
{
    gpio_reset_pin(LATCH);
    gpio_set_direction(LATCH, GPIO_MODE_OUTPUT);

    gpio_config_t io_config = {
        .pin_bit_mask = (1ULL << LATCH),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_config);
    gpio_set_level(LATCH, 0);

    spi_bus_config_t busconfig = {
        .mosi_io_num = GPIO_NUM_23,
        .miso_io_num = -1,
        .sclk_io_num = GPIO_NUM_18,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_bus_initialize(SPI2_HOST, &busconfig, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t deviceconfig = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
        .flags = SPI_DEVICE_TXBIT_LSBFIRST,
    };

    spi_bus_add_device(SPI2_HOST, &deviceconfig, &spi);
    /*
    int h[8] = {0,0,1,1,0,1,1,1};
    int e[8] = {0,1,0,0,1,1,1,1};
    int l[8] = {0,0,0,0,1,1,1,0};
    int o[8] = {0,1,1,1,1,1,1,0};
    int dot[8] = {1,0,0,0,0,0,0,0};
    int null[8] = {0,0,0,0,0,0,0,0};
    */
    int H = 0b00110111;
    int E = 0b01001111;
    int L = 0b00001110;
    int O = 0b01111110;
    int Dot = 0b10000000;
    int Null = 0b00000000;
    while(1)
    {
        shiftOut(H);
        vTaskDelay(delay(500));
        shiftOut(Null);
        vTaskDelay(delay(500));
        shiftOut(E);
        vTaskDelay(delay(500));
        shiftOut(Null);
        vTaskDelay(delay(500));
        shiftOut(L);
        vTaskDelay(delay(500));
        shiftOut(Null);
        vTaskDelay(delay(500));
        shiftOut(L);
        vTaskDelay(delay(500));
        shiftOut(Null);
        vTaskDelay(delay(500));
        shiftOut(O);
        vTaskDelay(delay(500));
        shiftOut(Null);
        vTaskDelay(delay(500));
        shiftOut(Dot);
        vTaskDelay(delay(500));
        shiftOut(Null);
        vTaskDelay(delay(500));
    }
}