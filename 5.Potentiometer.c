#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"

#define ADC_CH ADC1_CHANNEL_6   // GPIO34

int analogRead(adc_channel_t channel)
{
    static adc_oneshot_unit_handle_t adc_handle = NULL;

    if (adc_handle == NULL) {
        adc_oneshot_unit_init_cfg_t init_cfg = {
            .unit_id = ADC_UNIT_1,
        };
        adc_oneshot_new_unit(&init_cfg, &adc_handle);

        adc_oneshot_chan_cfg_t chan_cfg = {
            .bitwidth = ADC_BITWIDTH_DEFAULT,
            .atten = ADC_ATTEN_DB_12,      // 0 – 3.3V range
        };
        adc_oneshot_config_channel(adc_handle, channel, &chan_cfg);
    }

    int value = 0;
    adc_oneshot_read(adc_handle, channel, &value);
    return value;      
}

void app_main(void)
{
    while (1) {
        int v = analogRead(ADC_CH);
        printf("Analog value: %d\n", v);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
