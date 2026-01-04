#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/rmt_rx.h"

#define IR_RX_GPIO_NUM            27
#define IR_RESOLUTION_HZ          1000000 // 1 tick = 1us
#define MAX_SYMBOL_COUNT          100     
#define MOTOR_EN   GPIO_NUM_25   // L293D Enable (PWM)
#define MOTOR_IN1  GPIO_NUM_33   // L293D IN1
#define MOTOR_IN2  GPIO_NUM_32   // L293D IN2
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT   // 0–255
#define LEDC_FREQUENCY 20000                // 25 kHz

static const char *TAG = "\033[38;2;0;128;255mJARVIS\033[0m";
bool current_direction = true;

typedef struct {
    rmt_symbol_word_t symbols[MAX_SYMBOL_COUNT];
    size_t num_symbols;
} ir_packet_t;

static QueueHandle_t ir_queue = NULL;

/**
 * NEC Decoder Logic
 */
bool decode_nec(rmt_symbol_word_t *symbols, size_t len, uint8_t *out_command) {
    // NEC frames consist of a leader + 32 bits = 33 symbols.
    // If it's shorter, it's likely noise or a repeat code.
    if (len < 33) return false;

    // 1. Validate Leader Pulse (9ms Pulse, 4.5ms Space)
    // duration0 = Pulse (usually Low for IR sensors), duration1 = Space (High)
    if (symbols[0].duration0 < 8000 || symbols[0].duration0 > 10000 ||
        symbols[0].duration1 < 4000 || symbols[0].duration1 > 5000) {
        return false; 
    }

    uint32_t raw_data = 0;
    for (int i = 0; i < 32; i++) {
        // Logic 1 space is ~1690us, Logic 0 space is ~560us
        // We check the space duration (duration1) to distinguish bits
        if (symbols[i + 1].duration1 > 1000) {
            raw_data |= (1UL << i);
        }
    }

    if (raw_data == 0) return false;

    // Standard NEC mapping: [Addr][Inv-Addr][Cmd][Inv-Cmd]
    //uint8_t address = (uint8_t)(raw_data & 0xFF);
    uint8_t command = (uint8_t)((raw_data >> 16) & 0xFF);
    uint8_t inv_command = (raw_data >> 24) & 0xFF;

    /*if (raw_data != 0) {
        ESP_LOGI(TAG, "MATCH! Address: 0x%02X | Command: 0x%02X", address, command);
    }*/
    if ((command ^ inv_command) != 0xFF) {
        return false;
    }

    *out_command = command;
    return true;
}

/**
 * RMT RX Callback (Triggered when the frame ends)
 */
static bool rmt_rx_done_callback(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_data) {
    BaseType_t high_task_wakeup = pdFALSE;
    ir_packet_t packet;
    packet.num_symbols = edata->num_symbols;
    size_t to_copy = (packet.num_symbols < MAX_SYMBOL_COUNT) ? packet.num_symbols : MAX_SYMBOL_COUNT;
    memcpy(packet.symbols, edata->received_symbols, to_copy * sizeof(rmt_symbol_word_t));

    xQueueSendFromISR(ir_queue, &packet, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

void set_motor_direction(bool dir)
{
    if (dir)
    {
        gpio_set_level(MOTOR_IN1, 1);
        gpio_set_level(MOTOR_IN2, 0);
        current_direction = dir;
    }
    else
    {
        gpio_set_level(MOTOR_IN1, 0);
        gpio_set_level(MOTOR_IN2, 1);
        current_direction = dir;
    }
}

void motor_set_speed(uint32_t duty)
{
    static uint32_t current_duty = 0;

    set_motor_direction(current_direction);

    if (duty > current_duty)
    {
        // Ramp UP
        for (uint32_t d = current_duty; d <= duty; d += 4)
        {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, d);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    else if (duty < current_duty)
    {
        // Ramp DOWN
        for (int32_t d = current_duty; d >= (int32_t)duty; d -= 4)
        {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, d);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    current_duty = duty;
}


void motor_stop(void)
{
    gpio_set_level(MOTOR_IN1, 0);
    gpio_set_level(MOTOR_IN2, 0);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void motor_gpio_and_pwm_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR_IN1) | (1ULL << MOTOR_IN2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    ledc_timer_config_t timer_conf = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel_conf = {
        .gpio_num   = MOTOR_EN,
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&channel_conf);
}

void app_main(void)
{
    motor_gpio_and_pwm_init();
    ESP_LOGI(TAG, "Starting IR Receiver...");
    ir_queue = xQueueCreate(5, sizeof(ir_packet_t));

    // 1. Setup RMT RX Channel (Minimal config for v5.0 compatibility)
    rmt_rx_channel_config_t rx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = IR_RX_GPIO_NUM,
        .mem_block_symbols = 64,
        .resolution_hz = IR_RESOLUTION_HZ,
    };

    rmt_channel_handle_t rx_chan = NULL;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &rx_chan));

    // 2. Register Callback
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan, &cbs, NULL));

    // 3. Enable Channel
    ESP_ERROR_CHECK(rmt_enable(rx_chan));

    // 4. Start Receiving
    // In v5.0/v5.1, 'signal_range_max_ns' acts as the IDLE THRESHOLD.
    // 12ms = 12,000,000 nanoseconds.
    static rmt_symbol_word_t raw_symbols[MAX_SYMBOL_COUNT];
    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 1250,      // Filter noise shorter than 1.25us
        .signal_range_max_ns = 12000000,  // END frame if signal stays still for 12ms
    };
    
    ESP_ERROR_CHECK(rmt_receive(rx_chan, raw_symbols, sizeof(raw_symbols), &receive_config));

    ESP_LOGI(TAG, "Listening on GPIO %d. Ready!", IR_RX_GPIO_NUM);
    ESP_LOGE(TAG, "\033[38;2;255;0;0mWARNING! Do not switch the motor's direction before setting it to Zero!\033[0m", IR_RX_GPIO_NUM);

    ir_packet_t incoming_packet;
    uint8_t cmd;
    uint8_t speeds[5] = {0,192,213,234,255};
    uint8_t current_speed_index = 0;
    while (1) {
        if (xQueueReceive(ir_queue, &incoming_packet, portMAX_DELAY)) 
        {
            if(decode_nec(incoming_packet.symbols, incoming_packet.num_symbols, &cmd))
            {
                switch(cmd)
                {
                    case (0x18):
                        ESP_LOGI(TAG,"\033[38;2;0;128;255mUp\033[0m");
                        if (current_speed_index == 4) break;
                        current_speed_index++;
                        motor_set_speed(speeds[current_speed_index]);
                        ESP_LOGI(TAG,"\033[38;2;255;128;0mSpeed: %d\033[0m", speeds[current_speed_index]);
                        break;
                    case (0x08):
                        ESP_LOGI(TAG,"\033[38;2;0;128;255mLeft\033[0m");
                        set_motor_direction(0);
                        ESP_LOGI(TAG,"\033[38;2;0;255;128mSpinning Left\033[0m");
                        break;
                    case (0x1C):
                        ESP_LOGI(TAG,"\033[38;2;0;128;255mOK\033[0m");
                        motor_stop();
                        ESP_LOGI(TAG,"\033[38;2;255;0;0mMotor Stopped.\033[0m");
                        break;
                    case (0x5A):
                        ESP_LOGI(TAG,"\033[38;2;0;128;255mRight\033[0m");
                        set_motor_direction(1);
                        ESP_LOGI(TAG,"\033[38;2;0;255;128mSpinning Right\033[0m");
                        break;
                    case (0x52):
                        ESP_LOGI(TAG,"\033[38;2;0;128;255mDown\033[0m");
                        if (current_speed_index == 0) break;
                        current_speed_index--;
                        motor_set_speed(speeds[current_speed_index]);
                        ESP_LOGI(TAG,"\033[38;2;255;128;0mSpeed: %d\033[0m", speeds[current_speed_index]);
                        break;
                }
            }
            
            // Re-arm for next signal
            rmt_receive(rx_chan, raw_symbols, sizeof(raw_symbols), &receive_config);
        }
    }
}