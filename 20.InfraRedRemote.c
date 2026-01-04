#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt_rx.h"

#define IR_RX_GPIO_NUM            27
#define IR_RESOLUTION_HZ          1000000 // 1 tick = 1us
#define MAX_SYMBOL_COUNT          100     

static const char *TAG = "\033[38;2;0;128;255mJARVIS\033[0m";

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

void app_main(void)
{
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

    ESP_LOGI(TAG, "\033[38;2;0;255;128mReady! Now listening to IR signals on GPIO %d over NEC Protocol.\033[0m", IR_RX_GPIO_NUM);

    ir_packet_t incoming_packet;
    uint8_t cmd;
    
    while (1) {
        if (xQueueReceive(ir_queue, &incoming_packet, portMAX_DELAY)) 
        {
            if(decode_nec(incoming_packet.symbols, incoming_packet.num_symbols, &cmd))
            {
                switch(cmd)
                {
                    case (0x45):
                        ESP_LOGI(TAG,"\033[38;2;0;128;255m1\033[0m");
                        break;
                    case (0x46):
                        ESP_LOGI(TAG,"\033[38;2;0;128;255m2\033[0m");
                        break;
                    case (0x47):
                        ESP_LOGI(TAG,"\033[38;2;0;128;255m3\033[0m");
                        break;
                    case (0x44):
                        ESP_LOGI(TAG,"\033[38;2;0;128;255m4\033[0m");
                        break;
                    case (0x40):
                        ESP_LOGI(TAG,"\033[38;2;0;128;255m5\033[0m");
                        break;
                    case (0x43):
                        ESP_LOGI(TAG,"\033[38;2;0;128;255m6\033[0m");
                        break;
                    case (0x07):
                        ESP_LOGI(TAG,"\033[38;2;0;128;255m7\033[0m");
                        break;
                    case (0x15):
                        ESP_LOGI(TAG,"\033[38;2;0;128;255m8\033[0m");
                        break;
                    case (0x09):
                        ESP_LOGI(TAG,"\033[38;2;0;128;255m9\033[0m");
                        break;
                    case (0x16):
                        ESP_LOGI(TAG,"\033[38;2;0;128;255m*\033[0m");
                        break;
                    case (0x19):
                        ESP_LOGI(TAG,"\033[38;2;0;128;255m0\033[0m");
                        break;
                    case (0x0D):
                        ESP_LOGI(TAG,"\033[38;2;0;128;255m#\033[0m");
                        break;
                    case (0x18):
                        ESP_LOGI(TAG,"\033[38;2;0;128;255mUp\033[0m");
                        break;
                    case (0x08):
                        ESP_LOGI(TAG,"\033[38;2;0;128;255mLeft\033[0m");
                        break;
                    case (0x1C):
                        ESP_LOGI(TAG,"\033[38;2;0;128;255mOK\033[0m");
                        break;
                    case (0x5A):
                        ESP_LOGI(TAG,"\033[38;2;0;128;255mRight\033[0m");
                        break;
                    case (0x52):
                        ESP_LOGI(TAG,"\033[38;2;0;128;255mDown\033[0m");
                        break;
                }
            }
            
            // Re-arm for next signal
            rmt_receive(rx_chan, raw_symbols, sizeof(raw_symbols), &receive_config);
        }
    }
}