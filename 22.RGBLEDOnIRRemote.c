#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "driver/rmt_rx.h"

#define IR_RX_GPIO_NUM            27
#define IR_RESOLUTION_HZ          1000000
#define MAX_SYMBOL_COUNT          100     
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY 5000
#define R GPIO_NUM_26
#define G GPIO_NUM_33
#define B GPIO_NUM_32

static const char *TAG = "\033[38;2;0;128;255mJARVIS\033[0m";

typedef struct {
    rmt_symbol_word_t symbols[MAX_SYMBOL_COUNT];
    size_t num_symbols;
} ir_packet_t;

static QueueHandle_t ir_queue = NULL;

typedef enum {
    STATE_IDLE,
    STATE_ENTER_R,
    STATE_ENTER_G,
    STATE_ENTER_B
} rgb_state_t;

static rgb_state_t rgb_state = STATE_IDLE;

static int R_val = 0;
static int G_val = 0;
static int B_val = 0;
static int temp_val = 0;

static bool boss_mode = false;
static bool logout_armed = false;
static bool rgb_reenter_armed = false;

bool decode_nec(rmt_symbol_word_t *symbols, size_t len, uint8_t *out_command) {
    if (len < 33) return false;

    if (symbols[0].duration0 < 8000 || symbols[0].duration0 > 10000 ||
        symbols[0].duration1 < 4000 || symbols[0].duration1 > 5000) {
        return false; 
    }

    uint32_t raw_data = 0;
    for (int i = 0; i < 32; i++) {
        if (symbols[i + 1].duration1 > 1000) {
            raw_data |= (1UL << i);
        }
    }

    if (raw_data == 0) return false;

    uint8_t command = (uint8_t)((raw_data >> 16) & 0xFF);
    uint8_t inv_command = (raw_data >> 24) & 0xFF;

    if ((command ^ inv_command) != 0xFF) {
        return false;
    }

    *out_command = command;
    return true;
}

static bool rmt_rx_done_callback(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_data) {
    BaseType_t high_task_wakeup = pdFALSE;
    ir_packet_t packet;
    packet.num_symbols = edata->num_symbols;
    size_t to_copy = (packet.num_symbols < MAX_SYMBOL_COUNT) ? packet.num_symbols : MAX_SYMBOL_COUNT;
    memcpy(packet.symbols, edata->received_symbols, to_copy * sizeof(rmt_symbol_word_t));

    xQueueSendFromISR(ir_queue, &packet, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

bool validate(int pwd[4])
{
    char correct_password[4] = {'1','2','3','4'};
    for (int i = 0; i < 4; i++)
    {
        if (correct_password[i] != pwd[i]) return false;
    }
    return true;
}

void rgb_pwm_init(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t red_conf = {
        .gpio_num   = R,
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&red_conf));

    ledc_channel_config_t green_conf = {
        .gpio_num   = G,
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL_1,
        .timer_sel  = LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&green_conf));

    ledc_channel_config_t blue_conf = {
        .gpio_num   = B,
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL_2,
        .timer_sel  = LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&blue_conf));
}

#define pwm_init rgb_pwm_init

void writeRGB(int r, int g, int b)
{
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, r);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, g);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, b);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_2);
}

void app_main(void)
{
    pwm_init();
    ESP_LOGI(TAG, "Starting IR Receiver...");
    ir_queue = xQueueCreate(5, sizeof(ir_packet_t));

    rmt_rx_channel_config_t rx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = IR_RX_GPIO_NUM,
        .mem_block_symbols = 64,
        .resolution_hz = IR_RESOLUTION_HZ,
    };

    rmt_channel_handle_t rx_chan = NULL;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &rx_chan));

    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan, &cbs, NULL));

    ESP_ERROR_CHECK(rmt_enable(rx_chan));

    static rmt_symbol_word_t raw_symbols[MAX_SYMBOL_COUNT];
    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 1250,
        .signal_range_max_ns = 12000000,
    };
    
    ESP_ERROR_CHECK(rmt_receive(rx_chan, raw_symbols, sizeof(raw_symbols), &receive_config));

    ESP_LOGI(TAG, "Listening on GPIO %d. Ready!", IR_RX_GPIO_NUM);
    ESP_LOGI(TAG, "\033[38;2;0;128;255mEnter the Password!\033[0m", IR_RX_GPIO_NUM);

    ir_packet_t incoming_packet;
    uint8_t cmd;
    int top = 0;
    int pwd[4];
    char c = '\0';

    while (1) {
        if (xQueueReceive(ir_queue, &incoming_packet, portMAX_DELAY)) 
        {
            if(decode_nec(incoming_packet.symbols, incoming_packet.num_symbols, &cmd))
            {
                switch(cmd)
                {
                    case (0x45): ESP_LOGI(TAG,"\033[38;2;0;128;255m1\033[0m"); c = '1'; break;
                    case (0x46): ESP_LOGI(TAG,"\033[38;2;0;128;255m2\033[0m"); c = '2'; break;
                    case (0x47): ESP_LOGI(TAG,"\033[38;2;0;128;255m3\033[0m"); c = '3'; break;
                    case (0x44): ESP_LOGI(TAG,"\033[38;2;0;128;255m4\033[0m"); c = '4'; break;
                    case (0x40): ESP_LOGI(TAG,"\033[38;2;0;128;255m5\033[0m"); c = '5'; break;
                    case (0x43): ESP_LOGI(TAG,"\033[38;2;0;128;255m6\033[0m"); c = '6'; break;
                    case (0x07): ESP_LOGI(TAG,"\033[38;2;0;128;255m7\033[0m"); c = '7'; break;
                    case (0x15): ESP_LOGI(TAG,"\033[38;2;0;128;255m8\033[0m"); c = '8'; break;
                    case (0x09): ESP_LOGI(TAG,"\033[38;2;0;128;255m9\033[0m"); c = '9'; break;
                    case (0x19): ESP_LOGI(TAG,"\033[38;2;0;128;255m0\033[0m"); c = '0'; break;
                    case (0x16):
                        if (boss_mode)
                        {
                            logout_armed = true;
                            rgb_reenter_armed = false;
                            ESP_LOGI(TAG,"\033[38;2;255;255;0mLogout armed. Press # to confirm.\033[0m");
                        }
                        c = '\0';
                        break;
                    case (0x0D):
                        if (boss_mode && logout_armed)
                        {
                            boss_mode = false;
                            logout_armed = false;
                            rgb_reenter_armed = false;
                            rgb_state = STATE_IDLE;
                            temp_val = 0;
                            top = 0;
                            ESP_LOGI(TAG,"\033[38;2;255;0;0mLogged out.\033[0m");
                            ESP_LOGI(TAG,"\033[38;2;0;128;255mRe-enter the password to continue.");
                        }
                        else if (boss_mode && rgb_reenter_armed)
                        {
                            rgb_reenter_armed = false;
                            rgb_state = STATE_ENTER_R;
                            temp_val = 0;
                            ESP_LOGI(TAG, "Enter \033[38;2;255;0;0mR\033[0m value");
                        }
                        else if (boss_mode)
                        {
                            rgb_reenter_armed = true;
                            ESP_LOGI(TAG,"\033[38;2;255;255;0mRe-enter RGB armed. Press # to confirm.\033[0m");
                        }
                        c = '\0';
                        break;
                    case (0x1C): ESP_LOGI(TAG,"\033[38;2;0;128;255mOK\033[0m"); c = 'O'; break;
                    default: c = '\0'; break;
                }

                if (c >= '0' && c <= '9')
                {
                    logout_armed = false;
                    rgb_reenter_armed = false;
                    if (rgb_state == STATE_IDLE)
                    {
                        pwd[top++] = c;
                    }
                    else
                    {
                        temp_val = temp_val * 10 + (c - '0');
                        if (temp_val > 255) temp_val = 255;
                        ESP_LOGI(TAG, "Value: %d", temp_val);
                    }
                }

                if (cmd == 0x1C)
                {
                    if (rgb_state == STATE_IDLE && !boss_mode)
                    {
                        if (validate(pwd))
                        {
                            boss_mode = true;
                            ESP_LOGI(TAG, "\033[38;2;0;255;0mWelcome, Boss!\033[0m");
                            ESP_LOGI(TAG, "Enter \033[38;2;255;0;0mR\033[0m value");
                            rgb_state = STATE_ENTER_R;
                            temp_val = 0;
                        }
                        else
                        {
                            ESP_LOGI(TAG, "\033[38;2;255;0;0mIncorrect Password! Try Again.\033[0m");
                        }
                        top = 0;
                    }
                    else if (rgb_state == STATE_ENTER_R)
                    {
                        R_val = temp_val;
                        temp_val = 0;
                        rgb_state = STATE_ENTER_G;
                        ESP_LOGI(TAG, "Enter \033[38;2;0;255;0mG\033[0m value");
                    }
                    else if (rgb_state == STATE_ENTER_G)
                    {
                        G_val = temp_val;
                        temp_val = 0;
                        rgb_state = STATE_ENTER_B;
                        ESP_LOGI(TAG, "Enter \033[38;2;0;0;255mB\033[0m value");
                    }
                    else if (rgb_state == STATE_ENTER_B)
                    {
                        B_val = temp_val;
                        temp_val = 0;
                        rgb_state = STATE_IDLE;
                        ESP_LOGI(TAG,"\033[38;2;%d;%d;%dmYour Color is: --> \033[48;2;%d;%d;%dm            \033[0m",R_val,G_val,B_val,R_val,G_val,B_val);
                        writeRGB(R_val, G_val, B_val);
                    }
                }
            }
            rmt_receive(rx_chan, raw_symbols, sizeof(raw_symbols), &receive_config);
        }
    }
}