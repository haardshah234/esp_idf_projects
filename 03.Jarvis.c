#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

void app_main(void)
{
    char name[64];
    int idx = 0;

    printf("Enter your name: ");
    fflush(stdout);

    while (1) {
        int c = getchar();              
        if (c <= 0) 
        {                   
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (c == '\r' || c == '\n') {
            putchar('\n');              
            name[idx] = '\0';
            break;
        }

        if (c == 8 || c == 127) {
            if (idx > 0) {
                idx--;
                putchar('\b'); putchar(' '); putchar('\b');
                fflush(stdout);
            }
            continue;
        }

        if (isprint(c) && idx < (int)sizeof(name) - 1) {
            name[idx++] = (char)c;
            putchar(c);
            fflush(stdout);
        }
    }

    if (strcasecmp(name, "Haard") == 0) {
        ESP_LOGI("\x1b[38;2;0;120;255mJARVIS\x1b[0m", "\x1b[38;2;0;120;255mGreetings, Boss!\x1b[0m");
    } else {
        ESP_LOGI("\x1b[38;2;255;255;255mJARVIS\x1b[0m", "\x1b[38;2;255;255;255mHello, %s!\x1b[0m", name);
    }
}
