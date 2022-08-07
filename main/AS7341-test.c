#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "../components/aaa/bbb/AS7341/ESP32_AS7341.h"

uint8_t AS_SDA_PIN=18;
uint8_t AS_SCL_PIN=19;
uint8_t AS_I2C_PORT=0;

static const char *TAG = "Demo-AS7341";

void AS7341_task(){
    for(;;){
        ESP32_AS7341 AS7341={0};
        ESP_LOGI(TAG, "AS7341 task started");

        AS_init(&AS7341);
        AS_enable(&AS7341);
        AS_setASTEP(&AS7341, 50);
        AS_readAllChannels(&AS7341);

        ESP_LOGI(TAG, "AS7341: 410nm: %d, 440nm: %d, 470nm: %d, 510nm: %d, \
                Clear: %d, NIR: %d, 550nm: %d, 583nm: %d, 620nm: %d, 670nm: %d, \
                Clear: %d, NIR: %d", AS7341.data[0], AS7341.data[1],
                AS7341.data[2], AS7341.data[3], AS7341.data[4],
                AS7341.data[5], AS7341.data[6], AS7341.data[7],
                AS7341.data[8], AS7341.data[9], AS7341.data[10],
                AS7341.data[11]);

        AS_disableAll(&AS7341);
        AS_delete();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    xTaskCreate(&AS7341_task, "AS7341_task", 4096, NULL, 5, NULL);
}

