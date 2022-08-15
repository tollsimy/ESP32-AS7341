#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ESP32_AS7341.h"

static const char *TAG = "Demo-AS7341";

#define SDA_PIN (27)
#define SCL_PIN (32)
#define I2C_PORT (0)

I2C_CONF={
    .mode = I2C_MODE_MASTER;
    .sda_io_num = SDA_PIN;
    .scl_io_num = SCL_PIN;
    .sda_pullup_en = GPIO_PULLUP_DISABLE;     //disable if you have external pullup
    .scl_pullup_en = GPIO_PULLUP_DISABLE;
    .master.clk_speed = 400000;               //I2C Full Speed
}

void AS7341_task(){
    //Install I2C Driver
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &(I2C_CONF)));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    for(;;){
        ESP32_AS7341 AS7341={0};
        ESP_LOGI(TAG, "AS7341 task started");

        ESP_ERROR_CHECK(AS_init(&AS7341, I2C_PORT));
        AS_enable(&AS7341);
        AS_setASTEP(&AS7341, 50);
        AS_enableSpectralMeasurement(&AS7341, true);
        AS_readAllChannels(&AS7341);

        ESP_LOGI(TAG, "AS7341: 410nm: %d, 440nm: %d, 470nm: %d, 510nm: %d, "
                "Clear: %d, NIR: %d, 550nm: %d, 583nm: %d, 620nm: %d, 670nm: %d, "
                "Clear: %d, NIR: %d", AS7341.data[0], AS7341.data[1],
                AS7341.data[2], AS7341.data[3], AS7341.data[4],
                AS7341.data[5], AS7341.data[6], AS7341.data[7],
                AS7341.data[8], AS7341.data[9], AS7341.data[10],
                AS7341.data[11]);

        AS_disableAll(&AS7341);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_PORT));
}

void app_main(void)
{
    xTaskCreate(&AS7341_task, "AS7341_task", 4096, NULL, 5, NULL);
}

