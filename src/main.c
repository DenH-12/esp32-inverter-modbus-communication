#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "wifi.h"
#include "modbus.h"
#include "myNVS.h"
#include "spiffs.h"

static const char *TAG = "APP";

/* -------- MAIN -------- */
void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay to allow logging to initialize

    init_fs();
    nvs_init();
    wifi_init();
    mqtt_init();
    webserver_init();
    modbus_init();

    ESP_LOGI(TAG, "System ready");

    // Tasks
    xTaskCreate(modbus_read_task, "modbus_read_task", 8192, NULL, 5, NULL);
    xTaskCreate(update_system_info_task, "update_system_info_task", 4096, NULL, 5, NULL);
}
