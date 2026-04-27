#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "wifi.h"
#include "sniffer.h"

static const char *TAG = "APP";
uint64_t uptime_seconds;
esp_reset_reason_t last_reset_reason;

/* -------- MAIN -------- */
void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay to allow logging to initialize
    uptime_seconds = 0;
    last_reset_reason = esp_reset_reason();

    wifi_init();
    sniffer_init();

    ESP_LOGI(TAG, "System ready");

    // Tasks
    xTaskCreate(inverter_read_task, "inverter_read_task", 8192, NULL, 5, NULL);
    // xTaskCreate(sniffer_task, "modbus_scan_task", 4096, NULL, 5, NULL);
}
