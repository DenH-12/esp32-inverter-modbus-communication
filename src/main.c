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

static void ota_validation_task(void *pvParameter)
{
    vTaskDelay(pdMS_TO_TICKS(15000));

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;

    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK)
    {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
        {
            ESP_LOGI("OTA", "System stable for 15s. Confirming firmware validity...");
            esp_ota_mark_app_valid_cancel_rollback();
        }
    }

    vTaskDelete(NULL);
}

/* -------- MAIN -------- */
void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay to allow logging to initialize

    init_spiffs();
    nvs_init();
    wifi_init();
    mqtt_init();
    webserver_init();

    esp_err_t err = modbus_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize Modbus");
        return;
    }

    ESP_LOGI(TAG, "System ready");

    // Tasks
    if (err == ESP_OK)
    {
        xTaskCreate(modbus_read_task, "modbus_read_task", 8192, NULL, 5, NULL);
    }
    xTaskCreate(update_system_info_task, "update_system_info_task", 4096, NULL, 5, NULL);
    xTaskCreate(ota_validation_task, "ota_val_task", 3072, NULL, 5, NULL);
}
