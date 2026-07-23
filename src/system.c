#include "system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mqtt.h"
#include "haDiscovery.h"
#include "esp_timer.h"
#include "wifi.h"

extern uint8_t sys_data_send_interval_s;

uint64_t uptime_seconds;
esp_reset_reason_t last_reset_reason;

extern char device_id[];
extern char wifi_ip[];
extern char wifi_ssid[];

void update_system_info_task(void *pvParameters)
{
    char json_string[512];
    wifi_ap_record_t ap_info;

    last_reset_reason = esp_reset_reason();
    snprintf(json_string, sizeof(json_string),
             "{\"name\": \"%s\", \"version\": \"%s\", \"ssid\": \"%s\", \"ip\": \"%s\", \"reset_reason\": %d}",
             device_id, VERSION, wifi_ssid, wifi_ip, last_reset_reason);
    mqtt_publish_topic(mqtt_system_topic, json_string, true);

    size_t free_heap;
    size_t min_heap_free;

    if (sys_data_send_interval_s <= 0)
    {
        vTaskDelete(NULL);
    }

    while (1)
    {
        // Check MQTT connection
        if (!(xEventGroupGetBits(inverter_event_group) & INV_EVENT_MQTT_CONNECTED))
        {
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        // System status update
        uptime_seconds = esp_timer_get_time() / 1000000ULL;
        free_heap = esp_get_free_heap_size();
        min_heap_free = esp_get_minimum_free_heap_size();

        int8_t rssi = 0;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
            rssi = ap_info.rssi;

        snprintf(json_string, sizeof(json_string),
                 "{\"uptime\": %llu, \"rssi\": %d, \"free_heap\": %lu, \"min_free_heap\": %lu}",
                 uptime_seconds, rssi, free_heap, min_heap_free);

        mqtt_publish_topic(mqtt_system_realtime_topic, json_string, false);

        vTaskDelay(pdMS_TO_TICKS(sys_data_send_interval_s * 1000));
    }
}