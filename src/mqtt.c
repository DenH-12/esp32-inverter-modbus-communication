#include "mqtt.h"

static const char *TAG = "MQTT";

esp_mqtt_client_handle_t client;

static void mqtt_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch (event_id)
    {

    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        esp_mqtt_client_subscribe(client, "inverter/cmnd/+", 0);
        xEventGroupSetBits(inverter_event_group, INV_EVENT_MQTT_CONNECTED);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT disconnected");
        xEventGroupClearBits(inverter_event_group, INV_EVENT_MQTT_CONNECTED);
        break;

    case MQTT_EVENT_DATA:
        char topic[64];
        char data[16];
        snprintf(topic, sizeof(topic), "%.*s", event->topic_len, event->topic);
        snprintf(data, sizeof(data), "%.*s", event->data_len, event->data);

        if (strcmp(topic, "inverter/cmnd/refresh") == 0)
        {
            ESP_LOGI(TAG, "Received refresh command via MQTT");
            xEventGroupSetBits(inverter_event_group, INV_EVENT_REFRESH_SETTINGS);
        }
        else
        {
            for (size_t i = 0; i < inv_settings_map_len; i++)
            {
                const reg_metadata_t *meta = &inv_settings_map[i];

                if (strstr(topic, meta->name))
                {
                    float value = atof(data);
                    uint16_t raw_value = (uint16_t)(value / meta->multiplier);

                    inv_cmd_t cmd = {.reg = meta->reg, .value = raw_value};
                    xQueueSend(inv_cmd_queue, &cmd, portMAX_DELAY);
                    
                    ESP_LOGI(TAG, "Setting %s to: %d", meta->name, raw_value);

                    break;
                }
            }
        }
        break;
    }
}

void mqtt_init(void)
{
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .network.timeout_ms = 20000,
        .session.keepalive = 60,
    };

    client = esp_mqtt_client_init(&cfg);

    esp_mqtt_client_register_event(
        client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

    esp_mqtt_client_start(client);
}

void mqtt_publish_sensor(const char *sensor_name, float value, const char *unit)
{
    char topic[64];
    char payload[32];

    snprintf(topic, sizeof(topic), "esp32_bms/sensor/%s/state", sensor_name);
    snprintf(payload, sizeof(payload), "%.3f", value);

    esp_mqtt_client_publish(client, topic, payload, 0, 0, 0);
}

int mqtt_publish_readings(const char *topic, const char *payload)
{
    char topic_buffer[64];
    snprintf(topic_buffer, sizeof(topic_buffer), "%s", topic);

    return esp_mqtt_client_publish(client, topic_buffer, payload, 0, 1, 0);
}