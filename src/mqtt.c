#include "mqtt.h"

static const char *TAG = "MQTT";

static char mqtt_broker_uri[64] = {0};
static char mqtt_username[32] = {0};
static char mqtt_password[32] = {0};
static int32_t mqtt_port = 1883;
static char mqtt_will_topic[64] = {0};
static char subscribe_topic[64] = {0};

char mqtt_stats_topic[64] = {0};
char mqtt_settings_topic[64] = {0};
char mqtt_system_topic[64] = {0};
char mqtt_cmnd_topic[64] = {0};

esp_mqtt_client_handle_t client;
static void mqtt_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch (event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        esp_mqtt_client_publish(client, mqtt_will_topic, "online", 0, 1, 1);
        esp_mqtt_client_subscribe(client, subscribe_topic, 0);
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

        // Handle incoming MQTT commands 
        if (strstr(topic, mqtt_cmnd_topic))
        {
            // Check for system reset command
            if (strstr(topic, "system_reset") && atoi(data) == 1)
            {
                ESP_LOGI(TAG, "Received command for system reset via MQTT");
                esp_restart();
            }
            // Check for refresh settings command
            else if (strstr(topic, "refresh") && atoi(data) == 1)
            {
                ESP_LOGI(TAG, "Received refresh command via MQTT");
                xEventGroupSetBits(inverter_event_group, INV_EVENT_REFRESH_SETTINGS);
                break;
            }
            // Check for set setting commands
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
    // Read MQTT settings from NVS
    nvs_read_str("mqtt_broker", mqtt_broker_uri, sizeof(mqtt_broker_uri));
    nvs_read_str("mqtt_username", mqtt_username, sizeof(mqtt_username));
    nvs_read_str("mqtt_password", mqtt_password, sizeof(mqtt_password));
    nvs_read_int("mqtt_port", &mqtt_port);

    // Construct MQTT topics based on device ID
    snprintf(mqtt_will_topic, sizeof(mqtt_will_topic), "%s/status", device_id);
    snprintf(mqtt_stats_topic, sizeof(mqtt_stats_topic), "%s/stats", device_id);
    snprintf(mqtt_settings_topic, sizeof(mqtt_settings_topic), "%s/settings", device_id);
    snprintf(mqtt_system_topic, sizeof(mqtt_system_topic), "%s/system", device_id);
    snprintf(mqtt_cmnd_topic, sizeof(mqtt_cmnd_topic), "%s/cmnd", device_id);
    snprintf(subscribe_topic, sizeof(subscribe_topic), "%s/#", mqtt_cmnd_topic);

    // Check if MQTT broker URI is empty
    if (strlen(mqtt_broker_uri) == 0)
    {
        ESP_LOGW(TAG, "MQTT broker URI is empty. MQTT will not be initialized.");
        return;
    }
    // Initialize MQTT client configuration
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = mqtt_broker_uri,
        .credentials.client_id = device_id,
        .network.timeout_ms = 10000,
        .session.keepalive = 15,
        .session.last_will = {
            .topic = mqtt_will_topic,
            .msg = "offline",
            .qos = 1,
            .retain = true,
        },
    };

    // Override port, username, and password if they are set
    if (mqtt_port > 0)
    {
        cfg.broker.address.port = mqtt_port;
    }
    if (strlen(mqtt_username) > 0)
    {
        cfg.credentials.username = mqtt_username;
    }
    if (strlen(mqtt_password) > 0)
    {
        cfg.credentials.authentication.password = mqtt_password;
    }

    client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

int mqtt_publish_topic(const char *topic, const char *payload)
{
    char topic_buffer[64];
    snprintf(topic_buffer, sizeof(topic_buffer), "%s", topic);

    return esp_mqtt_client_publish(client, topic_buffer, payload, 0, 0, 0);
}




// void mqtt_publish_sensor(const char *sensor_name, float value, const char *unit)
// {
//     char topic[64];
//     char payload[32];

//     snprintf(topic, sizeof(topic), "esp32_bms/sensor/%s/state", sensor_name);
//     snprintf(payload, sizeof(payload), "%.3f", value);

//     esp_mqtt_client_publish(client, topic, payload, 0, 0, 0);
// }