#pragma once

#include "esp_log.h"
#include "mqtt_client.h"
#include "freertos/event_groups.h"
#include "sniffer.h"

#define MQTT_BROKER_URI "mqtt://10.42.0.1:1883"
#define MQTT_TOPIC_PREFIX "inverter/"
#define MQTT_REALTIME_TOPIC "inverter/stats"
#define MQTT_SETTINGS_TOPIC "inverter/settings"

extern QueueHandle_t btn_queue;

void mqtt_init(void);

void mqtt_publish_sensor(const char* sensor_name, float value, const char* unit);
int mqtt_publish_readings(const char *topic, const char *payload);