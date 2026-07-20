#pragma once

#include "esp_log.h"
#include "mqtt_client.h"
#include "freertos/event_groups.h"
#include "sniffer.h"
#include "myNVS.h"

extern QueueHandle_t btn_queue;
extern char device_id[32];

extern char mqtt_stats_topic[64];
extern char mqtt_settings_topic[64];
extern char mqtt_system_topic[64];
extern char mqtt_cmnd_topic[64];

void mqtt_init(void);

void mqtt_publish_sensor(const char* sensor_name, float value, const char* unit);
int mqtt_publish_topic(const char *topic, const char *payload);