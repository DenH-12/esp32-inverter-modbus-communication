#pragma once

#include "esp_log.h"
#include "mqtt_client.h"
#include "freertos/event_groups.h"
#include "sniffer.h"
#include "myNVS.h"
#include "system.h"

extern QueueHandle_t btn_queue;
extern char device_id[];

extern char mqtt_stats_topic[];
extern char mqtt_settings_topic[];
extern char mqtt_system_realtime_topic[];
extern char mqtt_system_topic[];
extern char mqtt_cmnd_topic[];
extern char mqtt_will_topic[];

esp_err_t mqtt_init(void);

void mqtt_publish_sensor(const char* sensor_name, float value, const char* unit);
int mqtt_publish_topic(const char *topic, const char *payload, bool retain);