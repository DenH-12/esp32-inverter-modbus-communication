#pragma once

#include "mqtt.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "webserver.h"
#include "esp_mac.h"
#include "myNVS.h"

#define WIFI_RETRY_INTERVAL_MS 30000

static char ssid[32] = {0};
static char password[64] = {0};
static TimerHandle_t wifi_retry_timer = NULL;

static void wifi_retry_timer_callback(TimerHandle_t xTimer);
void wifi_init(void);
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);