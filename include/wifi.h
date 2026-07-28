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

esp_err_t wifi_init(void);

extern char wifi_ssid[33];
extern char wifi_password[64];

