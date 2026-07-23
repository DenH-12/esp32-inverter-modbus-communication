#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

#define NVS_NAMESPACE "storage"
#define NVS_KEY_NEED_INIT "need_init"
#define NVS_DISCOVERY_TOPIC "discovery_topic"
#define NVS_KEY_WIFI_SSID "wifi_ssid"
#define NVS_KEY_WIFI_PASS "wifi_password"
#define NVS_KEY_MQTT_HOST "mqtt_broker"
#define NVS_KEY_MQTT_PORT "mqtt_port"
#define NVS_KEY_MQTT_USER "mqtt_username"
#define NVS_KEY_MQTT_PASS "mqtt_password"
#define NVS_KEY_DATA_SEC "data_sec"
#define NVS_KEY_SYS_SEC "sys_data_sec"
#define NVS_KEY_WEB_PORT "web_port"

void nvs_init(void);
esp_err_t nvs_reset_saved_settings();
