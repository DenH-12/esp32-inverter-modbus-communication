#include "wifi.h"

static const char *TAG = "WIFI";

static uint8_t mac[6];
char device_id[32];
char device_id_lower[64] = {0};
char wifi_ip[16] = {0};
char wifi_ssid[33] = {0};
char wifi_password[64] = {0};
char ap_password[64] = "87654321";

static TimerHandle_t wifi_retry_timer = NULL;
static void wifi_retry_timer_callback(TimerHandle_t xTimer);
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

static void init_device_id(void)
{
    esp_efuse_mac_get_default(mac);
    snprintf(device_id, sizeof(device_id), "ESP32_Solar_%02X%02X%02X", mac[3], mac[4], mac[5]);
    // in lowercase
    for (int i = 0; device_id[i] != '\0' && i < sizeof(device_id_lower) - 1; i++)
    {
        device_id_lower[i] = tolower((unsigned char)device_id[i]);
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        if (esp_wifi_connect() == ESP_OK)
        {
            ESP_LOGI(TAG, "Connecting...");
        }
        else
        {
            ESP_LOGI(TAG, "STA failed to connect. Retrying in %d ms...", WIFI_RETRY_INTERVAL_MS);

            if (wifi_retry_timer != NULL)
            {
                xTimerStart(wifi_retry_timer, 0);
            }
        }
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        xEventGroupClearBits(modbus_event_group, INV_EVENT_WIFI_CONNECTED);
        // auto-reconnect
        ESP_LOGI(TAG, "STA disconnected. Retrying in %d ms...", WIFI_RETRY_INTERVAL_MS);

        if (wifi_retry_timer != NULL)
        {
            xTimerStart(wifi_retry_timer, 0);
        }
    }
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = event_data;

        wifi_config_t wifi_config;
        if (esp_wifi_get_config(WIFI_IF_STA, &wifi_config) == ESP_OK)
        {
            snprintf(wifi_ssid, sizeof(wifi_ssid), "%s", wifi_config.sta.ssid);
        }
        esp_ip4addr_ntoa(&event->ip_info.ip, wifi_ip, sizeof(wifi_ip));
        
        ESP_LOGI(TAG, "Connected to: \"%s\", IP: \"%s\"", wifi_ssid, wifi_ip);

        xEventGroupSetBits(modbus_event_group, INV_EVENT_WIFI_CONNECTED);

        if (wifi_retry_timer != NULL)
        {
            xTimerStop(wifi_retry_timer, 0);
        }

    }

    // Handle events for clients connecting to the ESP32 AP
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "Client connected to ESP32 AP, MAC: " MACSTR, MAC2STR(event->mac));
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "Client disconnected from ESP32 AP, MAC: " MACSTR, MAC2STR(event->mac));
    }
}

esp_err_t wifi_init(void)
{
    // Initialize device ID based on MAC address
    init_device_id();

    // Initialize Wi-Fi
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle);
    if (err != ESP_OK)
    {
        return err;
    }
    // Read saved wifi config
    size_t len = sizeof(wifi_ssid);
    nvs_get_str(my_handle, NVS_KEY_WIFI_SSID, wifi_ssid, &len);
    len = sizeof(wifi_password);
    nvs_get_str(my_handle, NVS_KEY_WIFI_PASS, wifi_password, &len);

    nvs_close(my_handle);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    esp_netif_set_hostname(sta_netif, device_id);
    esp_netif_set_hostname(ap_netif, device_id);

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {.sta = {0}};
    strlcpy((char *)wifi_config.sta.ssid, wifi_ssid, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, wifi_password, sizeof(wifi_config.sta.password));

    wifi_config_t ap_config = {
        .ap = {
            .channel = 0,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA2_PSK}};
    snprintf((char *)ap_config.ap.ssid, sizeof(ap_config.ap.ssid), "%s", device_id);
    snprintf((char *)ap_config.ap.password, sizeof(ap_config.ap.password), "%s", ap_password);
    ap_config.ap.ssid_len = strlen(device_id);

    // Create a timer for retrying Wi-Fi connection attempts
    wifi_retry_timer = xTimerCreate(
        "WifiRetryTimer",
        pdMS_TO_TICKS(WIFI_RETRY_INTERVAL_MS),
        pdFALSE,
        NULL,
        wifi_retry_timer_callback);

    // Start Wi-Fi in AP+STA mode if SSID is provided, otherwise start in AP-only mode
    if (strlen(wifi_ssid) == 0)
    {
        ESP_LOGW(TAG, "Wi-Fi SSID is empty! Running in AP-only server mode.");
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    }
    else
    {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    }
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_LOGI(TAG, "WiFi started in %s mode.", (strlen(wifi_ssid) == 0) ? "AP" : "AP+STA");
    return err;
}

static void wifi_retry_timer_callback(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG, "Retrying Wi-Fi STA connection...");
    esp_wifi_connect();
}
