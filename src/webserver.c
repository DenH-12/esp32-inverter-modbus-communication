#include "webserver.h"

const char *TAG = "WEB";

// Assign URI handlers to specific URIs and methods
static httpd_uri_t config_post_uri = {
    .uri = "/api/config",
    .method = HTTP_POST,
    .handler = config_post_handler,
    .user_ctx = NULL};

static httpd_uri_t config_get_uri = {
    .uri = "/api/config",
    .method = HTTP_GET,
    .handler = config_get_handler,
    .user_ctx = NULL};

static httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_get_handler};

static httpd_uri_t reboot_uri = {
    .uri = "/api/reboot",
    .method = HTTP_POST,
    .handler = reboot_post_handler,
    .user_ctx = NULL};

httpd_handle_t server = NULL;
httpd_handle_t webserver_init(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_register_uri_handler(server, &config_post_uri);
        httpd_register_uri_handler(server, &config_get_uri);
        httpd_register_uri_handler(server, &index_uri);
        httpd_register_uri_handler(server, &reboot_uri);
    }
    ESP_LOGI(TAG, "Webserver started on port %d", config.server_port);
    return server;
}

esp_err_t webserver_stop(void)
{
    if (server != NULL)
    {
        httpd_stop(server);
        server = NULL;
        ESP_LOGI(TAG, "Webserver stopped");
    }
    return ESP_OK;
}

esp_err_t config_post_handler(httpd_req_t *req)
{
    // Read the incoming JSON data from the request
    char content[500] = {0};

    size_t recv_size = (req->content_len < sizeof(content) - 1) ? req->content_len : sizeof(content) - 1;
    httpd_req_recv(req, content, recv_size);

    ESP_LOGI(TAG, "Frontend sent: %s", content);

    cJSON *root = cJSON_Parse(content);
    if (root == NULL)
    {
        ESP_LOGE(TAG, "Failed to parse JSON!");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    // Save the received configuration to NVS
    cJSON *ssid = cJSON_GetObjectItem(root, "wifiSsid");
    if (cJSON_IsString(ssid) && (ssid->valuestring != NULL))
    {
        nvs_save_str("wifi_ssid", ssid->valuestring);
    }
    cJSON *password = cJSON_GetObjectItem(root, "wifiPassword");
    if (cJSON_IsString(password) && (password->valuestring != NULL))
    {
        nvs_save_str("wifi_password", password->valuestring);
    }
    cJSON *mqtt_broker = cJSON_GetObjectItem(root, "mqttBroker");
    if (cJSON_IsString(mqtt_broker) && (mqtt_broker->valuestring != NULL))
    {
        nvs_save_str("mqtt_broker", mqtt_broker->valuestring);
    }
    cJSON *mqtt_port = cJSON_GetObjectItem(root, "mqttPort");
    if (mqtt_port != NULL)
    {
        int32_t port_value = (int32_t)atoi(mqtt_port->valuestring);

        nvs_save_int("mqtt_port", port_value);
    }
    cJSON *mqtt_username = cJSON_GetObjectItem(root, "mqttUsername");
    if (cJSON_IsString(mqtt_username) && (mqtt_username->valuestring != NULL))
    {
        nvs_save_str("mqtt_username", mqtt_username->valuestring);
    }
    cJSON *mqtt_password = cJSON_GetObjectItem(root, "mqttPassword");
    if (cJSON_IsString(mqtt_password) && (mqtt_password->valuestring != NULL))
    {
        nvs_save_str("mqtt_password", mqtt_password->valuestring);
    }

    // Clean up the cJSON object
    cJSON_Delete(root);

    // Send a success response back to the frontend
    httpd_resp_set_type(req, "application/json");
    const char *response = "{\"status\":\"ok\"}";
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

esp_err_t config_get_handler(httpd_req_t *req)
{
    // Read the configuration from NVS
    char ssid[32] = {0};
    char password[64] = {0};
    char mqtt_broker[64] = {0};
    int32_t mqtt_port = 1883;
    char mqtt_username[32] = {0};
    char mqtt_password[32] = {0};

    nvs_read_str("wifi_ssid", ssid, sizeof(ssid));
    nvs_read_str("wifi_password", password, sizeof(password));
    nvs_read_str("mqtt_broker", mqtt_broker, sizeof(mqtt_broker));
    nvs_read_int("mqtt_port", &mqtt_port);
    nvs_read_str("mqtt_username", mqtt_username, sizeof(mqtt_username));
    nvs_read_str("mqtt_password", mqtt_password, sizeof(mqtt_password));

    // Get current Wi-Fi status and IP address
    char current_ip[16] = "0.0.0.0";
    char current_status[32] = "Disconnected";
    int rssi = 0;

    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
    {
        rssi = ap_info.rssi;
        strncpy(current_status, (char *)ap_info.ssid, sizeof(current_status));

        esp_netif_ip_info_t ip_info;
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK)
        {
            esp_ip4addr_ntoa(&ip_info.ip, current_ip, sizeof(current_ip));
        }
    }

    // Create a JSON object to send back to the frontend
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "wifiSsid", ssid);
    cJSON_AddStringToObject(root, "wifiPassword", password);
    cJSON_AddStringToObject(root, "mqttBroker", mqtt_broker);
    cJSON_AddNumberToObject(root, "mqttPort", mqtt_port);
    cJSON_AddStringToObject(root, "mqttUsername", mqtt_username);
    cJSON_AddStringToObject(root, "mqttPassword", mqtt_password);

    cJSON_AddStringToObject(root, "currentWifi", current_status);
    cJSON_AddStringToObject(root, "currentIp", current_ip);
    bool mqtt_connected = (xEventGroupGetBits(inverter_event_group) & INV_EVENT_MQTT_CONNECTED);
    cJSON_AddStringToObject(root, "currentMqtt", mqtt_connected ? "Connected" : "Disconnected");
    cJSON_AddNumberToObject(root, "currentRssi", rssi);

    char *response = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);

    free(response);
    return ESP_OK;
}

esp_err_t index_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t reboot_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Reboot requested via web interface.");
    httpd_resp_set_type(req, "application/json");
    const char *response = "{\"status\":\"rebooting\"}";
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);

    // Delay to ensure the response is sent before rebooting
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return ESP_OK; // This line will not be reached due to the restart
}