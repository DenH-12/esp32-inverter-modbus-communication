#include "webserver.h"
#include "modbus.h"
#include "myNVS.h"

const char *TAG = "WEB";

extern char ha_discovery_topic[];
extern uint16_t webserver_port;
extern uint16_t mqtt_port;
extern uint8_t data_read_interval_s;
extern uint8_t sys_data_send_interval_s;
extern QueueHandle_t scan_queue;
extern scan_status_t g_scan_status;

static esp_err_t index_get_handler(httpd_req_t *req);
static esp_err_t css_get_handler(httpd_req_t *req);
static esp_err_t js_get_handler(httpd_req_t *req);
static esp_err_t status_get_handler(httpd_req_t *req);
static esp_err_t settings_post_handler(httpd_req_t *req);
static esp_err_t settings_get_handler(httpd_req_t *req);
static esp_err_t reboot_post_handler(httpd_req_t *req);
static esp_err_t ha_discovery_post_handler(httpd_req_t *req);
static esp_err_t reset_post_handler(httpd_req_t *req);
static esp_err_t scan_registers_post_handler(httpd_req_t *req);
static esp_err_t scan_registers_get_handler(httpd_req_t *req);
static esp_err_t scan_registers_map_get_handler(httpd_req_t *req);
static esp_err_t scan_registers_map_post_handler(httpd_req_t *req);

// Assign URI handlers to specific URIs and methods
static httpd_uri_t index_uri =          { "/",                    HTTP_GET,    index_get_handler           };
static httpd_uri_t css_uri =            { "/style.css",           HTTP_GET,    css_get_handler             };
static httpd_uri_t js_uri =             { "/script.js",           HTTP_GET,    js_get_handler              };
static httpd_uri_t status_get_uri =     { "/api/status",          HTTP_GET,    status_get_handler          };
static httpd_uri_t settings_post_uri =  { "/api/settings",        HTTP_POST,   settings_post_handler       };
static httpd_uri_t settings_get_uri =   { "/api/settings",        HTTP_GET,    settings_get_handler        };
static httpd_uri_t reboot_uri =         { "/api/reboot",          HTTP_POST,   reboot_post_handler         };
static httpd_uri_t ha_discovery_uri =   { "/api/ha_discovery",    HTTP_POST,   ha_discovery_post_handler   };
static httpd_uri_t reset_post_uri =     { "/api/factory-reset",   HTTP_POST,   reset_post_handler          };
static httpd_uri_t scan_registers_post_uri = { "/api/scan-registers",  HTTP_POST,   scan_registers_post_handler };
static httpd_uri_t scan_registers_get_uri = { "/api/scan-registers",  HTTP_GET,   scan_registers_get_handler };
static httpd_uri_t scan_registers_map_post_uri = { "/api/registers-map", HTTP_POST, scan_registers_map_post_handler};
static httpd_uri_t scan_registers_map_get_uri = { "/api/registers-map", HTTP_GET, scan_registers_map_get_handler};

httpd_handle_t server = NULL;
httpd_handle_t webserver_init(void) 
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.server_port = webserver_port; // Set the server port
    config.lru_purge_enable = true;
    config.max_open_sockets = 5;
    config.max_uri_handlers = 15;

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_register_uri_handler(server, &index_uri);
        httpd_register_uri_handler(server, &css_uri);
        httpd_register_uri_handler(server, &js_uri);
        httpd_register_uri_handler(server, &status_get_uri);
        httpd_register_uri_handler(server, &settings_post_uri);
        httpd_register_uri_handler(server, &settings_get_uri);
        httpd_register_uri_handler(server, &reboot_uri);
        httpd_register_uri_handler(server, &ha_discovery_uri);
        httpd_register_uri_handler(server, &reset_post_uri);
        httpd_register_uri_handler(server, &scan_registers_post_uri);
        httpd_register_uri_handler(server, &scan_registers_get_uri);
        httpd_register_uri_handler(server, &scan_registers_map_get_uri);
        httpd_register_uri_handler(server, &scan_registers_map_post_uri);
    }
    ESP_LOGI(TAG, "Webserver started on port %d", config.server_port);
    return server;
}

static bool get_json_int(const cJSON *root, const char *key, int *out_val)
{
    cJSON *item = cJSON_GetObjectItem(root, key);
    if (!item)
        return false;

    if (cJSON_IsNumber(item))
    {
        *out_val = item->valueint;
        return true;
    }
    else if (cJSON_IsString(item) && item->valuestring != NULL)
    {
        *out_val = atoi(item->valuestring);
        return true;
    }
    return false;
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


static esp_err_t serve_file(httpd_req_t *req, const char *filepath, const char *content_type)
{
    FILE *file = fopen(filepath, "r");
    if (!file)
    {
        ESP_LOGE(TAG, "Could not open %s", filepath);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, content_type);

    char buffer[1024];
    size_t read_bytes;

    while ((read_bytes = fread(buffer, 1, sizeof(buffer), file)) > 0)
    {
        if (httpd_resp_send_chunk(req, buffer, read_bytes) != ESP_OK)
        {
            fclose(file);
            httpd_resp_sendstr_chunk(req, NULL);
            return ESP_FAIL;
        }
    }

    fclose(file);
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

static esp_err_t index_get_handler(httpd_req_t *req)
{
    return serve_file(req, "/spiffs/index.html", "text/html");
}

static esp_err_t css_get_handler(httpd_req_t *req)
{
    return serve_file(req, "/spiffs/style.css", "text/css");
}

static esp_err_t js_get_handler(httpd_req_t *req)
{
    return serve_file(req, "/spiffs/script.js", "application/javascript");
}

static esp_err_t status_get_handler(httpd_req_t *req) 
{

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
    cJSON_AddStringToObject(root, "currentWifi", current_status);
    cJSON_AddStringToObject(root, "currentIp", current_ip);
    bool mqtt_connected = (xEventGroupGetBits(modbus_event_group) & INV_EVENT_MQTT_CONNECTED);
    cJSON_AddStringToObject(root, "currentMqtt", mqtt_connected ? "Connected" : "Disconnected");
    cJSON_AddNumberToObject(root, "currentRssi", rssi);

    char *response = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);

    free(response);
    return ESP_OK;
}

static esp_err_t settings_post_handler(httpd_req_t *req)
{
    // Read the incoming JSON data from the request
    char content[1024] = {0};

    size_t recv_size = (req->content_len < sizeof(content) - 1) ? req->content_len : sizeof(content) - 1;

    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0)
    {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT)
        {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Frontend sent: %s", content);

    cJSON *root = cJSON_Parse(content);
    if (root == NULL)
    {
        ESP_LOGE(TAG, "Failed to parse JSON!");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    nvs_handle_t my_handle;

    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        cJSON_Delete(root);
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return err;
    }

    // Save the received configuration to NVS
    // Save Strings
    cJSON *ssid = cJSON_GetObjectItem(root, "wifiSsid");
    if (cJSON_IsString(ssid) && (ssid->valuestring != NULL)) {
        nvs_set_str(my_handle, NVS_KEY_WIFI_SSID, ssid->valuestring);
    }
    cJSON *password = cJSON_GetObjectItem(root, "wifiPassword");
    if (cJSON_IsString(password) && (password->valuestring != NULL)) {
        nvs_set_str(my_handle, NVS_KEY_WIFI_PASS, password->valuestring);
    }
    cJSON *mqtt_broker = cJSON_GetObjectItem(root, "mqttBroker");
    if (cJSON_IsString(mqtt_broker) && (mqtt_broker->valuestring != NULL)) {
        nvs_set_str(my_handle, NVS_KEY_MQTT_HOST, mqtt_broker->valuestring);
    }
    cJSON *mqtt_username = cJSON_GetObjectItem(root, "mqttUsername");
    if (cJSON_IsString(mqtt_username) && (mqtt_username->valuestring != NULL)) {
        nvs_set_str(my_handle, NVS_KEY_MQTT_USER, mqtt_username->valuestring);
    }
    cJSON *mqtt_password = cJSON_GetObjectItem(root, "mqttPassword");
    if (cJSON_IsString(mqtt_password) && (mqtt_password->valuestring != NULL)) {
        nvs_set_str(my_handle, NVS_KEY_MQTT_PASS, mqtt_password->valuestring);
    }
    // Save Integers
   int val = 0;
    if (get_json_int(root, "mqttPort", &val) && val > 0 && val <= 65535) {
        nvs_set_u16(my_handle, NVS_KEY_MQTT_PORT, (uint16_t)val);
    }

    if (get_json_int(root, "dataReadInterval", &val) && val >= 0 && val <= 255) {
        nvs_set_u8(my_handle, NVS_KEY_DATA_SEC, (uint8_t)val);
    }

    if (get_json_int(root, "sysDataSendInterval", &val) && val >= 0 && val <= 255) {
        nvs_set_u8(my_handle, NVS_KEY_SYS_SEC, (uint8_t)val);
    }

    if (get_json_int(root, "webServerPort", &val)) {
        if (val >= 80 && val <= 65535) {
            webserver_port = (uint16_t)val;
        } else {
            webserver_port = 80;
        }
        nvs_set_u16(my_handle, NVS_KEY_WEB_PORT, webserver_port);
    }

    nvs_commit(my_handle);
    nvs_close(my_handle);

    // Clean up the cJSON object
    cJSON_Delete(root);

    // Send a success response back to the frontend
    httpd_resp_set_type(req, "application/json");
    const char *response = "{\"status\":\"ok\"}";
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

static esp_err_t settings_get_handler(httpd_req_t *req)
{
    // Read the configuration from NVS
    char mqtt_broker[64] = {0};
    char mqtt_username[32] = {0};
    char mqtt_password[32] = {0};

    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS handle!");
        return err;
    }

    size_t len = sizeof(wifi_ssid);
    nvs_get_str(my_handle, NVS_KEY_WIFI_SSID, wifi_ssid, &len);
    len = sizeof(wifi_password);
    nvs_get_str(my_handle, NVS_KEY_WIFI_PASS, wifi_password, &len);
    len = sizeof(mqtt_broker);
    nvs_get_str(my_handle, NVS_KEY_MQTT_HOST, mqtt_broker, &len);
    nvs_get_u16(my_handle, NVS_KEY_MQTT_PORT, &mqtt_port);
    len = sizeof(mqtt_username);
    nvs_get_str(my_handle, NVS_KEY_MQTT_USER, mqtt_username, &len);
    len = sizeof(mqtt_password);
    nvs_get_str(my_handle, NVS_KEY_MQTT_PASS, mqtt_password, &len);
    len = sizeof(ha_discovery_topic);
    nvs_get_str(my_handle, NVS_DISCOVERY_TOPIC, ha_discovery_topic, &len);

    nvs_close(my_handle);

    // Create a JSON object to send back to the frontend
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "version", VERSION);
    cJSON_AddStringToObject(root, "wifiSsid", wifi_ssid);
    cJSON_AddStringToObject(root, "wifiPassword", wifi_password);
    cJSON_AddStringToObject(root, "mqttBroker", mqtt_broker);
    cJSON_AddNumberToObject(root, "mqttPort", mqtt_port);
    cJSON_AddStringToObject(root, "mqttUsername", mqtt_username);
    cJSON_AddStringToObject(root, "mqttPassword", mqtt_password);
    cJSON_AddStringToObject(root, "haDiscoveryTopic", ha_discovery_topic);
    cJSON_AddNumberToObject(root, "dataReadInterval", data_read_interval_s);
    cJSON_AddNumberToObject(root, "sysDataSendInterval", sys_data_send_interval_s);
    cJSON_AddNumberToObject(root, "webServerPort", webserver_port);

    char *response = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);

    free(response);
    return ESP_OK;
}

static esp_err_t reboot_post_handler(httpd_req_t *req)
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

static esp_err_t ha_discovery_post_handler(httpd_req_t *req)
{
    // Check MQTT connection before reading
    if (!(xEventGroupGetBits(modbus_event_group) & INV_EVENT_MQTT_CONNECTED))
    {
        ESP_LOGW(TAG, "Cannot run HA discovery: MQTT not connected");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "MQTT not connected");
        return ESP_FAIL;
    }

    // Read the incoming JSON data from the request
    char content[1024] = {0};

    size_t recv_size = (req->content_len < sizeof(content) - 1) ? req->content_len : sizeof(content) - 1;

    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0)
    {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT)
        {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }

    cJSON *root = cJSON_Parse(content);
    if (root == NULL)
    {
        ESP_LOGE(TAG, "Failed to parse JSON!");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Frontend requested HA discovery: %s", content);

    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS handle!");
        cJSON_Delete(root);
        return err;
    }

    // Save the received configuration to NVS
    cJSON *ha_discovery_topic_json = cJSON_GetObjectItem(root, "haDiscoveryTopic");
    if (cJSON_IsString(ha_discovery_topic_json) && (ha_discovery_topic_json->valuestring != NULL))
    {
        nvs_set_str(my_handle, NVS_DISCOVERY_TOPIC, ha_discovery_topic_json->valuestring);
        size_t len = sizeof(ha_discovery_topic);
        snprintf(ha_discovery_topic, len, "%s", ha_discovery_topic_json->valuestring);
    }
    nvs_commit(my_handle);

    nvs_close(my_handle);

    // Clean up the cJSON object
    cJSON_Delete(root);

    // Publishing HA Discovery configurations
    ha_discovery_publish_device_info();
    ha_discovery_publish_buttons();
    
    for (size_t i = 0; i < sys_sensors_count; i++)
    {
        ESP_LOGI(TAG, "Publishing HA discovery for: %s", sys_sensors[i].name);
        ha_discovery_publish_sys_sensor(&sys_sensors[i]);
    }

    for (size_t i = 0; i < realtime_map_count; i++)
    {
        ESP_LOGI(TAG, "Publishing HA discovery for: %s", realtime_map[i].name);
        ha_discovery_publish_realtime(&realtime_map[i]);
    }
    for (size_t i = 0; i < settings_map_count; i++)
    {
        ESP_LOGI(TAG, "Publishing HA discovery for: %s", settings_map[i].name);
        ha_discovery_publish_settings(&settings_map[i]);
    }

    // Send a success response back to the frontend
    httpd_resp_set_type(req, "application/json");
    const char *response = "{\"status\":\"ok\"}";
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

static esp_err_t reset_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Reset settings requested via web interface.");
    nvs_reset_saved_settings();
    
    httpd_resp_set_type(req, "application/json");
    const char *response = "{\"status\":\"ok\"}";
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);

    // Delay to ensure the response is sent before rebooting
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();

    return ESP_OK;
}

static esp_err_t scan_registers_post_handler(httpd_req_t *req)
{
    // Read the incoming JSON data from the request
    char content[512] = {0};

    size_t recv_size = (req->content_len < sizeof(content) - 1) ? req->content_len : sizeof(content) - 1;

    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0)
    {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT)
        {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }

    cJSON *root = cJSON_Parse(content);
    if (root == NULL)
    {
        ESP_LOGE(TAG, "Failed to parse JSON!");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Scan registers requested via web interface.");

    scan_args_t args;

    int val = 0;
    if (get_json_int(root, "startAddr", &val))
    {
        args.start_addr = (uint16_t)val;
    }
    if (get_json_int(root, "endAddr", &val))
    {
        args.end_addr = (uint16_t)val;
    }
    if (get_json_int(root, "scanStep", &val))
    {
        args.reg_num_per_read = (uint16_t)val;
    }

    cJSON_Delete(root);

    xSemaphoreTake(g_scan_status.mutex, portMAX_DELAY);
    if (g_scan_status.is_running)
    {
        xSemaphoreGive(g_scan_status.mutex);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Scan already in progress");
        return ESP_FAIL;
    }
    g_scan_status.is_running = true;
    g_scan_status.is_done = false;
    g_scan_status.current_addr = args.start_addr;
    g_scan_status.end_addr = args.end_addr;

    g_scan_status.results_array = cJSON_CreateArray();
    xSemaphoreGive(g_scan_status.mutex);

    if (xQueueSend(scan_queue, &args, 0) != pdTRUE)
    {
        xSemaphoreTake(g_scan_status.mutex, portMAX_DELAY);
        g_scan_status.is_running = false;
        xSemaphoreGive(g_scan_status.mutex);

        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_sendstr(req, "{\"status\":\"started\"}");
    return ESP_OK;
}

static esp_err_t scan_registers_get_handler(httpd_req_t *req)
{
    xSemaphoreTake(g_scan_status.mutex, portMAX_DELAY);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "running", g_scan_status.is_running);
    cJSON_AddBoolToObject(root, "done", g_scan_status.is_done);
    cJSON_AddNumberToObject(root, "current", g_scan_status.current_addr);
    cJSON_AddNumberToObject(root, "end", g_scan_status.end_addr);

    cJSON_AddItemToObject(root, "results", g_scan_status.results_array);
    g_scan_status.results_array = cJSON_CreateArray();

    xSemaphoreGive(g_scan_status.mutex);

    char *json_out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json_out);
    free(json_out);

    return ESP_OK;
}

static esp_err_t scan_registers_map_post_handler(httpd_req_t *req)
{
    FILE *file = fopen("/spiffs/registers_map.json", "w");
    if (!file)
    {
        ESP_LOGE(TAG, "Failed to open file for writing");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Read the incoming data in chunks and write to SPIFFS
    char buf[1024];
    int received = 0;
    int remaining = req->content_len;

    while (remaining > 0)
    {
        // Read a chunk of data from the request
        int read_bytes = httpd_req_recv(req, buf, (remaining < sizeof(buf)) ? remaining : sizeof(buf));

        if (read_bytes <= 0)
        {
            if (read_bytes == HTTPD_SOCK_ERR_TIMEOUT)
            {
                // If a timeout occurs, we can choose to continue reading or handle it as an error. Here, we will just continue to the next iteration.
                continue;
            }

            // Error in connection or interruption
            ESP_LOGE(TAG, "File receive failed/interrupted");
            fclose(file);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }

        // Write the received chunk to SPIFFS
        size_t written = fwrite(buf, 1, read_bytes, file);
        if (written != read_bytes)
        {
            ESP_LOGE(TAG, "Failed to write all bytes to SPIFFS");
            fclose(file);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }

        remaining -= read_bytes;
        received += read_bytes;
    }

    fclose(file);

    ESP_LOGI(TAG, "Registers map saved successfully. Received %d bytes.", received);

    // Send a success response back to the frontend
    httpd_resp_set_type(req, "application/json");
    const char *response = "{\"status\":\"ok\"}";
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

static esp_err_t scan_registers_map_get_handler(httpd_req_t *req)
{
    // Serve the registers_map.json file
    return serve_file(req, "/spiffs/registers_map.json", "application/json");
}