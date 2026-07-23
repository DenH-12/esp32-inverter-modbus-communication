#include "myNVS.h"

static const char *TAG = "NVS_CONFIG";

char ha_discovery_topic[128];
uint8_t data_read_interval_s = 0;
uint8_t sys_data_send_interval_s = 0;
uint16_t webserver_port = 0;
uint16_t mqtt_port = 0;

size_t ha_discovery_topic_len = sizeof(ha_discovery_topic);

void nvs_init(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS partition is empty or corrupted! Erasing and retrying...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Check reset settings flag
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
        return;

    int8_t need_init = 1;
    err = nvs_get_i8(my_handle, NVS_KEY_NEED_INIT, &need_init);

    if (err == ESP_ERR_NVS_NOT_FOUND || need_init == 1)
    {
        ESP_LOGI(TAG, "Writing default settings...");
        nvs_reset_saved_settings();
    }

    // read saved data from NVS
    nvs_get_str(my_handle, NVS_DISCOVERY_TOPIC, ha_discovery_topic, &ha_discovery_topic_len);
    nvs_get_u8(my_handle, NVS_KEY_DATA_SEC, &data_read_interval_s);
    nvs_get_u8(my_handle, NVS_KEY_SYS_SEC, &sys_data_send_interval_s);
    nvs_get_u16(my_handle, NVS_KEY_WEB_PORT, &webserver_port);
    nvs_get_u16(my_handle, NVS_KEY_MQTT_PORT, &mqtt_port);

    nvs_close(my_handle);

    ESP_LOGI(TAG, "NVS Flash successfully initialized!");
}

esp_err_t nvs_reset_saved_settings()
{
    nvs_handle_t my_handle;
    esp_err_t err;

    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
        return err;

    err = nvs_erase_all(my_handle);
    if (err != ESP_OK)
    {
        nvs_close(my_handle);
        return err;
    }

    nvs_set_u16(my_handle, NVS_KEY_MQTT_PORT, 1883);
    nvs_set_u8(my_handle, NVS_KEY_DATA_SEC, 3);
    nvs_set_u8(my_handle, NVS_KEY_SYS_SEC, 5);
    nvs_set_u16(my_handle, NVS_KEY_WEB_PORT, 80);
    nvs_set_str(my_handle, NVS_DISCOVERY_TOPIC, "homeassistant");
    nvs_set_i8(my_handle, NVS_KEY_NEED_INIT, 0);

    err = nvs_commit(my_handle);
    nvs_close(my_handle);

    return err;
}