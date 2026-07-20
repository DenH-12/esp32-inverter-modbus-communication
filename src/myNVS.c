#include "myNVS.h"

static const char *NVS_TAG = "NVS_CONFIG";

void nvs_init(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW("NVS", "NVS partition is empty or corrupted! Erasing and retrying...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);
    ESP_LOGI("NVS", "NVS Flash successfully initialized!");
}

void nvs_save_str(const char *key, const char *value)
{
    nvs_handle_t my_handle;

    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK)
    {
        esp_err_t err = nvs_set_str(my_handle, key, value);
        if (err == ESP_OK)
        {
            nvs_commit(my_handle);
            ESP_LOGI(NVS_TAG, "Key '%s' saved: %s", key, value);
        }
        else
        {
            ESP_LOGE(NVS_TAG, "Failed to set string for key '%s'", key);
        }
        nvs_close(my_handle);
    }
    else
    {
        ESP_LOGE(NVS_TAG, "Error opening NVS handle!");
    }
}

void nvs_save_int(const char *key, int32_t value)
{
    nvs_handle_t my_handle;

    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK)
    {
        esp_err_t err = nvs_set_i32(my_handle, key, value);
        if (err == ESP_OK)
        {
            nvs_commit(my_handle);
            ESP_LOGI(NVS_TAG, "Key '%s' saved: %ld", key, value);
        }
        else
        {
            ESP_LOGE(NVS_TAG, "Failed to set int for key '%s'", key);
        }
        nvs_close(my_handle);
    }
    else
    {
        ESP_LOGE(NVS_TAG, "Error opening NVS handle!");
    }
}

esp_err_t nvs_read_str(const char *key, char *out_buffer, size_t max_size)
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);

    if (err != ESP_OK)
    {
        return err;
    }

    size_t required_size = max_size;
    err = nvs_get_str(my_handle, key, out_buffer, &required_size);
    nvs_close(my_handle);

    if (err != ESP_OK)
    {
        ESP_LOGW(NVS_TAG, "Key '%s' not found", key);
    }

    return err;
}

esp_err_t nvs_read_int(const char *key, int32_t *out_value)
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err != ESP_OK)
    {
        return err;
    }
    err = nvs_get_i32(my_handle, key, out_value);
    nvs_close(my_handle);

    if (err != ESP_OK)
    {
        ESP_LOGW(NVS_TAG, "Key '%s' not found", key);
    }

    return err;
}
