#include "spiffs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cJSON.h"

static const char *TAG = "SPIFFS";

realtime_reg_t *realtime_map = NULL;
settings_reg_t *settings_map = NULL;

size_t realtime_map_count = 0;
size_t settings_map_count = 0;

static char *dup_string(const char *value)
{
    if (!value)
        return NULL;

    size_t len = strlen(value) + 1;
    char *copy = malloc(len);
    if (!copy)
        return NULL;

    memcpy(copy, value, len);
    return copy;
}

static void free_register_maps(void)
{
    if (realtime_map)
    {
        for (size_t i = 0; i < realtime_map_count; i++)
        {
            free((void *)realtime_map[i].id);
            free((void *)realtime_map[i].name);
            free((void *)realtime_map[i].device_class);
            free((void *)realtime_map[i].unit);
        }
        free(realtime_map);
        realtime_map = NULL;
    }

    if (settings_map)
    {
        for (size_t i = 0; i < settings_map_count; i++)
        {
            free((void *)settings_map[i].id);
            free((void *)settings_map[i].name);
            free((void *)settings_map[i].unit);
            free((void *)settings_map[i].type);

            if (settings_map[i].options)
            {
                for (size_t j = 0; settings_map[i].options[j].val != NULL || settings_map[i].options[j].label != NULL; j++)
                {
                    free((void *)settings_map[i].options[j].val);
                    free((void *)settings_map[i].options[j].label);
                }
                free((void *)settings_map[i].options);
            }
        }
        free(settings_map);
        settings_map = NULL;
    }

    realtime_map_count = 0;
    settings_map_count = 0;
}

static bool get_json_bool(const cJSON *item, bool fallback)
{
    if (cJSON_IsBool(item))
        return cJSON_IsTrue(item);

    if (cJSON_IsNumber(item))
        return item->valuedouble != 0.0;

    return fallback;
}

static const char *get_string_value(const cJSON *item, const char *fallback)
{
    if (cJSON_IsString(item) && item->valuestring)
        return item->valuestring;

    return fallback;
}

void load_register_config(void)
{
    FILE *file = fopen(PATH_REGISTERS_CONFIG, "r");
    if (!file)
    {
        ESP_LOGE(TAG, "Could not open %s", PATH_REGISTERS_CONFIG);
        return;
    }

    if (fseek(file, 0, SEEK_END) != 0)
    {
        fclose(file);
        ESP_LOGE(TAG, "Failed to seek %s", PATH_REGISTERS_CONFIG);
        return;
    }

    long size = ftell(file);
    if (size < 0)
    {
        fclose(file);
        ESP_LOGE(TAG, "Failed to read size of %s", PATH_REGISTERS_CONFIG);
        return;
    }

    rewind(file);

    char *buffer = malloc((size_t)size + 1);
    if (!buffer)
    {
        fclose(file);
        ESP_LOGE(TAG, "Out of memory while loading register config");
        return;
    }

    size_t read_bytes = fread(buffer, 1, (size_t)size, file);
    buffer[read_bytes] = '\0';
    fclose(file);

    cJSON *root = cJSON_Parse(buffer);
    free(buffer);

    if (!root)
    {
        ESP_LOGE(TAG, "Failed to parse register JSON");
        return;
    }

    cJSON *realtime_array = cJSON_GetObjectItemCaseSensitive(root, "realtime_map");
    cJSON *settings_array = cJSON_GetObjectItemCaseSensitive(root, "settings_map");
    if (!cJSON_IsArray(realtime_array) || !cJSON_IsArray(settings_array))
    {
        cJSON_Delete(root);
        ESP_LOGE(TAG, "registers.json is missing realtime_map or settings_map");
        return;
    }

    free_register_maps();

    realtime_map_count = cJSON_GetArraySize(realtime_array);
    realtime_map = calloc(realtime_map_count, sizeof(*realtime_map));
    if (realtime_map_count > 0 && !realtime_map)
    {
        cJSON_Delete(root);
        ESP_LOGE(TAG, "Out of memory for realtime registers");
        return;
    }

    settings_map_count = cJSON_GetArraySize(settings_array);
    settings_map = calloc(settings_map_count, sizeof(*settings_map));
    if (settings_map_count > 0 && !settings_map)
    {
        free_register_maps();
        cJSON_Delete(root);
        ESP_LOGE(TAG, "Out of memory for settings registers");
        return;
    }

    if (realtime_map && realtime_map_count > 0)
    {
        for (size_t i = 0; i < realtime_map_count; i++)
        {
            cJSON *entry = cJSON_GetArrayItem(realtime_array, i);
            if (!cJSON_IsObject(entry))
                continue;

            cJSON *id_item = cJSON_GetObjectItemCaseSensitive(entry, "id");
            cJSON *reg_item = cJSON_GetObjectItemCaseSensitive(entry, "reg");
            cJSON *factor_item = cJSON_GetObjectItemCaseSensitive(entry, "factor");
            cJSON *signed_item = cJSON_GetObjectItemCaseSensitive(entry, "signed");
            cJSON *name_item = cJSON_GetObjectItemCaseSensitive(entry, "name");
            cJSON *class_item = cJSON_GetObjectItemCaseSensitive(entry, "class");
            cJSON *unit_item = cJSON_GetObjectItemCaseSensitive(entry, "unit");

            realtime_map[i].id = dup_string(get_string_value(id_item, ""));
            realtime_map[i].reg = (uint16_t)(cJSON_IsNumber(reg_item) ? (uint16_t)reg_item->valueint : 0);
            realtime_map[i].factor = cJSON_IsNumber(factor_item) ? (float)factor_item->valuedouble : 1.0f;
            realtime_map[i].is_signed = get_json_bool(signed_item, false);
            realtime_map[i].name = dup_string(get_string_value(name_item, ""));
            realtime_map[i].device_class = dup_string(get_string_value(class_item, NULL));
            realtime_map[i].unit = dup_string(get_string_value(unit_item, ""));
        }
    }

    if (settings_map && settings_map_count > 0)
    {
        for (size_t i = 0; i < settings_map_count; i++)
        {
            cJSON *entry = cJSON_GetArrayItem(settings_array, i);
            if (!cJSON_IsObject(entry))
                continue;

            cJSON *id_item = cJSON_GetObjectItemCaseSensitive(entry, "id");
            cJSON *reg_item = cJSON_GetObjectItemCaseSensitive(entry, "reg");
            cJSON *factor_item = cJSON_GetObjectItemCaseSensitive(entry, "factor");
            cJSON *name_item = cJSON_GetObjectItemCaseSensitive(entry, "name");
            cJSON *unit_item = cJSON_GetObjectItemCaseSensitive(entry, "unit");
            cJSON *type_item = cJSON_GetObjectItemCaseSensitive(entry, "type");
            cJSON *options_item = cJSON_GetObjectItemCaseSensitive(entry, "options");

            settings_map[i].id = dup_string(get_string_value(id_item, ""));
            settings_map[i].reg = (uint16_t)(cJSON_IsNumber(reg_item) ? (uint16_t)reg_item->valueint : 0);
            settings_map[i].factor = cJSON_IsNumber(factor_item) ? (float)factor_item->valuedouble : 1.0f;
            settings_map[i].name = dup_string(get_string_value(name_item, ""));
            settings_map[i].unit = dup_string(get_string_value(unit_item, ""));
            settings_map[i].type = dup_string(get_string_value(type_item, ""));

            if (cJSON_IsObject(options_item))
            {
                size_t option_count = 0;
                for (cJSON *child = options_item->child; child != NULL; child = child->next)
                    option_count++;

                ha_option_t *options = calloc(option_count + 1, sizeof(*options));
                if (options)
                {
                    size_t idx = 0;
                    for (cJSON *child = options_item->child; child != NULL; child = child->next)
                    {
                        if (cJSON_IsString(child))
                        {
                            options[idx].val = dup_string(child->string);
                            options[idx].label = dup_string(child->valuestring);
                            idx++;
                        }
                    }
                    settings_map[i].options = options;
                }
            }
        }
    }

    cJSON_Delete(root);
    ESP_LOGI(TAG, "Loaded %zu realtime and %zu settings registers", realtime_map_count, settings_map_count);
}

void init_spiffs(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "spiffs",
        .max_files = 5,
        .format_if_mount_failed = true};

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error mounting (%s)", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Mounted in /spiffs");
    load_register_config();
}