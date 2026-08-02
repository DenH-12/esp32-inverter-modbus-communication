#include "haDiscovery.h"
#include <ctype.h>
#include <string.h>
#include "cJSON.h"

static const char *TAG = "haDiscovery";
extern char device_id[];
extern char device_id_lower[];
extern char mqtt_stats_topic[];
extern char ha_discovery_topic[];
extern char wifi_ip[];
extern uint16_t webserver_port;

ha_sys_sensor_t sys_sensors[] = {
    {"name",          "Name",                  NULL,               NULL,  false},
    {"version",       "Version",               NULL,               NULL,  false},
    {"ssid",          "SSID",                  NULL,               NULL,  false},
    {"ip",            "IP",                    NULL,               NULL,  false},
    {"rssi",          "RSSI",                  "signal_strength",  "dBm",  true},
    {"uptime",        "Uptime",                "duration",         "s",    true},
    {"reset_reason",  "Last Reset Reason",     NULL,               NULL,  false},
    {"free_heap",     "Free Heap",             "data_size",        "B",    true},
    {"min_free_heap", "Minimum Heap Free",     "data_size",        "B",    true},
};

size_t sys_sensors_count = sizeof(sys_sensors) / sizeof(sys_sensors[0]);

void ha_discovery_publish_device_info()
{
    char topic[256];
    char unique_id[64];
    char configuration_url[64];

    cJSON *root = cJSON_CreateObject();
    if (!root) return;

    cJSON *device = cJSON_CreateObject();
    cJSON *identifiers = cJSON_CreateArray();

    snprintf(unique_id, sizeof(unique_id), "%s_%s", device_id_lower, "device_info");
    snprintf(configuration_url, sizeof(configuration_url), "http://%s:%u", wifi_ip, webserver_port);
    snprintf(topic, sizeof(topic), "%s/binary_sensor/%s/config", ha_discovery_topic, unique_id);

    cJSON_AddStringToObject(root, "name", "Status");
    cJSON_AddStringToObject(root, "unique_id", unique_id);
    cJSON_AddStringToObject(root, "state_topic", mqtt_will_topic);
    cJSON_AddStringToObject(root, "payload_on", "online");
    cJSON_AddStringToObject(root, "payload_off", "offline");
    cJSON_AddStringToObject(root, "entity_category", "diagnostic");

    cJSON_AddItemToArray(identifiers, cJSON_CreateString(device_id));
    cJSON_AddItemToObject(device, "identifiers", identifiers);
    cJSON_AddStringToObject(device, "name", device_id);
    cJSON_AddStringToObject(device, "model", "ESP32 Gateway");
    cJSON_AddStringToObject(device, "manufacturer", "Custom ESP32");
    cJSON_AddStringToObject(device, "sw_version", VERSION);
    cJSON_AddStringToObject(device, "configuration_url", configuration_url);

    cJSON_AddItemToObject(root, "device", device);

    char *payload = cJSON_PrintUnformatted(root);
    if (payload)
    {
        mqtt_publish_topic(topic, payload, true); // retain true
        free(payload);
    }

    cJSON_Delete(root);
}

void ha_discovery_publish_sys_sensor(ha_sys_sensor_t *sensor)
{
    if (!sensor)
        return;

    char topic[256];
    char value_template[96] = {0};
    char unique_id[64];

    cJSON *root = cJSON_CreateObject();
    cJSON *device = cJSON_CreateObject();
    cJSON *identifiers = cJSON_CreateArray();

    snprintf(unique_id, sizeof(unique_id), "%s_%s", device_id_lower, sensor->name_id);
    snprintf(topic, sizeof(topic), "%s/sensor/%s/config", ha_discovery_topic, unique_id);
    snprintf(value_template, sizeof(value_template), "{{ value_json.%s }}", sensor->name_id);

    cJSON_AddStringToObject(root, "name", sensor->name);
    cJSON_AddStringToObject(root, "unique_id", unique_id);
    cJSON_AddStringToObject(root, "state_topic", sensor->realtime ? mqtt_system_realtime_topic : mqtt_system_topic);
    cJSON_AddStringToObject(root, "value_template", value_template);
    cJSON_AddStringToObject(root, "entity_category", "diagnostic");
    if (sensor->unit)
        cJSON_AddStringToObject(root, "unit_of_measurement", sensor->unit);
    if (sensor->device_class)
        cJSON_AddStringToObject(root, "device_class", sensor->device_class);

    cJSON_AddItemToArray(identifiers, cJSON_CreateString(device_id));
    cJSON_AddItemToObject(device, "identifiers", identifiers);
    cJSON_AddItemToObject(root, "device", device);

    char *payload = cJSON_PrintUnformatted(root);
    if (payload)
    {
        mqtt_publish_topic(topic, payload, true); // retain true
        free(payload);
    }

    cJSON_Delete(root);
}

void ha_discovery_publish_realtime(realtime_reg_t *reg)
{
    if (!reg) return;

    char topic[256];
    char value_template[96] = {0};
    char unique_id[64];

    cJSON *root = cJSON_CreateObject();
    cJSON *device = cJSON_CreateObject();
    cJSON *identifiers = cJSON_CreateArray();

    snprintf(unique_id, sizeof(unique_id), "%s_%s", device_id_lower, reg->id);
    snprintf(topic, sizeof(topic), "%s/sensor/%s/config", ha_discovery_topic, unique_id);
    snprintf(value_template, sizeof(value_template), "{{ value_json.%s }}", reg->id);

    cJSON_AddStringToObject(root, "name", reg->name);
    cJSON_AddStringToObject(root, "unique_id", unique_id);
    cJSON_AddStringToObject(root, "state_topic", mqtt_stats_topic);
    cJSON_AddStringToObject(root, "value_template", value_template);
    if (reg->unit)
        cJSON_AddStringToObject(root, "unit_of_measurement", reg->unit);
    if (reg->device_class)
        cJSON_AddStringToObject(root, "device_class", reg->device_class);

    cJSON_AddItemToArray(identifiers, cJSON_CreateString(device_id));
    cJSON_AddItemToObject(device, "identifiers", identifiers);
    cJSON_AddItemToObject(root, "device", device);

    char *payload = cJSON_PrintUnformatted(root);
    if (payload) {
        mqtt_publish_topic(topic, payload, true); // retain true
        free(payload);
    }
    
    cJSON_Delete(root);
}

void ha_discovery_publish_settings(settings_reg_t *reg)
{
    if (!reg)
        return;

    char topic[256];
    char command_topic[128] = {0};
    char value_template[256] = {0};
    char command_template[256] = {0};
    char unique_id[64];

    cJSON *root = cJSON_CreateObject();
    cJSON *device = cJSON_CreateObject();
    cJSON *identifiers = cJSON_CreateArray();
    cJSON *options_array = cJSON_CreateArray();

    snprintf(unique_id, sizeof(unique_id), "%s_%s", device_id_lower, reg->id);
    snprintf(topic, sizeof(topic), "%s/%s/%s/config", ha_discovery_topic, reg->type, unique_id);
    snprintf(command_topic, sizeof(command_topic), "%s/%s", mqtt_cmnd_topic, reg->id);

    if (strcmp(reg->type, "number") == 0) {
        snprintf(value_template, sizeof(value_template), "{{ value_json.%s }}", reg->id);
    }
    else if (strcmp(reg->type, "select") == 0)
    {
        // Filling value_template and command_template
        if (reg->options != NULL)
        {
            char val_map_buf[256] = "{";
            char cmd_map_buf[256] = "{";
            bool first = true;

            for (const ha_option_t *m = reg->options; m->label != NULL; m++)
            {
                // Fill options array
                cJSON_AddItemToArray(options_array, cJSON_CreateString(m->label));

                // Create Jinja2 mapping for value_template: {'0': 'Off', '1': 'On'}
                // and for command_template: {'Off': '0', 'On': '1'}
                char val_pair[64], cmd_pair[64];
                snprintf(val_pair, sizeof(val_pair), "%s'%s': '%s'", first ? "" : ", ", m->val, m->label);
                snprintf(cmd_pair, sizeof(cmd_pair), "%s'%s': '%s'", first ? "" : ", ", m->label, m->val);

                strlcat(val_map_buf, val_pair, sizeof(val_map_buf));
                strlcat(cmd_map_buf, cmd_pair, sizeof(cmd_map_buf));
                first = false;
            }
            strlcat(val_map_buf, "}", sizeof(val_map_buf));
            strlcat(cmd_map_buf, "}", sizeof(cmd_map_buf));

            snprintf(value_template, sizeof(value_template), "{{ %s.get(value_json.%s | string, 'Unknown') }}", val_map_buf, reg->id);
            snprintf(command_template, sizeof(command_template), "{{ %s.get(value, '%s') }}", cmd_map_buf, reg->options[0].val);
        }
        else
        {
            // Default
            snprintf(value_template, sizeof(value_template), "{{ value_json.%s }}", reg->id);
            snprintf(command_template, sizeof(command_template), "{{ value }}");
        }
    }

    cJSON_AddStringToObject(root, "name", reg->name);
    cJSON_AddStringToObject(root, "unique_id", unique_id);
    cJSON_AddStringToObject(root, "state_topic", mqtt_settings_topic);
    cJSON_AddStringToObject(root, "command_topic", command_topic);
    cJSON_AddStringToObject(root, "value_template", value_template);
    
    if (strcmp(reg->type, "number") == 0) {
        cJSON_AddStringToObject(root, "mode", "box");
        if (reg->unit)
            cJSON_AddStringToObject(root, "unit_of_measurement", reg->unit);
    }
    else if (strcmp(reg->type, "select") == 0)
    {
        cJSON_AddStringToObject(root, "command_template", command_template);
        cJSON_AddItemToObject(root, "options", options_array);
    }
    cJSON_AddItemToArray(identifiers, cJSON_CreateString(device_id));
    cJSON_AddItemToObject(device, "identifiers", identifiers);
    cJSON_AddItemToObject(root, "device", device);

    char *payload = cJSON_PrintUnformatted(root);
    if (payload)
    {
        mqtt_publish_topic(topic, payload, true); // retain true
        free(payload);
    }

    cJSON_Delete(root);
}

void ha_discovery_publish_buttons() {
    char topic[256];
    char unique_id[64];
    char command_topic[128] = {0};

    char *buttons[] = {"refresh_settings", "system_reset"};

    for (size_t i = 0; i < sizeof(buttons) / sizeof(buttons[0]); i++) {
        snprintf(unique_id, sizeof(unique_id), "%s_%s", device_id_lower, buttons[i]);
        snprintf(topic, sizeof(topic), "%s/button/%s/config", ha_discovery_topic, unique_id);
        snprintf(command_topic, sizeof(command_topic), "%s/%s", mqtt_cmnd_topic, buttons[i]);

        cJSON *root = cJSON_CreateObject();
        if (!root) return;

        cJSON *device = cJSON_CreateObject();
        cJSON *identifiers = cJSON_CreateArray();

        cJSON_AddStringToObject(root, "name", buttons[i]);
        cJSON_AddStringToObject(root, "unique_id", unique_id);
        cJSON_AddStringToObject(root, "command_topic", command_topic);
        cJSON_AddStringToObject(root, "payload_press", "1");

        cJSON_AddItemToArray(identifiers, cJSON_CreateString(device_id));
        cJSON_AddItemToObject(device, "identifiers", identifiers);

        cJSON_AddItemToObject(root, "device", device);

        char *payload = cJSON_PrintUnformatted(root);
        if (payload) {
            mqtt_publish_topic(topic, payload, true); // retain true
            free(payload);
        }

        cJSON_Delete(root);
    }
}
