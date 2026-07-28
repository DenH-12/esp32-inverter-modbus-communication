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

ha_sensor_t sensors[] = {
    {ID_GRID_VOLTAGE,    "Grid Volts",   "voltage",        "V"  },
    {ID_GRID_FREQUENCY,  "Grid Freq",    "frequency",      "Hz" },
    {ID_GRID_POWER,      "Grid Power",   "power",          "W"  },
    {ID_OUTPUT_VOLTAGE,  "Out Volts",    "voltage",        "V"  },
    {ID_OUTPUT_FREQUENCY,"Out Freq",     "frequency",      "Hz" },
    {ID_OUTPUT_POWER,    "Out Power",    "power",          "W"  },
    {ID_OUTPUT_CURRENT,  "Out Curr",     "current",        "A"  },
    {ID_OUTPUT_VA,       "Out VA",       "apparent_power", "VA" },
    {ID_BATTERY_VOLTAGE, "Batt Volts",   "voltage",        "V"  },
    {ID_BATTERY_CURRENT, "Batt Curr",    "current",        "A"  },
    {ID_BATTERY_POWER,   "Batt Power",   "power",          "W"  },
    {ID_PV_VOLTAGE,      "PV Volts",     "voltage",        "V"  },
    {ID_PV_CURRENT,      "PV Curr",      "current",        "A"  },
    {ID_PV_POWER,        "PV Power",     "power",          "W"  },
    {ID_LOAD_PERCENT,    "Load Pct",     NULL,             "%"  },
    {ID_TEMP_DC,         "DC Temp",      "temperature",    "°C" },
    {ID_TEMP_INV,        "Inv Temp",     "temperature",    "°C" },
};

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

// ----------------- Mappers -----------------
// Mapper On/Off
static const ha_mapper_t map_on_off[] = {
    {"0", "Off"},
    {"1", "On"},
    {NULL, NULL} // End of list
};
// Mapper output priority
static const ha_mapper_t map_output_priority[] = {
    {"0", "UTI"},
    {"1", "SOL"},
    {"2", "SBU"},
    {"3", "SUB"},
    {"4", "SUF"},
    {NULL, NULL}
};
// Mapper output voltage range
static const ha_mapper_t map_input_voltage_range[] = {
    {"0", "Appliances"},
    {"1", "UPS"},
    {"2", "Generator"},
    {NULL, NULL}
};
// Mapper buzzer mode
static const ha_mapper_t map_buzzer_mode[] = {
    {"0", "Off"},
    {"1", "On"},
    {"2", "Mute when input source changes"},
    {"3", "Alarm Only"},
    {NULL, NULL}
};
// Mapper output voltage
static const ha_mapper_t map_output_voltage[] = {
    {"220", "220Vac"},
    {"230", "230Vac"},
    {"240", "240Vac"},
    {NULL, NULL}};
// Mapper output frequency
static const ha_mapper_t map_output_frequency[] = {
    {"50", "50Hz"},
    {"60", "60Hz"},
    {NULL, NULL}
};
// Mapper battery type
static const ha_mapper_t map_battery_type[] = {
    {"0", "AGM"},
    {"1", "Flooded"},
    {"2", "User"},
    {"3", "Li1"},
    {"4", "Li2"},
    {"5", "Li3"},
    {"6", "Li4"},
    {NULL, NULL}
};
// Mapper battery charge priority
static const ha_mapper_t map_battery_charge_priority[] = {
    {"0", "Utility First"},
    {"1", "Solar First"},
    {"2", "Solar&Utility"},
    {"3", "Solar Only"},
    {NULL, NULL}
};
// ----------------------------------
ha_select_t selects[] = {
    {ID_OUTPUT_PRIORITY,             "Output Priority",            map_output_priority},
    {ID_INPUT_VOLTAGE_RANGE,         "Input Voltage Range",        map_input_voltage_range},
    {ID_BUZZER_MODE,                 "Buzzer Mode",                map_buzzer_mode},
    {ID_BEEP_SOURCE_INTERRUPT,       "Beep Source Interrupt",      map_on_off},
    {ID_BACKLIGHT,                   "Backlight",                  map_on_off},
    {ID_RETURN_DEFAULT_DISPLAY,      "Return Default Display",     map_on_off},
    {ID_POWER_SAVE_MODE,             "Power Save Mode",            map_on_off},
    {ID_OVERLOAD_RESTART,            "Overload Restart",           map_on_off},
    {ID_OVERTEMP_RESTART,            "Overtemperature Restart",    map_on_off},
    {ID_OVERLOAD_BYPASS,             "Overload Bypass",            map_on_off},
    {ID_SET_OUTPUT_VOLTAGE,          "Output Voltage",             map_output_voltage},
    {ID_SET_OUTPUT_FREQUENCY,        "Output Frequency",           map_output_frequency},
    {ID_BATTERY_TYPE,                "Battery Type",               map_battery_type},
    {ID_BATTERY_CHARGE_PRIORITY,     "Battery Charge Priority",    map_battery_charge_priority},
    {ID_BATTERY_EQUALIZATION_MODE,   "Battery Equalization Mode",  map_on_off},
};

ha_number_t numbers[] = {
    {ID_BATTERY_DC_PROTECT_VOLTAGE,          "Batt DC Protect Volts",  "V"},
    {ID_BATTERY_BULK_VOLTAGE,                "Batt Bulk Volts",        "V"},
    {ID_BATTERY_FLOAT_VOLTAGE,               "Batt Float Volts",       "V"},
    {ID_BATTERY_RECOVERY_VOLTAGE,            "Batt Recovery Volts",    "V"},
    {ID_BATTERY_PROTECTION_VOLTAGE,          "Batt Protect Volts",     "V"},
    {ID_BATTERY_CUTOFF_VOLTAGE,              "Batt Cutoff Volts",      "V"},
    {ID_BATTERY_CV_FLOAT_TIME,               "Batt CV Float Time",     "s"},
    {ID_BATTERY_MAX_CHARGE_CURRENT,          "Batt Max Chg Current",   "A"},
    {ID_BATTERY_MAX_AC_CHARGE_CURRENT,       "Batt Max AC Chg Curr",   "A"},
    {ID_BATTERY_EQUALIZATION_CHARGE_VOLTAGE, "Batt Eq Chg Volts",      "V"},
    {ID_BATTERY_EQUALIZATION_CHARGE_TIME,    "Batt Eq Chg Time",       "s"},
    {ID_BATTERY_EQUALIZATION_TIMEOUT,        "Batt Eq Timeout",        "s"},
    {ID_BATTERY_EQUALIZATION_INTERVAL,       "Batt Eq Interval",       "s"},
    {ID_BATTERY_PROTECTION_SOC,              "Batt Protect SOC",       "%"},
    {ID_BATTERY_RECOVERY_SOC,                "Batt Recovery SOC",      "%"},
    {ID_BATTERY_CUTOFF_SOC,                  "Batt Cutoff SOC",        "%"},
};

size_t sensors_count = sizeof(sensors) / sizeof(sensors[0]);
size_t sys_sensors_count = sizeof(sys_sensors) / sizeof(sys_sensors[0]);
size_t numbers_count = sizeof(numbers) / sizeof(numbers[0]);
size_t selects_count = sizeof(selects) / sizeof(selects[0]);

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

void ha_discovery_publish_sensor(ha_sensor_t *sensor)
{
    if (!sensor) return;

    char topic[256];
    char value_template[96] = {0};
    char unique_id[64];

    cJSON *root = cJSON_CreateObject();
    cJSON *device = cJSON_CreateObject();
    cJSON *identifiers = cJSON_CreateArray();

    const reg_metadata_t *meta = get_sensor_by_id(sensor->id);
    if (!meta) return;
    snprintf(unique_id, sizeof(unique_id), "%s_%s", device_id_lower, meta->name_id);
    snprintf(topic, sizeof(topic), "%s/sensor/%s/config", ha_discovery_topic, unique_id);
    snprintf(value_template, sizeof(value_template), "{{ value_json.%s }}", meta->name_id);

    cJSON_AddStringToObject(root, "name", sensor->name);
    cJSON_AddStringToObject(root, "unique_id", unique_id);
    cJSON_AddStringToObject(root, "state_topic", mqtt_stats_topic);
    cJSON_AddStringToObject(root, "value_template", value_template);
    if (sensor->unit)
        cJSON_AddStringToObject(root, "unit_of_measurement", sensor->unit);
    if (sensor->device_class)
        cJSON_AddStringToObject(root, "device_class", sensor->device_class);

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

void ha_discovery_publish_number(ha_number_t *number)
{
    if (!number)
        return;

    char topic[256];
    char value_template[96] = {0};
    char command_topic[128];
    char unique_id[64];

    cJSON *root = cJSON_CreateObject();
    cJSON *device = cJSON_CreateObject();
    cJSON *identifiers = cJSON_CreateArray();

    const reg_metadata_t *meta = get_number_select_by_id(number->id);
    if (!meta)
        return;
    snprintf(unique_id, sizeof(unique_id), "%s_%s", device_id_lower, meta->name_id);
    snprintf(topic, sizeof(topic), "%s/number/%s/config", ha_discovery_topic, unique_id);
    snprintf(value_template, sizeof(value_template), "{{ value_json.%s }}", meta->name_id);
    snprintf(command_topic, sizeof(command_topic), "%s/%s", mqtt_cmnd_topic, meta->name_id);

    cJSON_AddStringToObject(root, "name", number->name);
    cJSON_AddStringToObject(root, "unique_id", unique_id);
    cJSON_AddStringToObject(root, "state_topic", mqtt_settings_topic);
    cJSON_AddStringToObject(root, "value_template", value_template);
    cJSON_AddStringToObject(root, "command_topic", command_topic);
    cJSON_AddStringToObject(root, "mode", "box");
    if (number->unit)
        cJSON_AddStringToObject(root, "unit_of_measurement", number->unit);

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

void ha_discovery_publish_select(ha_select_t *select)
{
    if (!select)
        return;

    char topic[256];
    char command_topic[128];
    char value_template[256];
    char command_template[256];
    char unique_id[64];

    cJSON *root = cJSON_CreateObject();
    cJSON *device = cJSON_CreateObject();
    cJSON *identifiers = cJSON_CreateArray();
    cJSON *options_array = cJSON_CreateArray();

    const reg_metadata_t *meta = get_number_select_by_id(select->id);
    if (!meta)
        return;
    snprintf(unique_id, sizeof(unique_id), "%s_%s", device_id_lower, meta->name_id);
    snprintf(topic, sizeof(topic), "%s/select/%s/config", ha_discovery_topic, unique_id);
    snprintf(command_topic, sizeof(command_topic), "%s/%s", mqtt_cmnd_topic, meta->name_id);

    // Filling value_template and command_template
    if (select->mappers != NULL)
    {
        char val_map_buf[256] = "{";
        char cmd_map_buf[256] = "{";
        bool first = true;

        for (const ha_mapper_t *m = select->mappers; m->key != NULL; m++)
        {
            // Fill options array
            cJSON_AddItemToArray(options_array, cJSON_CreateString(m->key));

            // Create Jinja2 mapping for value_template: {'0': 'Off', '1': 'On'}
            // and for command_template: {'Off': '0', 'On': '1'}
            char val_pair[64], cmd_pair[64];
            snprintf(val_pair, sizeof(val_pair), "%s'%s': '%s'", first ? "" : ", ", m->value, m->key);
            snprintf(cmd_pair, sizeof(cmd_pair), "%s'%s': '%s'", first ? "" : ", ", m->key, m->value);

            strlcat(val_map_buf, val_pair, sizeof(val_map_buf));
            strlcat(cmd_map_buf, cmd_pair, sizeof(cmd_map_buf));
            first = false;
        }
        strlcat(val_map_buf, "}", sizeof(val_map_buf));
        strlcat(cmd_map_buf, "}", sizeof(cmd_map_buf));

        snprintf(value_template, sizeof(value_template), "{{ %s.get(value_json.%s | string, 'Unknown') }}", val_map_buf, meta->name_id);
        snprintf(command_template, sizeof(command_template), "{{ %s.get(value, '%s') }}", cmd_map_buf, select->mappers[0].value);
    }
    else
    {
        // Default
        snprintf(value_template, sizeof(value_template), "{{ value_json.%s }}", meta->name_id);
        snprintf(command_template, sizeof(command_template), "{{ value }}");
    }

    cJSON_AddStringToObject(root, "name", select->name);
    cJSON_AddStringToObject(root, "unique_id", unique_id);
    cJSON_AddStringToObject(root, "state_topic", mqtt_settings_topic);
    cJSON_AddStringToObject(root, "command_topic", command_topic);
    cJSON_AddStringToObject(root, "value_template", value_template);
    cJSON_AddStringToObject(root, "command_template", command_template);
    cJSON_AddItemToObject(root, "options", options_array);

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

const reg_metadata_t *get_sensor_by_id(uint16_t id)
{
    for (size_t i = 0; i < inv_map_len; i++)
    {
        if (inv_map[i].id == id)
        {
            return &inv_map[i];
        }
    }
    ESP_LOGE(TAG, "Sensor ID %d not found", id);

    return NULL;
}

const reg_metadata_t *get_number_select_by_id(uint16_t id)
{
    for (size_t i = 0; i < inv_settings_map_len; i++)
    {
        if (inv_settings_map[i].id == id)
        {
            return &inv_settings_map[i];
        }
    }
    ESP_LOGE(TAG, "Sensor ID %d not found", id);

    return NULL;
}

