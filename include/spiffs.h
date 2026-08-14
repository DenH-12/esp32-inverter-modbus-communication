#pragma once

#include "esp_spiffs.h"
#include "esp_log.h"

#define PATH_REGISTERS_CONFIG "/spiffs/registers_map.json"
#define PATH_INDEX "/spiffs/index.html"

// Struct for select options
typedef struct
{
    const char *val;
    const char *label;
} ha_option_t;

// Real-time Register
typedef struct
{
    const char *id;
    uint16_t reg;
    float factor;
    bool is_signed;
    const char *name;
    const char *device_class;
    const char *unit;
} realtime_reg_t;

// Settings Register
typedef struct
{
    const char *id;
    uint16_t reg;
    float factor;
    const char *name;
    const char *unit;
    const char *type;
    const ha_option_t *options;
} settings_reg_t;

extern realtime_reg_t *realtime_map;
extern settings_reg_t *settings_map;
extern size_t realtime_map_count;
extern size_t settings_map_count;

void init_spiffs(void);
void load_register_config(void);