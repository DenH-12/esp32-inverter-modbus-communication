#pragma once

#include "mqtt.h"

typedef struct
{
    const char *name_id;
    const char *name;
    const char *device_class;
    const char *unit;
    bool realtime;
} ha_sys_sensor_t;

extern ha_sys_sensor_t sys_sensors[];
extern size_t sys_sensors_count;

void ha_discovery_publish_device_info();
void ha_discovery_publish_sys_sensor(ha_sys_sensor_t *sensor);
void ha_discovery_publish_realtime(realtime_reg_t *reg);
void ha_discovery_publish_settings(settings_reg_t *reg);
void ha_discovery_publish_buttons();