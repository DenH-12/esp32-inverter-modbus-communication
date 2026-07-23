#pragma once

#include "mqtt.h"

typedef struct
{
    sensor_id_t id;
    const char *name;
    const char *device_class;
    const char *unit;
} ha_sensor_t;

typedef struct
{
    const char *name_id;
    const char *name;
    const char *device_class;
    const char *unit;
    bool realtime;
} ha_sys_sensor_t;

typedef struct
{
    const char *value;
    const char *key;
} ha_mapper_t;
typedef struct
{
    sensor_id_t id;
    const char *name;
    const ha_mapper_t *mappers;
} ha_select_t;

typedef struct
{
    sensor_id_t id;
    const char *name;
    const char *unit;
    int min;
    int max;
} ha_number_t;

extern ha_sensor_t sensors[];
extern ha_sys_sensor_t sys_sensors[];
extern ha_number_t numbers[];
extern ha_select_t selects[];
extern size_t sensors_count;
extern size_t sys_sensors_count;
extern size_t numbers_count;
extern size_t selects_count;

void ha_discovery_publish_device_info();
void ha_discovery_publish_sensor(ha_sensor_t *sensor);
void ha_discovery_publish_sys_sensor(ha_sys_sensor_t *sensor);
void ha_discovery_publish_number(ha_number_t *number);
void ha_discovery_publish_select(ha_select_t *select);

const reg_metadata_t *get_sensor_by_id(uint16_t id);
const reg_metadata_t *get_number_select_by_id(uint16_t id);
