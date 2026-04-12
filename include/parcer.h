#pragma once

#include <stdio.h>
#include <stdint.h>
#include "mqtt.h"

void process_bms_package(uint8_t *data, int len);