#pragma once

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "parcer.h"
#include "pylontech.h"

void uart_init();

void tx_task();
void rx_task();