#pragma once

#include "uart.h"

#define RS485_PORT_NUM UART_NUM_2
#define RS485_BAUD_RATE 9600
#define RS485_TX_PIN 18
#define RS485_RX_PIN 5
#define RS485_BUF_SIZE 1024

typedef struct {
    uint16_t voltage_limit;
    uint16_t discharge_limit;
    uint16_t charge_current;
    uint16_t discharge_current;
    uint16_t status;
} pylon_params_t;

void pylon_init();

void sniffer_task();

uint16_t calculate_pylon_checksum(const char* s);

void send_pylon_response_61(pylon_params_t *params, char* adr_str);

void send_pylon_response_63(char* adr_str);

void make_length(char *out, int len);