#pragma once

#include "driver/uart.h"
#include "driver/gpio.h" 
#include "mqtt.h"
#include "cJSON.h"
#include "spiffs.h"

#define SNIFFER_PORT_NUM UART_NUM_1
#define SNIFFER_BAUD_RATE 9600
#define SNIFFER_TX_PIN 17
#define SNIFFER_RX_PIN 16
#define SNIFFER_BUF_SIZE 1024

#define SNIFFER_MODBUS_ADDR 0x01
#define SNIFFER_OUTPUT_READINGS 0
#define SNIFFER_PUBLISH_MQTT 1

#define INV_EVENT_REFRESH_SETTINGS (1 << 0) // Set this bit to trigger a settings refresh
#define INV_EVENT_MQTT_CONNECTED (1 << 1)   // Set this bit when MQTT is connected, so we can publish immediately
#define INV_EVENT_WIFI_CONNECTED (1 << 2)   // Set this bit when Wi-Fi is connected, so we can publish immediately


// Structure to define Modbus blocks we want to read
typedef struct
{
    uint16_t start_addr;
    uint16_t count;
    const char *label;
} modbus_block_t;

typedef struct
{
    uint16_t reg;
    uint16_t value;
} inv_cmd_t;

typedef struct
{
    uint16_t start_addr;
    uint16_t end_addr;
    uint16_t reg_num_per_read;
} scan_args_t;

typedef struct
{
    bool is_running;
    bool is_done;
    uint16_t current_addr;
    uint16_t end_addr;
    cJSON *results_array;
    SemaphoreHandle_t mutex;
} scan_status_t;

extern EventGroupHandle_t modbus_event_group;
extern QueueHandle_t modbus_cmd_queue;

void modbus_init();
void scan_status_init(void);

void modbus_read_task(void *pvParameters);
void run_modbus_scan(const scan_args_t *args);
esp_err_t modbus_write_register(uint16_t reg_addr, uint16_t value);

uint16_t crc16_modbus(const uint8_t *data, uint16_t len);
esp_err_t send_modbus_request(uint16_t start_addr, uint16_t count);
void parse_modbus_data(uint16_t start_addr, uint8_t *data);
void modbus_read_block(const modbus_block_t *block);
