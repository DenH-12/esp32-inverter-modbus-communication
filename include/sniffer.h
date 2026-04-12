#pragma once

#include "uart.h"

#define SNIFFER_PORT_NUM UART_NUM_1
#define SNIFFER_BAUD_RATE 9600
#define SNIFFER_TX_PIN 17
#define SNIFFER_RX_PIN 16
#define SNIFFER_BUF_SIZE 1024

#define SNIFFER_MODBUS_ADDR 0x01
#define SNIFFER_OUTPUT_READINGS 0
#define SNIFFER_PUBLISH_MQTT 1
#define SNIFFER_READ_INTERVAL_MS 2000

#define INV_EVENT_REFRESH_SETTINGS (1 << 0) // Set this bit to trigger a settings refresh read from the inverter
#define INV_EVENT_MQTT_CONNECTED (1 << 1)   // Set this bit when MQTT is connected, so we can publish immediately

// Structure to define Modbus blocks we want to read
typedef struct
{
    uint16_t start_addr;
    uint16_t count;
    const char *label;
} modbus_block_t;

// Register addresses
typedef enum
{
    // Bлок REAL_TIME_DATA (200+)
    REG_GRID_VOLTAGE = 202,
    REG_GRID_FREQ = 203,
    REG_GRID_POWER = 204,
    REG_OUT_VOLTAGE = 205,
    REG_OUT_FREQ = 207,
    REG_OUT_POWER = 208,
    REG_OUT_CURRENT = 211,
    REG_OUT_VA = 213,
    REG_BATT_VOLTAGE = 215,
    REG_BATT_CURRENT = 216,
    REG_BATT_POWER = 217,
    REG_PV_VOLTAGE = 219,
    REG_PV_CURRENT = 220,
    REG_PV_POWER = 223,
    REG_LOAD_PCT = 225,
    REG_TEMP_DC = 226,
    REG_TEMP_INV = 227,

    // Bлок SETTINGS_DATA (300+)
    REG_SET_OUT_PRIORITY = 301,
    REG_SET_INPUT_V_RANGE = 302,
    REG_SET_BUZZER_MODE = 303,
    REG_SET_BEEP_SOURCE_INTRPT = 304,
    REG_SET_BACKLIGHT = 305,
    REG_SET_RETURN_DEFLT_DISPLAY = 306,
    REG_SET_PWR_SAVE_MODE = 307,
    REG_SET_OVERLOAD_RESTART = 308,
    REG_SET_OVERTEMP_RESTART = 309,
    REG_SET_OVERLOAD_BYPASS = 310,
    REG_SET_OUT_V = 320,
    REG_SET_OUT_F = 321,
    REG_SET_BATT_TYPE = 322,
    REG_SET_BATT_DC_PROTECT_V = 323,
    REG_SET_BATT_BULK_V = 324,
    REG_SET_BATT_FLOAT_V = 325,
    REG_SET_BATT_RECOVERY_V = 326,
    REG_SET_BATT_PROTECTION_V = 327,
    REG_SET_BATT_CUTOFF_V = 329,
    REG_SET_BATT_CV_FLOAT_TIME = 330,
    REG_SET_BATT_CHG_PRIORITY = 331,
    REG_SET_BATT_MAX_CHG_A = 332,
    REG_SET_BATT_MAX_AC_CHG_A = 333,
    REG_SET_BATT_EQ_CHG_V = 334,
    REG_SET_BATT_EQ_CHG_TIME = 335,
    REG_SET_BATT_EQ_TIMEOUT = 336,
    REG_SET_BATT_EQ_INTERVAL = 337,
    REG_SET_BATT_EQ_MODE = 338, // Not sure if this is correct
    REG_SET_BATT_PROTECTION_SOC = 341,
    REG_SET_BATT_RECOVERY_SOC = 342,
    REG_SET_BATT_CUTOFF_SOC = 343,

} inverter_reg_t;

typedef struct
{
    inverter_reg_t reg;
    const char *name;
    float multiplier;
    bool is_signed;
} reg_metadata_t;

typedef struct
{
    uint16_t reg;
    uint16_t value;
} inv_cmd_t;

extern EventGroupHandle_t inverter_event_group;
extern const reg_metadata_t inv_settings_map[];
extern QueueHandle_t inv_cmd_queue;
extern const size_t inv_settings_map_len;

void sniffer_init();

void sniffer_task();

void modbus_scan_task(void *arg);
void inverter_read_task(void *arg);
esp_err_t inverter_write_register(uint16_t reg_addr, uint16_t value);

uint16_t crc16_modbus(const uint8_t *data, uint16_t len);
esp_err_t send_modbus_request(uint16_t start_addr, uint16_t count);
void parse_inverter_data(uint16_t start_addr, uint8_t *data);
void inverter_read_block(const modbus_block_t *block);