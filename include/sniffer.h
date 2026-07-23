#pragma once

#include "driver/uart.h"
#include "driver/gpio.h" 
#include "mqtt.h"

#define SNIFFER_PORT_NUM UART_NUM_1
#define SNIFFER_BAUD_RATE 9600
#define SNIFFER_TX_PIN 17
#define SNIFFER_RX_PIN 16
#define SNIFFER_BUF_SIZE 1024

#define SNIFFER_MODBUS_ADDR 0x01
#define SNIFFER_OUTPUT_READINGS 0
#define SNIFFER_PUBLISH_MQTT 1

#define INV_EVENT_REFRESH_SETTINGS (1 << 0) // Set this bit to trigger a settings refresh read from the inverter
#define INV_EVENT_MQTT_CONNECTED (1 << 1)   // Set this bit when MQTT is connected, so we can publish immediately
#define INV_EVENT_WIFI_CONNECTED (1 << 2)   // Set this bit when Wi-Fi is connected, so we can publish immediately

// sensors ids
typedef enum
{
    ID_GRID_VOLTAGE,
    ID_GRID_FREQUENCY,
    ID_GRID_POWER,
    ID_OUTPUT_VOLTAGE,
    ID_OUTPUT_FREQUENCY,
    ID_OUTPUT_POWER,
    ID_OUTPUT_CURRENT,
    ID_OUTPUT_VA,
    ID_BATTERY_VOLTAGE,
    ID_BATTERY_CURRENT,
    ID_BATTERY_POWER,
    ID_PV_VOLTAGE,
    ID_PV_CURRENT,
    ID_PV_POWER,
    ID_LOAD_PERCENT,
    ID_TEMP_DC,
    ID_TEMP_INV,
    ID_STATUS,
    ID_UPTIME_SECONDS,
    ID_LAST_RESET_REASON,
    ID_FREE_HEAP,
    ID_MIN_HEAP_FREE,
    ID_OUTPUT_PRIORITY,
    ID_INPUT_VOLTAGE_RANGE,
    ID_BUZZER_MODE,
    ID_BEEP_SOURCE_INTERRUPT,
    ID_BACKLIGHT,
    ID_RETURN_DEFAULT_DISPLAY,
    ID_POWER_SAVE_MODE,
    ID_OVERLOAD_RESTART,
    ID_OVERTEMP_RESTART,
    ID_OVERLOAD_BYPASS,
    ID_SET_OUTPUT_VOLTAGE,
    ID_SET_OUTPUT_FREQUENCY,
    ID_BATTERY_TYPE,
    ID_BATTERY_DC_PROTECT_VOLTAGE,
    ID_BATTERY_BULK_VOLTAGE,
    ID_BATTERY_FLOAT_VOLTAGE,
    ID_BATTERY_RECOVERY_VOLTAGE,
    ID_BATTERY_PROTECTION_VOLTAGE,
    ID_BATTERY_CUTOFF_VOLTAGE,
    ID_BATTERY_CV_FLOAT_TIME,
    ID_BATTERY_CHARGE_PRIORITY,
    ID_BATTERY_MAX_CHARGE_CURRENT,
    ID_BATTERY_MAX_AC_CHARGE_CURRENT,
    ID_BATTERY_EQUALIZATION_CHARGE_VOLTAGE,
    ID_BATTERY_EQUALIZATION_CHARGE_TIME,
    ID_BATTERY_EQUALIZATION_TIMEOUT,
    ID_BATTERY_EQUALIZATION_INTERVAL,
    ID_BATTERY_EQUALIZATION_MODE,
    ID_BATTERY_PROTECTION_SOC,
    ID_BATTERY_RECOVERY_SOC,
    ID_BATTERY_CUTOFF_SOC,
    IDS_COUNT
} sensor_id_t;

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
    // Block REAL_TIME_DATA (200+)
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

    // Block SETTINGS_DATA (300+)
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
    sensor_id_t id;
    inverter_reg_t reg;
    float multiplier;
    bool is_signed;
    const char *name_id;
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
extern const size_t inv_map_len;
extern const reg_metadata_t inv_map[];

void sniffer_init();

void sniffer_task();

void modbus_scan_task(void *arg);
void inverter_read_task(void *arg);
esp_err_t inverter_write_register(uint16_t reg_addr, uint16_t value);

uint16_t crc16_modbus(const uint8_t *data, uint16_t len);
esp_err_t send_modbus_request(uint16_t start_addr, uint16_t count);
void parse_inverter_data(uint16_t start_addr, uint8_t *data);
void inverter_read_block(const modbus_block_t *block);
