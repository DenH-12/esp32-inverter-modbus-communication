#include "sniffer.h"
#include "mqtt.h"

static const char *TAG = "INVERTER";

EventGroupHandle_t inverter_event_group;
QueueHandle_t inv_cmd_queue;
size_t free_heap;
size_t min_heap_free;

// Add blocks you want to read here
static const modbus_block_t blocks_to_read[] = {
    {200, 30, "REAL_TIME_DATA"},
    //{300, 51, "SETTINGS_DATA"},
};

const reg_metadata_t inv_map[] = {
    // Real-time data (Block 200+)
    {REG_GRID_VOLTAGE, "grid_voltage", 0.1f, false},
    {REG_GRID_FREQ, "grid_freq", 0.01f, false},
    {REG_GRID_POWER, "grid_power", 1.0f, false},
    {REG_OUT_VOLTAGE, "out_voltage", 0.1f, false},
    {REG_OUT_FREQ, "out_freq", 0.01f, false},
    {REG_OUT_POWER, "out_power", 1.0f, true},
    {REG_OUT_CURRENT, "out_current", 0.1f, false},
    {REG_OUT_VA, "out_va", 1.0f, false},
    {REG_BATT_VOLTAGE, "batt_voltage", 0.1f, false},
    {REG_BATT_CURRENT, "batt_current", 0.1f, true},
    {REG_BATT_POWER, "batt_power", 1.0f, true},
    {REG_PV_VOLTAGE, "pv_voltage", 0.1f, false},
    {REG_PV_CURRENT, "pv_current", 0.1f, false},
    {REG_PV_POWER, "pv_power", 1.0f, false},
    {REG_LOAD_PCT, "load_pct", 1.0f, false},
    {REG_TEMP_DC, "temp_dc", 1.0f, true},
    {REG_TEMP_INV, "temp_inv", 1.0f, true},
};

const reg_metadata_t inv_settings_map[] = {
    // Settings (Block 300+)
    {REG_SET_OUT_PRIORITY, "set_output_priority", 1.0f, false},
    {REG_SET_INPUT_V_RANGE, "set_input_voltage_range", 1.0f, false},
    {REG_SET_BUZZER_MODE, "set_buzzer_mode", 1.0f, false},
    {REG_SET_BEEP_SOURCE_INTRPT, "set_beep_source_intrpt", 1.0f, false},
    {REG_SET_BACKLIGHT, "set_backlight", 1.0f, false},
    {REG_SET_RETURN_DEFLT_DISPLAY, "set_return_default_display", 1.0f, false},
    {REG_SET_PWR_SAVE_MODE, "set_pwr_save_mode", 1.0f, false},
    {REG_SET_OVERLOAD_RESTART, "set_overload_restart", 1.0f, false},
    {REG_SET_OVERTEMP_RESTART, "set_overtemp_restart", 1.0f, false},
    {REG_SET_OVERLOAD_BYPASS, "set_overload_bypass", 1.0f, false},
    {REG_SET_OUT_V, "set_out_voltage", 0.1f, false},
    {REG_SET_OUT_F, "set_out_frequency", 0.01f, false},
    {REG_SET_BATT_TYPE, "set_batt_type", 1.0f, false},
    {REG_SET_BATT_DC_PROTECT_V, "set_batt_dc_protect_voltage", 0.1f, false},
    {REG_SET_BATT_BULK_V, "set_batt_bulk_voltage", 0.1f, false},
    {REG_SET_BATT_FLOAT_V, "set_batt_float_voltage", 0.1f, false},
    {REG_SET_BATT_RECOVERY_V, "set_batt_recovery_voltage", 0.1f, false},
    {REG_SET_BATT_PROTECTION_V, "set_batt_protection_voltage", 0.1f, false},
    {REG_SET_BATT_CUTOFF_V, "set_batt_cutoff_voltage", 0.1f, false},
    {REG_SET_BATT_CV_FLOAT_TIME, "set_batt_cv_float_time", 1.0f, false},
    {REG_SET_BATT_CHG_PRIORITY, "set_batt_chg_priority", 1.0f, false},
    {REG_SET_BATT_MAX_CHG_A, "set_batt_max_chg_a", 0.1f, false},
    {REG_SET_BATT_MAX_AC_CHG_A, "set_batt_max_ac_chg_current", 0.1f, false},
    {REG_SET_BATT_EQ_CHG_V, "set_batt_eq_chg_voltage", 0.1f, false},
    {REG_SET_BATT_EQ_CHG_TIME, "set_batt_eq_chg_time", 1.0f, false},
    {REG_SET_BATT_EQ_TIMEOUT, "set_batt_eq_timeout", 1.0f, false},
    {REG_SET_BATT_EQ_INTERVAL, "set_batt_eq_interval", 1.0f, false},
    {REG_SET_BATT_EQ_MODE, "set_batt_eq_mode", 1.0f, false},
    {REG_SET_BATT_PROTECTION_SOC, "set_batt_protection_soc", 1.0f, false},
    {REG_SET_BATT_RECOVERY_SOC, "set_batt_recovery_soc", 1.0f, false},
    {REG_SET_BATT_CUTOFF_SOC, "set_batt_cut_off_soc", 1.0f, false},
};
const size_t inv_settings_map_len = sizeof(inv_settings_map) / sizeof(inv_settings_map[0]);

void sniffer_init()
{
    inverter_event_group = xEventGroupCreate();
    xEventGroupSetBits(inverter_event_group, INV_EVENT_REFRESH_SETTINGS); // Trigger initial settings read on startup

    uart_config_t sniffer_config = {
        .baud_rate = SNIFFER_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(SNIFFER_PORT_NUM, &sniffer_config));
    ESP_ERROR_CHECK(uart_set_pin(SNIFFER_PORT_NUM, SNIFFER_TX_PIN, SNIFFER_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(SNIFFER_PORT_NUM, SNIFFER_BUF_SIZE, 0, 0, NULL, 0));

    inv_cmd_queue = xQueueCreate(10, sizeof(inv_cmd_t));
}

void inverter_read_task(void *pvParameters)
{
    const modbus_block_t settings_block = {300, 51, "SETTINGS_DATA"};
    inv_cmd_t cmd;

    while (1)
    {
        update_system_status();

        // Check MQTT connection before reading
        if (!(xEventGroupGetBits(inverter_event_group) & INV_EVENT_MQTT_CONNECTED))
        {
            ESP_LOGW(TAG, "MQTT not connected, skipping read cycle wait 5 seconds...");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        // Check if there are any pending commands to write before reading
        else if (xQueueReceive(inv_cmd_queue, &cmd, 0) == pdPASS)
        {
            inverter_write_register(cmd.reg, cmd.value);
            xEventGroupSetBits(inverter_event_group, INV_EVENT_REFRESH_SETTINGS); // Trigger a settings refresh after writing
            vTaskDelay(pdMS_TO_TICKS(100));                                       // Short delay after write before next action
        }

        for (int i = 0; i < sizeof(blocks_to_read) / sizeof(modbus_block_t); i++)
        {
            if (xEventGroupGetBits(inverter_event_group) & INV_EVENT_REFRESH_SETTINGS)
            {
                inverter_read_block(&settings_block);

                xEventGroupClearBits(inverter_event_group, INV_EVENT_REFRESH_SETTINGS);
                vTaskDelay(pdMS_TO_TICKS(100));
            }

            inverter_read_block(&blocks_to_read[i]);

            vTaskDelay(pdMS_TO_TICKS(100)); // Short delay between block reads
        }
        vTaskDelay(pdMS_TO_TICKS(SNIFFER_READ_INTERVAL_MS)); // Delay before next full read cycle
    }
}


void parse_inverter_data(uint16_t start_addr, uint8_t *data)
{
    if (start_addr == 200)
    {
#define GET_REG(addr) ((data[(addr - 200) * 2] << 8) | data[(addr - 200) * 2 + 1])
        if (SNIFFER_PUBLISH_MQTT)
        {
            char json_string[512];
            json_string[0] = '{';  // Start JSON
            json_string[1] = '\0'; // set terminator, so strncat works

            for (size_t i = 0; i < sizeof(inv_map) / sizeof(inv_map[0]); i++)
            {
                const reg_metadata_t *meta = &inv_map[i];
                float value = 0.0f;
                if (meta->is_signed)
                {
                    value = (int16_t)(GET_REG(meta->reg)) * meta->multiplier;
                }
                else
                {
                    value = GET_REG(meta->reg) * meta->multiplier;
                }
                char line[64];
                snprintf(line, sizeof(line), "\"%s\": %g,", meta->name, value);
                strncat(json_string, line, sizeof(json_string) - strlen(json_string) - 1);
            }
            // Remove trailing comma
            if (strlen(json_string) > 0)
            {
                json_string[strlen(json_string) - 1] = '\0';
            }
            strncat(json_string, "}", sizeof(json_string) - strlen(json_string) - 1); // End JSON

            mqtt_publish_topic(MQTT_REALTIME_TOPIC, json_string);
        }
    }
    else if (start_addr == 300)
    {
#define GET_REG_300(addr) ((data[(addr - 300) * 2] << 8) | data[(addr - 300) * 2 + 1])
        if (SNIFFER_PUBLISH_MQTT)
        {
            static char json_string[2048];
            json_string[0] = '{';  // Start JSON
            json_string[1] = '\0'; // set terminator, so strncat works

            for (size_t i = 0; i < sizeof(inv_settings_map) / sizeof(inv_settings_map[0]); i++)
            {
                const reg_metadata_t *meta = &inv_settings_map[i];
                float value = 0.0f;
                if (meta->is_signed)
                {
                    value = (int16_t)(GET_REG_300(meta->reg)) * meta->multiplier;
                }
                else
                {
                    value = GET_REG_300(meta->reg) * meta->multiplier;
                }
                char line[64];
                snprintf(line, sizeof(line), "\"%s\": %g,", meta->name, value);
                strncat(json_string, line, sizeof(json_string) - strlen(json_string) - 1);
            }
            // Remove trailing comma
            if (strlen(json_string) > 0)
            {
                json_string[strlen(json_string) - 1] = '\0';
            }
            strncat(json_string, "}", sizeof(json_string) - strlen(json_string) - 1); // End JSON

            mqtt_publish_topic(MQTT_SETTINGS_TOPIC, json_string);
        }
    }
}

void inverter_read_block(const modbus_block_t *block)
{
    uint8_t rx_buf[256];
    uint8_t tx_pkt[8] = {SNIFFER_MODBUS_ADDR, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    // Prepare Modbus request
    tx_pkt[2] = (block->start_addr >> 8) & 0xFF;
    tx_pkt[3] = block->start_addr & 0xFF;
    tx_pkt[4] = (block->count >> 8) & 0xFF;
    tx_pkt[5] = block->count & 0xFF;

    uint16_t crc = crc16_modbus(tx_pkt, 6);
    tx_pkt[6] = crc & 0xFF;
    tx_pkt[7] = (crc >> 8) & 0xFF;

    // Send request
    uart_flush(SNIFFER_PORT_NUM);
    uart_write_bytes(SNIFFER_PORT_NUM, (const char *)tx_pkt, 8);

    // Read response (expecting: SlaveID(1) + Func(1) + ByteCount(1) + Data(N*2) + CRC(2))
    int expected_len = 3 + (block->count * 2) + 2;
    int len = uart_read_bytes(SNIFFER_PORT_NUM, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(500));

    if (len >= expected_len && rx_buf[0] == SNIFFER_MODBUS_ADDR && rx_buf[1] == 0x03)
    {
        parse_inverter_data(block->start_addr, &rx_buf[3]);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to read block %s", block->label);
    }
}

uint16_t crc16_modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i];
        for (uint16_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

esp_err_t inverter_write_register(uint16_t reg_addr, uint16_t value)
{
    uint8_t tx_pkt[11];
    tx_pkt[0] = SNIFFER_MODBUS_ADDR; // Slave ID
    tx_pkt[1] = 0x10;                // Function: Write Register
    tx_pkt[2] = (reg_addr >> 8);     // Address High
    tx_pkt[3] = (reg_addr & 0xFF);   // Address Low
    tx_pkt[4] = 0x00;                // Number of registers high (1 register)
    tx_pkt[5] = 0x01;                // Number of registers low
    tx_pkt[6] = 0x02;                // Byte Count (2 bytes for single register)
    tx_pkt[7] = (value >> 8);        // Value High
    tx_pkt[8] = (value & 0xFF);      // Value Low

    // Calculate CRC
    uint16_t crc = crc16_modbus(tx_pkt, 9);
    tx_pkt[9] = crc & 0xFF;
    tx_pkt[10] = (crc >> 8) & 0xFF;

    vTaskDelay(pdMS_TO_TICKS(50));
    uart_flush(SNIFFER_PORT_NUM);
    uart_write_bytes(SNIFFER_PORT_NUM, (const char *)tx_pkt, sizeof(tx_pkt));

    // Wait for response (expecting: SlaveID(1) + Func(1) + Address(2) + Count(2) + CRC(2))
    uint8_t rx_buf[11];
    int len = uart_read_bytes(SNIFFER_PORT_NUM, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(500));

    if (len == 8 && memcmp(tx_pkt, rx_buf, 6) == 0)
    {
        ESP_LOGI("WRITE", "Success: Reg %d = %d", reg_addr, value);
        return ESP_OK;
    }
    else
    {
        ESP_LOGE("WRITE", "Failed or No Response");
        if (len > 0)
            ESP_LOG_BUFFER_HEX("WRITE", rx_buf, len);

        return ESP_FAIL;
    }
}

void update_system_status() {
    // System status update
    uptime_seconds = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000;
    free_heap = esp_get_free_heap_size();
    min_heap_free = esp_get_minimum_free_heap_size();

    char json_string[512];

    snprintf(json_string, sizeof(json_string), "{\"uptime_seconds\": %llu, \"last_reset_reason\": %d, \"free_heap\": %lu, \"min_heap_free\": %lu}",
    uptime_seconds, last_reset_reason, free_heap, min_heap_free);

    mqtt_publish_topic(MQTT_SYSTEM_TOPIC, json_string);
}

// void sniffer_task(void *arg)
// {
//     uint8_t *data = (uint8_t *)malloc(SNIFFER_BUF_SIZE);
//     ESP_LOGI(TAG, "Sniffer started. Listening to Inverter on %d baud...", SNIFFER_BAUD_RATE);

//     while (1)
//     {
//         int len = uart_read_bytes(SNIFFER_PORT_NUM, data, SNIFFER_BUF_SIZE, 100 / portTICK_PERIOD_MS);

//         if (len > 0)
//         {
//             // Виводимо довжину та HEX для точного аналізу
//             ESP_LOGI(TAG, "Received %d bytes", len);
//             ESP_LOG_BUFFER_HEX(TAG, data, len);

//             printf("\n-------------------\n");
//         }
//     }
//     free(data);
// }

// void modbus_scan_task(void *arg)
// {
//     uint8_t rx_data[128];
//     uint8_t request[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};

//     ESP_LOGI(TAG, "Starting Modbus Register Scan (0-500)...");

//     for (uint16_t addr = 300; addr <= 350; addr++)
//     {
//         // Update address in request
//         request[2] = (addr >> 8) & 0xFF;
//         request[3] = addr & 0xFF;

//         // Calculate CRC
//         uint16_t crc = crc16_modbus(request, 6);
//         request[6] = crc & 0xFF;
//         request[7] = (crc >> 8) & 0xFF;

//         uart_flush(SNIFFER_PORT_NUM);
//         uart_write_bytes(SNIFFER_PORT_NUM, (const char *)request, 8);

//         int len = uart_read_bytes(SNIFFER_PORT_NUM, rx_data, sizeof(rx_data), pdMS_TO_TICKS(150));
//         if (len >= 5 && rx_data[0] == 0x01 && rx_data[1] == 0x03)
//         {
//             uint16_t value = (rx_data[3] << 8) | rx_data[4];
//             if (value != 0) // Filter out zero values which are often just "empty" registers
//             {
//                 ESP_LOGI(TAG, "Found Register: DEC: %d | HEX: 0x%04X | Value: %d (0x%04X)", addr, addr, value, value);
//             }
//         } else if (len > 0)
//         {
//             ESP_LOGI(TAG, "Unexpected response for address %d 0x%04X: ", addr, addr);
//             ESP_LOG_BUFFER_HEX(TAG, rx_data, len);
//         }
//         // else
//         // {
//         //     ESP_LOGI(TAG, "No response for address %d 0x%04X", addr, addr);
//         // }

//         // Short delay to avoid overwhelming the device
//         vTaskDelay(pdMS_TO_TICKS(20));
//     }

//     ESP_LOGI(TAG, "Scan finished.");
//     vTaskDelete(NULL);
// }

// // Structure to hold inverter state (example, can be expanded)
// typedef struct
// {
//     // (Block 200)
//     uint16_t grid_voltage; // 202
//     uint16_t grid_freq;    // 203
//     uint16_t grid_power;   // 204
//     uint16_t out_voltage;  // 205
//     uint16_t out_freq;     // 207
//     uint16_t out_power;    // 208
//     uint16_t out_current;  // 211
//     uint16_t out_va;       // 213
//     uint16_t batt_voltage; // 215
//     uint16_t batt_current; // 216
//     uint16_t batt_power;   // 217
//     uint16_t pv_voltage;   // 219
//     uint16_t pv_current;   // 220
//     uint16_t pv_power;     // 223
//     uint16_t load_pct;     // 225
//     uint16_t temp_dc;      // 226
//     uint16_t temp_inv;     // 227

//     // Settings (Block 320)
//     uint16_t set_out_priority;         // 301
//     uint16_t set_input_v_range;        // 302
//     uint16_t set_buzzer_mode;          // 303
//     uint16_t set_beep_source_intrpt;   // 304
//     uint16_t set_backlight;            // 305
//     uint16_t set_return_deflt_display; // 306
//     uint16_t set_pwr_save_mode;        // 307
//     uint16_t set_overload_restart;     // 308
//     uint16_t set_overtemp_restart;     // 309
//     uint16_t set_overload_bypass;      // 310
//     uint16_t set_out_v;                // 320
//     uint16_t set_out_f;                // 321
//     uint16_t set_batt_type;            // 322
//     uint16_t set_batt_dc_protect_v;    // 323
//     uint16_t set_batt_bulk_v;          // 324
//     uint16_t set_batt_float_v;         // 325
//     uint16_t set_batt_recovery_v;      // 326
//     uint16_t set_batt_protection_v;    // 327
//     uint16_t set_batt_cutoff_v;        // 329
//     uint16_t set_batt_chg_priority;    // 331
//     uint16_t set_batt_max_chg_a;       // 332
//     uint16_t set_batt_max_ac_chg_a;    // 333
//     uint16_t set_batt_eq_chg_v;        // 334
//     uint16_t set_batt_eq_chg_time;     // 335
//     uint16_t set_batt_eq_timeout;      // 336
//     uint16_t set_batt_eq_interval;     // 337
//     uint16_t set_batt_eq_mode;         // 338?
//     uint16_t set_batt_protection_soc;  // 341
//     uint16_t set_batt_recovery_soc;    // 342
//     uint16_t set_batt_cutoff_soc;      // 343
//     uint16_t set_batt_cv_float_time;   // 351?

//     uint16_t set_boot_method; // 406

// } inverter_state_t;
// inverter_state_t inv; // Global variable to hold current inverter state
