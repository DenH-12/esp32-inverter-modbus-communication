#include "modbus.h"

static const char *TAG = "MODBUS";

EventGroupHandle_t modbus_event_group;
QueueHandle_t modbus_cmd_queue;
QueueHandle_t scan_queue;
scan_status_t g_scan_status;
extern uint8_t data_read_interval_s;
modbus_block_t realtime_block = {0};
modbus_block_t settings_block = {0};

void modbus_init()
{
    modbus_event_group = xEventGroupCreate();
    // Trigger initial settings read on startup
    xEventGroupSetBits(modbus_event_group, INV_EVENT_REFRESH_SETTINGS);

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

    modbus_cmd_queue = xQueueCreate(10, sizeof(inv_cmd_t));
    scan_queue = xQueueCreate(5, sizeof(scan_args_t));

    scan_status_init();
}

void scan_status_init(void)
{
    g_scan_status.mutex = xSemaphoreCreateMutex();
    g_scan_status.results_array = cJSON_CreateArray();
    g_scan_status.is_running = false;
    g_scan_status.is_done = false;
}

void modbus_read_task(void *pvParameters)
{
    if (data_read_interval_s == 0)
    {
        ESP_LOGW(TAG, "Delay is 0, deleting task");
        vTaskDelete(NULL); // Exit the task if readding data delay = 0
    }

    // Determine the min and max register addresses for realtime and settings blocks based on the register maps
    if (realtime_map && realtime_map_count > 0)
    {
        uint16_t min_reg = UINT16_MAX;
        uint16_t max_reg = 0;

        for (size_t i = 0; i < realtime_map_count; i++)
        {
            uint16_t reg = realtime_map[i].reg;
            if (reg < min_reg)
                min_reg = reg;
            if (reg > max_reg)
                max_reg = reg;
        }

        if (min_reg <= max_reg)
        {
            realtime_block = (modbus_block_t){min_reg, (max_reg - min_reg + 1), "realtime_block"};
        }
    }

    if (settings_map && settings_map_count > 0)
    {
        uint16_t min_reg = UINT16_MAX;
        uint16_t max_reg = 0;

        for (size_t i = 0; i < settings_map_count; i++)
        {
            uint16_t reg = settings_map[i].reg;
            if (reg < min_reg)
                min_reg = reg;
            if (reg > max_reg)
                max_reg = reg;
        }

        if (min_reg <= max_reg)
        {
            settings_block = (modbus_block_t){min_reg, (max_reg - min_reg + 1), "settings_block"};
        }
    }

    ESP_LOGI(TAG, "Realtime block: start=%d, count=%d", realtime_block.start_addr, realtime_block.count);
    ESP_LOGI(TAG, "Settings block: start=%d, count=%d", settings_block.start_addr, settings_block.count);

    inv_cmd_t cmd;
    scan_args_t scan_req;

    while (1)
    {
        // Check if there are any scan requests in the queue
        if (xQueueReceive(scan_queue, &scan_req, 0) == pdTRUE)
        {
            run_modbus_scan(&scan_req);

            xSemaphoreTake(g_scan_status.mutex, portMAX_DELAY);
            g_scan_status.is_running = false;
            g_scan_status.is_done = true;
            xSemaphoreGive(g_scan_status.mutex);

            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Check MQTT connection before reading
        if (!(xEventGroupGetBits(modbus_event_group) & INV_EVENT_MQTT_CONNECTED))
        {
            ESP_LOGW(TAG, "MQTT not connected, skipping read cycle wait 5 seconds...");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        // Check if there are any pending commands to write before reading
        else if (xQueueReceive(modbus_cmd_queue, &cmd, 0) == pdPASS)
        {
            modbus_write_register(cmd.reg, cmd.value);
            xEventGroupSetBits(modbus_event_group, INV_EVENT_REFRESH_SETTINGS); // Trigger a settings refresh after writing
            vTaskDelay(pdMS_TO_TICKS(100)); // Short delay after write before next action
        }

        // Check refresh settings bit
        if (xEventGroupGetBits(modbus_event_group) & INV_EVENT_REFRESH_SETTINGS)
        {
            if (settings_block.count > 0)
                modbus_read_block(&settings_block);

            xEventGroupClearBits(modbus_event_group, INV_EVENT_REFRESH_SETTINGS);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (realtime_block.count > 0)
            modbus_read_block(&realtime_block);

        vTaskDelay(pdMS_TO_TICKS(data_read_interval_s * 1000)); // Delay before next full read cycle
    }
}

void parse_modbus_data(uint16_t start_addr, uint8_t *data)
{
    // Parse the data based on the starting address and publish to MQTT if needed
    // For realtime block
    if (start_addr == realtime_block.start_addr)
    {
#define GET_REG(addr) ((data[(addr - realtime_block.start_addr) * 2] << 8) | data[(addr - realtime_block.start_addr) * 2 + 1])
        if (SNIFFER_PUBLISH_MQTT)
        {
            char json_string[1024];
            json_string[0] = '{';  // Start JSON
            json_string[1] = '\0'; // set terminator, so strncat works

            // Loop through the realtime_map to get the values and build the JSON string
            for (size_t i = 0; i < realtime_map_count; i++)
            {
                const realtime_reg_t *meta = &realtime_map[i];
                float value = 0.0f;
                if (meta->is_signed)
                {
                    value = (int16_t)(GET_REG(meta->reg)) * meta->factor;
                }
                else
                {
                    value = GET_REG(meta->reg) * meta->factor;
                }
                char line[64];
                snprintf(line, sizeof(line), "\"%s\": %g,", meta->id, value);
                strncat(json_string, line, sizeof(json_string) - strlen(json_string) - 1);
            }
            // Remove trailing comma
            if (strlen(json_string) > 0)
            {
                json_string[strlen(json_string) - 1] = '\0';
            }
            strncat(json_string, "}", sizeof(json_string) - strlen(json_string) - 1); // End JSON

            mqtt_publish_topic(mqtt_stats_topic, json_string, false);
        }
    }
    // For settings block
    else if (start_addr == settings_block.start_addr)
    {
#define GET_REG_300(addr) ((data[(addr - settings_block.start_addr) * 2] << 8) | data[(addr - settings_block.start_addr) * 2 + 1])
        if (SNIFFER_PUBLISH_MQTT)
        {
            static char json_string[2048];
            json_string[0] = '{';  // Start JSON
            json_string[1] = '\0'; // set terminator, so strncat works

            for (size_t i = 0; i < settings_map_count; i++)
            {
                const settings_reg_t *meta = &settings_map[i];
                float value = 0.0f;
                value = GET_REG_300(meta->reg) * meta->factor;

                char line[64];
                snprintf(line, sizeof(line), "\"%s\": %g,", meta->id, value);
                strncat(json_string, line, sizeof(json_string) - strlen(json_string) - 1);
            }
            // Remove trailing comma
            if (strlen(json_string) > 0)
            {
                json_string[strlen(json_string) - 1] = '\0';
            }
            strncat(json_string, "}", sizeof(json_string) - strlen(json_string) - 1); // End JSON

            mqtt_publish_topic(mqtt_settings_topic, json_string, false);
        }
    }
}

void modbus_read_block(const modbus_block_t *block)
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

    // Check CRC from response
    uint16_t rx_crc = rx_buf[len - 2] | (rx_buf[len - 1] << 8);
    uint16_t calc_crc = crc16_modbus(rx_buf, len - 2);

    if (rx_crc != calc_crc)
    {
        ESP_LOGE(TAG, "CRC Mismatch! Calculated: 0x%04X, Received: 0x%04X", calc_crc, rx_crc);
        return;
    }

    if (len >= expected_len && rx_buf[0] == SNIFFER_MODBUS_ADDR && rx_buf[1] == 0x03)
    {
        parse_modbus_data(block->start_addr, &rx_buf[3]);
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

esp_err_t modbus_write_register(uint16_t reg_addr, uint16_t value)
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

    // Check CRC from response
    uint16_t rx_crc = rx_buf[len - 2] | (rx_buf[len - 1] << 8);
    uint16_t calc_crc = crc16_modbus(rx_buf, len - 2);

    if (rx_crc != calc_crc)
    {
        ESP_LOGE(TAG, "CRC Mismatch! Calculated: 0x%04X, Received: 0x%04X", calc_crc, rx_crc);
        return ESP_FAIL;
    }

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

void run_modbus_scan(const scan_args_t *args)
{
    uint8_t rx_buf[256];
    uint8_t tx_pkt[8] = {SNIFFER_MODBUS_ADDR, 0x03};

    ESP_LOGI(TAG, "Starting Modbus Register Scan (%u-%u)...", args->start_addr, args->end_addr);

    for (uint16_t addr = args->start_addr; addr <= args->end_addr; )
    {
        // Calculate how much registers remaining to read
        uint16_t remaining = args->end_addr - addr + 1;
        uint16_t count = (remaining < args->reg_num_per_read) ? remaining : args->reg_num_per_read;

        // Update address in request
        tx_pkt[2] = (addr >> 8) & 0xFF; // Address High
        tx_pkt[3] = addr & 0xFF;        // Address Low
        tx_pkt[4] = (count >> 8) & 0xFF; // Number of registers high
        tx_pkt[5] = count & 0xFF;        // Number of registers low

        // Calculate CRC
        uint16_t crc = crc16_modbus(tx_pkt, 6);
        tx_pkt[6] = crc & 0xFF;
        tx_pkt[7] = (crc >> 8) & 0xFF;

        uart_flush(SNIFFER_PORT_NUM);
        uart_write_bytes(SNIFFER_PORT_NUM, (const char *)tx_pkt, 8);
        
        int len = uart_read_bytes(SNIFFER_PORT_NUM, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(500));
        uint16_t expected_len = 5 + (count * 2);

        if (len < 5 || len != expected_len)
        {
            ESP_LOGW(TAG, "Read 0x%04X (%u registers): Timeout or invalid length (%d bytes, expected %d)", addr, count, len, expected_len);
            addr += count;
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        // Check CRC from response
        uint16_t rx_crc = rx_buf[len - 2] | (rx_buf[len - 1] << 8);
        uint16_t calc_crc = crc16_modbus(rx_buf, len - 2);

        if (rx_crc != calc_crc)
        {
            ESP_LOGE(TAG, "CRC Mismatch! Calculated: 0x%04X, Received: 0x%04X", calc_crc, rx_crc);
            addr += count;
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        if (rx_buf[0] == SNIFFER_MODBUS_ADDR && rx_buf[1] == 0x03)
        {
            for (uint16_t i = 0; i < count; i++)
            {
                uint16_t idx = 3 + (i * 2);
                uint16_t value = (rx_buf[idx] << 8) | rx_buf[idx + 1];
                uint16_t reg_addr = addr + i;

                if (value != 0) // Filter out zero values which are often just "empty" registers
                {
                    ESP_LOGI(TAG, "Found Register: DEC: %d (0x%04X) | Value: uint_16t=%d (0x%04X)", reg_addr, reg_addr, value, value);
                    xSemaphoreTake(g_scan_status.mutex, portMAX_DELAY);

                    cJSON *item = cJSON_CreateObject();
                    cJSON_AddNumberToObject(item, "addr", (double)reg_addr);
                    cJSON_AddNumberToObject(item, "val", (double)value);
                    cJSON_AddItemToArray(g_scan_status.results_array, item);

                    xSemaphoreGive(g_scan_status.mutex);
                }
            }
        }
        
        addr += count;
        // Short delay to avoid overwhelming the device
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    ESP_LOGI(TAG, "Scan finished.");
}