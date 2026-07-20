#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

void nvs_init(void);
void nvs_save_str(const char *key, const char *value);
void nvs_save_int(const char *key, int32_t value);
esp_err_t nvs_read_str(const char *key, char *out_buffer, size_t max_size);
esp_err_t nvs_read_int(const char *key, int32_t *out_value);