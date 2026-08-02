#include <esp_http_server.h>
#include <cJSON.h>
#include "esp_log.h"
#include "myNVS.h"
#include "wifi.h"
#include "haDiscovery.h"

httpd_handle_t webserver_init(void);
esp_err_t webserver_stop(void);