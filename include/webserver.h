#include <esp_http_server.h>
#include <cJSON.h>
#include "esp_log.h"
#include "frontend.h"
#include "myNVS.h"
#include "wifi.h"
#include "haDiscovery.h"

httpd_handle_t webserver_init(void);
esp_err_t webserver_stop(void);
esp_err_t status_get_handler(httpd_req_t *req);
esp_err_t settings_post_handler(httpd_req_t *req);
esp_err_t settings_get_handler(httpd_req_t *req);
esp_err_t index_get_handler(httpd_req_t *req);
esp_err_t reboot_post_handler(httpd_req_t *req);
esp_err_t ha_discovery_post_handler(httpd_req_t *req);
esp_err_t reset_post_handler(httpd_req_t *req);