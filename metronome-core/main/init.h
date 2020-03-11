#ifndef INIT_H
#define INIT_H 1

/* project configuration macros */
#include "sdkconfig.h"
#include "esp_err.h"

esp_err_t fs_init(void);
esp_err_t wifi_ap_init(void);
esp_err_t http_server_start(const char *base_path);


#endif
