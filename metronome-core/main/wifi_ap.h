#ifndef WIFI_AP_H
#define WIFI_AP_H 1

#include "esp_wifi.h"
#include "esp_event.h"
#include "lwip/err.h"
#include "lwip/sys.h"

void wifi_init_softap(void);

#endif