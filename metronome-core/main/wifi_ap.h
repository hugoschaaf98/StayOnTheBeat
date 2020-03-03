#ifndef WIFI_AP_H
#define WIFI_AP_H 1

#include "esp_wifi.h"
#include "esp_event.h"
#include "lwip/err.h"
#include "lwip/sys.h"

/* Metronome WiFi configuration 
*/
#define METRONOME_WIFI_SSID		"METRONOME-CORE"
#define METRONOME_WIFI_PASS		"StayOnTheBeat"
#define METRONOME_MAX_STA_CONN	4

void wifi_init_softap(void);

#endif