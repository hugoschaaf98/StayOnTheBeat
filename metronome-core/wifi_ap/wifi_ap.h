#ifndef WIFI_AP_H
#define WIFI_AP_H 1

#include "esp_wifi.h"
#include "esp_event.h"
#include "lwip/err.h"
#include "lwip/sys.h"

/* Metronome WiFi configuration 
*/
#ifndef METRONOME_WIFI_SSID
#define METRONOME_WIFI_SSID		"Metronome-default"
#endif
#ifndef METRONOME_WIFI_PASS
#define METRONOME_WIFI_PASS		"default"
#endif
#ifndef METRONOME_MAX_STA_CONN
#define METRONOME_MAX_STA_CONN	4
#endif

void wifi_init_softap(void);

#endif