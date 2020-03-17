#include "init.h"

#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "mdns.h" /* local dns */
#include "lwip/apps/netbiosns.h" /* for domain name but with windows devices */


#include <string.h>
#include "esp_log.h"

static const char* TAG = "wifi_ap_init";

#define MDNS_INSTANCE "Metronome-WebServer"

static void mdns_configure(void)
{
    mdns_init();
    mdns_hostname_set(CONFIG_M_MDNS_HOST_NAME);
    mdns_instance_name_set(MDNS_INSTANCE);
    netbiosns_init();
    netbiosns_set_name(CCONFIG_M_MDNS_HOST_NAME);


    mdns_txt_item_t serviceTxtData[] = {
        {"board", "esp32"},
        {"path", "/"}
    };

    ESP_ERROR_CHECK(mdns_service_add("Metronome-WebServer", "_http", "_tcp", 80, serviceTxtData,
                                     sizeof(serviceTxtData) / sizeof(serviceTxtData[0])));
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

esp_err_t wifi_ap_init(void)
{

    esp_err_t ret = ESP_OK;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    mdns_configure();

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    /* configure wifi */
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = CONFIG_M_WIFI_SSID,
            .ssid_len = strlen(CONFIG_M_WIFI_SSID),
            .password = CONFIG_M_WIFI_PASSWORD,
            .max_connection = CONFIG_M_WIFI_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    /* if password empty set open acces */
    if (strlen(CONFIG_M_WIFI_PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_ap_init finished. SSID:%s password:%s",
             CONFIG_M_WIFI_SSID, CONFIG_M_WIFI_PASSWORD);

    return ret;
}