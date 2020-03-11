/* standart */
#include <stdio.h>
#include <string.h>

/* FreeRTOs */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* network */
#include "esp_netif.h"
#include "esp_event.h"
#include "mdns.h" /* local dns */


/* general */
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_log.h"

/* custom */
#include "init.h"


static const char* TAG = "metronome_main";

void metronome_init(void);

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* MAIN                                                                    */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

void app_main(void)
{
    ESP_LOGI(TAG, "Metronome initialization...");
    metronome_init();
}

void metronome_init(void)
{
    /* Initialize NVS */
    ESP_LOGV(TAG, "NVS initialization...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGV(TAG, "Wifi initialization...");
    wifi_ap_init();

    ESP_LOGV(TAG, "File System initialization...");
    ESP_ERROR_CHECK(fs_init());

    ESP_LOGV(TAG, "Starting http server...");
    ESP_ERROR_CHECK(http_server_start(CONFIG_M_WEB_MOUNT_POINT));
}