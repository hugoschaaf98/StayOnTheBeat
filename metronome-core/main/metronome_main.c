#include <stdio.h>
/* standart */
#include <string.h>

/* project configuration macros */
#include "sdkconfig.h"

/* FreeRTOs */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Gpio */
#include "driver/gpio.h"

/* storage */
#include "esp_vfs_semihost.h"
#include "esp_vfs_fat.h"
#include "esp_spiffs.h"

/* network */
#include "esp_netif.h"
#include "esp_event.h"
#include "mdns.h" /* local dns */
#include "lwip/apps/netbiosns.h"

/* SD card */
#if CONFIG_WEB_DEPLOY_SD
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#endif

/* general */
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_log.h"

/* custom */
#include "wifi_ap.h"


static const char* TAG = "metronome_main";



/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* MAIN
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

void app_main(void)
{
    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();

    initialise_mdns();
    netbiosns_init();
    netbiosns_set_name(CONFIG_MDNS_HOST_NAME);

    ESP_ERROR_CHECK(init_fs());
    ESP_ERROR_CHECK(start_rest_server(CONFIG_WEB_MOUNT_POINT));
}
