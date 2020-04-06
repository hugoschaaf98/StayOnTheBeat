# Edit following two lines to set component requirements (see docs)
# Includes memo :

# file < fs_init.c >

#include "esp_vfs_fat.h"
#include "esp_vfs_semihost.h"
#include "esp_spiffs.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/gpio.h"

# file < wifi_ap_init.c >

#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "mdns.h" /* local dns */
#include "lwip/apps/netbiosns.h" /* for domain name but with windows devices */
#include <string.h>
#include "esp_log.h"

idf_component_register(SRCS "fs_init.c" "wifi_ap_init.c"
                    INCLUDE_DIRS "."
                    REQUIRES "vfs fatfs spiffs sdmmc driver esp_netif esp_wifi esp_event lwip mdns log")
