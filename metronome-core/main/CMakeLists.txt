# Edit following two lines to set component requirements (see docs)

# file < metronome_main.c >

# /* FreeRTOs */
# #include "freertos/FreeRTOS.h"
# #include "freertos/task.h"

# // /* network */
# // #include "esp_netif.h"
# // #include "esp_event.h"
# // #include "mdns.h" /* local dns */


# /* general */
# // #include "esp_system.h"
# #include "nvs_flash.h"
# #include "esp_log.h"

# /* custom */
# #include "m_init.h"
# #include "m_http_server.h"
# #include "m_beat.h"
 idf_component_register(SRCS "metronome_main.c"
                     INCLUDE_DIRS "."
                     REQUIRES "freertos nvs_flash log m_beat m_http_server m_init")

if(CONFIG_EXAMPLE_WEB_DEPLOY_SF)
    set(WEB_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../front")
    if(EXISTS ${WEB_SRC_DIR}/dist)
        spiffs_create_partition_image(www ${WEB_SRC_DIR}/dist FLASH_IN_PROJECT)
    else()
        message(FATAL_ERROR "${WEB_SRC_DIR}/dist doesn't exit. Please run 'npm run build' in ${WEB_SRC_DIR}")
    endif()
endif()