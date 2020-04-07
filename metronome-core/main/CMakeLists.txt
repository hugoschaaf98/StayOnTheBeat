# Edit following two lines to set component requirements (see docs)

# file < metronome_main.c >
# Include list and corresponding component names
#
# file --> component

# /* FreeRTOS */
# "freertos/FreeRTOS.h"
# "freertos/task.h"		--> freertos

# /* general */
# "nvs_flash.h"			--> nvs_flash
# "esp_log.h"			--> log
# "driver/timer.h"		--> driver

# /* custom */
# "m_init.h"			--> m_init
# "m_http_server.h"		--> m_http_server  
# "m_beat.h"			--> m_beat

idf_component_register(SRCS "metronome_main.c"
                     INCLUDE_DIRS "."
                     REQUIRES "freertos driver nvs_flash log m_beat m_http_server m_init ")

if(CONFIG_EXAMPLE_WEB_DEPLOY_SF)
    set(WEB_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../front")
    if(EXISTS ${WEB_SRC_DIR}/dist)
        spiffs_create_partition_image(www ${WEB_SRC_DIR}/dist FLASH_IN_PROJECT)
    else()
        message(FATAL_ERROR "${WEB_SRC_DIR}/dist doesn't exit. Please run 'npm run build' in ${WEB_SRC_DIR}")
    endif()
endif()