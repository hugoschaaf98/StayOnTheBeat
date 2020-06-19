# Edit following two lines to set component requirements (see docs)

# file < metronome_main.c >
# Include list and corresponding component names
#
# file --> component
#
# "freertos/FreeRTOS.h"
# "freertos/task.h"		--> freertos
# "esp_attr.h"          --> xtensa
# "nvs_flash.h"			--> nvs_flash
# "esp_log.h"			--> log
# "driver/timer.h"		--> driver
# "m_init.h"			--> m_init
# "m_http_server.h"		--> m_http_server  
# "m_beat.h"			--> m_beat

idf_component_register(SRCS "metronome_main.c"
                     INCLUDE_DIRS "."
                     REQUIRES "freertos xtensa driver nvs_flash log m_beat m_http_server m_init ")