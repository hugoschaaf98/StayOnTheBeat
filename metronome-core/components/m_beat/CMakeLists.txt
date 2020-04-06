# Edit following two lines to set component requirements (see docs)

# file < m_beat.c >

# #include "freertos/FreeRTOS.h"
# #include "freertos/task.h"
# #include "esp_log.h"
# #include "esp_attr.h"

idf_component_register(SRCS "m_beat.c"
                    INCLUDE_DIRS "."
                    REQUIRES "freertos log xtensa driver")