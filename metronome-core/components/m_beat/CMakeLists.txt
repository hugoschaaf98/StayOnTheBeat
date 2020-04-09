# Edit following two lines to set component requirements (see docs)

# file < m_beat.c >
# Include list and corresponding component names
#
# file --> component
#
# "freertos/FreeRTOS.h"
# "freertos/task.h"		
# "freertos/queue.h"	--> freertos
# "esp_attr.h"          --> xtensa
# "esp_heap_caps.h"     --> heap
# "esp_log.h"           --> log
# "driver/timer.h"		
# "driver/i2s.h"		--> driver

idf_component_register(SRCS "m_beat.c"
                    INCLUDE_DIRS "."
                    REQUIRES "freertos xtensa heap log driver")