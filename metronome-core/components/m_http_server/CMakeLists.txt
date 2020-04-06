# Edit following two lines to set component requirements (see docs)

# file < http_server.c >

# #include "esp_http_server.h"
# //#include "esp_system.h"
# #include "esp_log.h"
# #include "esp_vfs.h"
# #include "cJSON.h"
# #include "errno.h"


idf_component_register(SRCS "m_http_server.c"
                    INCLUDE_DIRS "."
                    REQUIRES "esp_http_server log vfs json")