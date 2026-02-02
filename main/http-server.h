
#include "esp_http_server.h"

httpd_handle_t start_webserver(void);

#define QUERY_PARAM(name, type) \
    if  (eq(#name , param)) { \
      name = ato##type(value); \
      ESP_LOGI("REST_HANDLER", #name " %" #type, name);\
    } else

#define DONE_QUERY_PARAMS \
	{ \
	    ESP_LOGI("REST_HANDLER", "Unknown parameter: %s = %s", param, value); \
	}
