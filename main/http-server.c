#include "esp_log.h"

#include <complex.h>
#include <fcntl.h>
#include <fenv.h>
#include <locale.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include "http-server.h"
#include "filesystem.h"
#include "http_parser.h"
#include "state.h"

#define ROOT2 1.41421356237

extern int mode;

// basic string processing to help with query parameters
bool prefix(const char *pre, const char *str)
{
    return strncmp(pre, str, strlen(pre)) == 0;
}

inline bool eq(const char *a, const char *b)
{
    return strcmp(a, b) == 0;
}

inline float interpolate(float min, float max, float t)
{
    return min + (max - min) * t;
}

float clamp(float value)
{
    return fmin(fmax(value, 0.0), 1.0);
}

esp_err_t state_handler(httpd_req_t *req) {
	size_t buf_len = 768;
    char *buf = calloc(1, buf_len);
    if (!buf) {
        ESP_LOGE("REST_HANDLER", "Failed to allocate buffer");
        return ESP_ERR_NO_MEM;
    }
    sprintf(buf, "{"
    	"\"Kp\":%f,\n"
        "\"Ki\":%f,\n"
        "\"Kd\":%f,\n"
        "\"motorEnable\":%s,\n"
        "\"rpm\":%f,\n"
        "\"angle\":%f\n"
        "}",
      Kp,
      Ki,
      Kd,
      motorEnable ? "true" : "false",
      rpm,
      angle);
    httpd_resp_set_type(req, "text/json");
    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    free(buf);
    return ESP_OK;
}

void process_query_param(const char *param, const char *value)
{
	// ===== Operation =====
    if (eq("action", param)) {
        if (eq("stop", value)) {
            ESP_LOGI("REST_HANDLER", "Stop");
            rpm = 0;
        } else if (eq("drive", value)) {
            ESP_LOGI("REST_HANDLER", "Drive");
            // TODO: Set velocity and steering angle
        }
    }
    // ===== Controller Tuning =====
    else
    QUERY_PARAM(Kp,f)
    QUERY_PARAM(Ki,f)
    QUERY_PARAM(Kd,f)
    QUERY_PARAM(rpm,f)
    QUERY_PARAM(angle,f)
    DONE_QUERY_PARAMS
}

void handle_query_params(char *buf) {
    // Add your control logic here based on the query string
    // process the string "in place" by replacing the & and =with nulls
    char *p = buf;
    char *c = buf;
    char *v = NULL;
    for(;;) {
        // & or null terminator means we are at the end of a parameter
        if (*c == '&' || *c == 0) {
            bool last = (*c == 0); // null means last parameter
            *c = 0; // null terminate
            process_query_param(p,v);
            // start again on the next parameter
            p = c+1;
            v = NULL;
            if (last)
                break;
        } else if (*c == '=') {
            // = means start of the value
            *c = 0; // null terminate preceeding parameter name
            v = c+1; // value starts after '='
        }
        c++;
    }
}

// Define the REST API endpoint handler
esp_err_t control_handler(httpd_req_t *req) {
    char *buf;
    size_t buf_len;

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            // Process the query parameters
            ESP_LOGI("REST_HANDLER", "control query => %s", buf);

            // calls process_query_param for each parameter
            handle_query_params(buf);

            // after query params are processed we can update state
        } else {
            // Handle error case
            ESP_LOGE("REST_HANDLER", "Failed to process query parameters");
        }
        free(buf);
    }

    // Respond to the client with current state
    return state_handler(req);
}

// URI handler for serving static files from the filesystem
esp_err_t get_file_handler(httpd_req_t *req) {
    char filepath[512];
    strcpy(filepath, FS_BASE_PATH);
    strcat(filepath, "/www");
    if (strcmp(req->uri, "/") == 0) {
        strcat(filepath, "/index.html");
    } else {
        strcat(filepath, req->uri);
    }

    struct stat st;
    if (stat(filepath, &st) == -1) {
        ESP_LOGE("HTTP_SERVER", "File not found: %s", filepath);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    FILE *fd = fopen(filepath, "r");
    if (!fd) {
        ESP_LOGE("HTTP_SERVER", "Failed to open file : %s", filepath);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char *buf = 0;
    if (st.st_size > 0) {
        buf = malloc(st.st_size);
        if (!buf) {
            fclose(fd);
            ESP_LOGE("HTTP_SERVER", "Failed to allocate memory");
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        fread(buf, 1, st.st_size, fd);
        fclose(fd);
    }
    // set the content type based on the file extension
    char *content_type = "text/html";
    if (strcmp(filepath + strlen(filepath) - 4, ".css") == 0) {
        content_type = "text/css";
    } else if (strcmp(filepath + strlen(filepath) - 4, ".js") == 0) {
        content_type = "application/javascript";
    }
    httpd_resp_set_type(req, content_type);
    httpd_resp_send(req, buf, st.st_size);
    if (buf) {
        free(buf);
    }

    return ESP_OK;
}

// Register the URI handler
httpd_uri_t put_control_uri = {
    .uri      = "/api/control",
    .method   = HTTP_PUT,
    .handler  = control_handler,
    .user_ctx = NULL
};

httpd_uri_t get_state_uri = {
    .uri      = "/api/state",
    .method   = HTTP_GET,
    .handler  = state_handler,
    .user_ctx = NULL
};

// Register the file handler
httpd_uri_t file_uri = {
    .uri      = "/*", // Wildcard to match all paths
    .method   = HTTP_GET,
    .handler  = get_file_handler,
    .user_ctx = NULL
};


httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    // Enable wildcard matching
    config.uri_match_fn = httpd_uri_match_wildcard;
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &put_control_uri));
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &get_state_uri));
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &file_uri)); // uses /* wildcard to match all paths, so MUST be registered last
    }
    return server;
}
