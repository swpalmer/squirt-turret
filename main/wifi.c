/* Connect to WiFi */
#include "esp_wifi.h"
#include <string.h>
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "http-server.h"

#define TAG "WiFi"

#define WIFI_TX_POWER_20DBM 84

static esp_netif_t *sta_netif = NULL;

static httpd_handle_t server = NULL;
extern esp_netif_t *sta_netif;

void wifi_scan_and_print(void) {
    wifi_scan_config_t scan_config = {
        .ssid = 0,
        .bssid = 0,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE
    };

    ESP_LOGI(TAG, "Starting Wi-Fi scan...");
    ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));

    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);

    if (ap_count == 0) {
        ESP_LOGW(TAG, "No APs found!");
        return;
    }

    wifi_ap_record_t *ap_records = (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * ap_count);
    if (ap_records == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for AP records");
        return;
    }

    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_count, ap_records));

    ESP_LOGI(TAG, "Found %d APs:", ap_count);
    for (int i = 0; i < ap_count; i++) {
        ESP_LOGI(TAG, "SSID: %s, RSSI: %d, Auth: %d",
                 ap_records[i].ssid, ap_records[i].rssi, ap_records[i].authmode);
    }

    free(ap_records);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            ESP_LOGI("WIFI_HANDLER", "Wi-Fi station started");
        } else if (event_id == WIFI_EVENT_STA_CONNECTED) {
            ESP_LOGI("WIFI_HANDLER", "Wi-Fi connected to AP");
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            esp_wifi_connect(); // keep trying to connect
            wifi_event_sta_disconnected_t* disconnected = (wifi_event_sta_disconnected_t*) event_data;
            ESP_LOGE("WIFI_HANDLER", "Wi-Fi disconnected! Reason: %d", disconnected->reason);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI("WIFI_HANDLER", "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        if (server == NULL) {
            ESP_LOGI("WIFI_HANDLER", "Starting web server");
            server = start_webserver();
        }
    }
}

void wifi_init() {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    sta_netif = esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(WIFI_TX_POWER_20DBM));
}

void wifi_connect(const char *ssid, const char *password) {
    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config)); // Zero out the structure

    strcpy((char *)wifi_config.sta.ssid, ssid);
    strcpy((char *)wifi_config.sta.password, password);

    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_connect());
}

void wifi_disconnect() {
    ESP_ERROR_CHECK(esp_wifi_disconnect());
}

void app_wifi_deinit() {
    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_wifi_deinit());
    ESP_ERROR_CHECK(esp_event_loop_delete_default());
    if (sta_netif) {
        esp_netif_destroy_default_wifi(sta_netif);
    }
    ESP_ERROR_CHECK(nvs_flash_deinit());
}
