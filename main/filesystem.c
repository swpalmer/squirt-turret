// Initialize Filesystem Partition
//#include "esp_spiffs.h"
//
// Note: LittleFS is a lightweight file system for embedded systems.
// It is designed to be used with SPI flash memory and is optimized for low power consumption.
// requires:  idf.py add-dependency joltwallet/littlefs
#include "esp_littlefs.h"
#include "esp_log.h"
#include <sys/stat.h>
#include <dirent.h>
#include "filesystem.h"

static const char *TAG = "FILESYSTEM";

esp_err_t start_filesystem() {
    ESP_LOGI(TAG, "Initializing filesystem");

    esp_vfs_littlefs_conf_t conf = {
        .base_path = FS_BASE_PATH,
        .partition_label = "fs",
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_littlefs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find LittleFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
        }
        return ret;
    }

    size_t total = 0, used = 0;
    ret = esp_littlefs_info("fs", &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get 'fs' partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    return ESP_OK;
}



void list_files(const char *path) {
    DIR *dir = opendir(path);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open directory %s", path);
        return;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        ESP_LOGI(TAG, "Found file: %s", entry->d_name);
    }

    closedir(dir);
}
