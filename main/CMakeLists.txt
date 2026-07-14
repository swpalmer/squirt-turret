idf_component_register(
    SRCS app_main.c svpwm/esp_svpwm.c foc/esp_foc.c
         filesystem.c wifi.c http-server.c
         as5600.c
         INCLUDE_DIRS "svpwm" "foc"
    PRIV_REQUIRES esp_driver_mcpwm esp_driver_gpio
         esp_wifi nvs_flash esp_http_server
         esp_driver_i2c esp_timer
)

# Note: you must have a partition named the first argument (here it's "fs")
# in your partition table csv file.
littlefs_create_partition_image(fs ../fs FLASH_IN_PROJECT)
