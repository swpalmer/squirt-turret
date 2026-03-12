#include "driver/i2c_master.h"

#define I2C_PORT 0
#define I2C_SDA  8
#define I2C_SCL  9
#define AS5600_ADDR 0x36

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t as5600_handle;

void as5600_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_ADDR,
        .scl_speed_hz = 400000,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &as5600_handle));
}

static uint16_t as5600_read_raw_angle(void)
{
    uint8_t reg = 0x0E;
    uint8_t data[2];

    ESP_ERROR_CHECK(i2c_master_transmit_receive(
        as5600_handle,
        &reg, 1,
        data, 2,
        -1));

    uint16_t angle = ((uint16_t)data[0] << 8) | data[1];
    return angle & 0x0FFF;  // 12-bit mask
}

float as5600_get_mech_deg(void)
{
    uint16_t raw = as5600_read_raw_angle();
    return (raw * 360.0f) / 4096.0f;
}
