#include "driver/i2c_master.h"

#define I2C_PORT_0 0
#define I2C_SDA_0  21 // 8
#define I2C_SCL_0  32 // 9
#define I2C_SDA_1  33
#define I2C_SCL_1  0

#define AS5600_ADDR 0x36

static int i2c_scl_pin [2] = {I2C_SCL_0, I2C_SCL_1};
static int i2c_sda_pin [2] = {I2C_SDA_0, I2C_SDA_1};
static i2c_master_bus_handle_t bus_handle [2] = {nullptr, nullptr};
static i2c_master_dev_handle_t as5600_handle [2] = {nullptr, nullptr};

void as5600_init(int axis)
{
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT_0 + axis,
        .sda_io_num = i2c_sda_pin[axis],
        .scl_io_num = i2c_scl_pin[axis],
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle[axis]));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_ADDR,
        .scl_speed_hz = 400000,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle[axis], &dev_config, &as5600_handle[axis]));
}

static uint16_t as5600_read_raw_angle(int axis)
{
    uint8_t reg = 0x0E;
    uint8_t data[2];

    if (axis < 0 || axis > 1)
        return 0xffff; // outof 12-bit range

    ESP_ERROR_CHECK(i2c_master_transmit_receive(
        as5600_handle[axis],
        &reg, 1,
        data, 2,
        -1));

    uint16_t angle = ((uint16_t)data[0] << 8) | data[1];
    return angle & 0x0FFF;  // 12-bit mask
}

float as5600_get_mech_deg(int axis)
{
    uint16_t raw = as5600_read_raw_angle(axis);
    return (raw * 360.0f) / 4096.0f;
}
