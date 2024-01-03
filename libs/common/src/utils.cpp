#include "utils.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "drivetrain_config.h"

void initI2C(i2c_inst_t* &i2c_port, uint baudrate, uint sda_pin, uint scl_pin) {
    i2c_port = i2c_default; // or i2c0, i2c1, etc.
    i2c_init(i2c_port, baudrate);

    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
}