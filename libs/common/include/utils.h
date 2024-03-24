#include <cstdio>
#include <math.h>
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "drivetrain_config.h"

void initI2C(i2c_inst_t* &i2c_port, uint baudrate, uint sda_pin, uint scl_pin);

float wrap_pi(const float heading);

bool reserved_addr(uint8_t addr);

void scan_i2c_bus();