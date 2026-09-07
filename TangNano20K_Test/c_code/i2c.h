#ifndef I2C_H
#define I2C_H

#include "std_int_types.h"

int i2c_write(uint8_t addr, uint8_t *data, uint8_t len);

#endif