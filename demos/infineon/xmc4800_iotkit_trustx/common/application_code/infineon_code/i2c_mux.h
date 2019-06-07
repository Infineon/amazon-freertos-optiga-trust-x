#ifndef I2C_MUX_H
#define I2C_MUX_H

#include <stdint.h>

int32_t i2c_mux_acquire(void);

void i2c_mux_release(void);

#endif
