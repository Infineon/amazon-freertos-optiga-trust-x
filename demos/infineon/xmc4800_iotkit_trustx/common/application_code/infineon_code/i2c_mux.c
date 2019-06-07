#include "i2c_mux.h"

static volatile uint32_t i2c_mutex;

int32_t i2c_mux_acquire(void)
{
  if(i2c_mutex == 0)
  {
	i2c_mutex++;
    if(i2c_mutex == 1)
    {
      return 0;
    }
  }
  return 1;
}

void i2c_mux_release(void)
{
  i2c_mutex = 0;
}

