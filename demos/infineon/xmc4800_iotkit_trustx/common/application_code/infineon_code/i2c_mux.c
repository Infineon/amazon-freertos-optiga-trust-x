#include "i2c_mux.h"

#include "FreeRTOS.h"
#include "semphr.h"

static volatile uint32_t i2c_mutex;
extern SemaphoreHandle_t xIicSemaphoreHandle;

int32_t i2c_mux_acquire(void)
{
	if ( xSemaphoreTake(xIicSemaphoreHandle, portMAX_DELAY) == pdTRUE )
	    return 0;
}

void i2c_mux_release(void)
{
	xSemaphoreGive(xIicSemaphoreHandle);
}

