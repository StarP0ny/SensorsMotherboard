#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
void i2c_bus_init(void);
bool sensor_write(uint8_t channel,uint8_t addr,const uint8_t *data,size_t len);
bool sensor_read(uint8_t channel,uint8_t addr,uint8_t *data,size_t len);
