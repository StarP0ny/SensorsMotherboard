#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
extern uint8_t i2c_muxes[2];
bool i2c_path_valid(uint8_t bus,uint8_t mux,uint8_t channel);
bool i2c_mux_set(uint8_t bus,uint8_t address,bool enabled);
void i2c_bus_init(void);
/* 1 ACK, 0 NACK, -1 bus/mux error. No data bytes are written. */
int i2c_probe(uint8_t bus,uint8_t mux,uint8_t channel,uint8_t addr);
bool sensor_write(uint8_t bus,uint8_t mux,uint8_t channel,uint8_t addr,const uint8_t *data,size_t len);
bool sensor_read(uint8_t bus,uint8_t mux,uint8_t channel,uint8_t addr,uint8_t *data,size_t len);

#ifdef __cplusplus
}
#endif
