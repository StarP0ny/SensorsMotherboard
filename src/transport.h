#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
void transport_init(void);
void transport_poll(void);
size_t transport_read(unsigned port,uint8_t *out,size_t capacity);
bool transport_send(unsigned port,const uint8_t *data,size_t length,bool control);
uint32_t transport_drops(void);
bool usb_app_rx(const uint8_t *data,uint16_t length);
void usb_app_reset(void);
