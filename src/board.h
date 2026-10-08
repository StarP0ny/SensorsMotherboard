#pragma once
#include <stdbool.h>
#include <stdint.h>
void board_init(void);
uint64_t mono_us(void);
bool board_button_pressed(void);
void board_auto_led(bool on);
void board_uid(uint8_t out[12]);
