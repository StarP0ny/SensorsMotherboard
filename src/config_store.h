#pragma once
#include <stdbool.h>
#include <stdint.h>
#define CONFIG_BYTES 232
#ifdef __cplusplus
extern "C" {
#endif
bool config_load(uint8_t data[CONFIG_BYTES]);
bool config_save(const uint8_t data[CONFIG_BYTES]);
#ifdef __cplusplus
}
#endif
