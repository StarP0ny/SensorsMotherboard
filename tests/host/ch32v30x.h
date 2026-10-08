#pragma once
#include <stdint.h>
typedef struct { uint32_t RSTSCKR; } MockRCC;
extern MockRCC mock_rcc;
#define RCC (&mock_rcc)
