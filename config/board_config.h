#pragma once
#include "ch32v30x.h"
/* External button: PB0 to GND. LED: PB1 -> resistor -> LED -> GND. */
#define AUTO_BUTTON_PORT GPIOB
#define AUTO_BUTTON_PIN GPIO_Pin_0
#define AUTO_LED_PORT GPIOB
#define AUTO_LED_PIN GPIO_Pin_1
#define MUX_RESET_PORT GPIOB
#define MUX_RESET_PIN GPIO_Pin_5
#define MUX_ADDRESS 0x70
#define SCD_CHANNEL 0
#define SHT_CHANNEL 1
#define I2C_CLOCK_HZ 400000U
#define UART_BAUD 460800U
