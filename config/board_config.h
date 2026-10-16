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
/* Initial topology only; a valid saved configuration overrides these defaults.
 * 0: direct connections; 1: sensors on bus 1 use TCA9548A branches. */
#ifndef MUX_ENABLED
#define MUX_ENABLED 0
#endif
#define SCD_BUS 1
#define SHT_BUS 1
#define SDP_BUS 2
#define SDP_CHANNEL 2
#define SDP_ADDRESS 0x25
#define SCD_CHANNEL 0
#define SHT_CHANNEL 1
#define I2C_CLOCK_HZ 400000U
#define UART_BAUD 460800U
