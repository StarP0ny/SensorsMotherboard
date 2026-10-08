#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef void (*SampleFn)(uint16_t id,uint64_t mono,const uint16_t *words,uint16_t count);
typedef void (*ErrorFn)(uint16_t id,uint16_t error);
enum SensorError { SENSOR_I2C=1, SENSOR_CRC=2, SENSOR_NOT_READY=3 };
typedef struct {
    uint64_t due;
    uint32_t period_us;
    uint16_t state;
    bool present;
    bool active;
} Sensor;
extern Sensor sensors[2];
void sensors_init(SampleFn sample,ErrorFn error);
void sensors_poll(uint16_t run_mask);
uint8_t sensirion_crc(const uint8_t *data,unsigned length);
