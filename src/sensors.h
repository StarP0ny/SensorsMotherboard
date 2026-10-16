#pragma once
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef void (*SampleFn)(uint16_t id,uint64_t mono,const uint16_t *words,uint16_t count);
typedef void (*ErrorFn)(uint16_t id,uint16_t error);
enum SensorError { SENSOR_I2C=1, SENSOR_CRC=2, SENSOR_NOT_READY=3 };
typedef struct {
    uint16_t id,model;
    uint8_t bus,mux,channel,address;
    uint64_t due;
    uint32_t period_us;
    uint16_t state;
    bool present;
    bool active;
} Sensor;
#define SENSOR_COUNT 16
#define SENSOR_MASK ((1U<<SENSOR_COUNT)-1U)
extern Sensor sensors[SENSOR_COUNT];
void sensors_init(SampleFn sample,ErrorFn error);
void sensors_poll(uint16_t run_mask);
bool sensors_assign(uint16_t id,uint16_t model,uint8_t bus,uint8_t mux,uint8_t channel,uint8_t address,uint32_t period);
bool sensors_remove(uint16_t id);
uint16_t sensors_mask(void);
uint16_t sensors_present(void);
bool sensors_identify(uint8_t bus,uint8_t mux,uint8_t channel,uint8_t address);
void sensors_identify_poll(void);
void sensors_identify_stop(void);
uint8_t sensirion_crc(const uint8_t *data,unsigned length);

#ifdef __cplusplus
}
#endif
