#include "sensors.h"
#include "i2c_bus.h"
#include "board.h"
#include "board_config.h"
Sensor sensors[2];
static SampleFn sample_fn;
static ErrorFn error_fn;
uint8_t sensirion_crc(const uint8_t *data,unsigned length) {
    uint8_t crc=0xff;
    while(length--) {
        crc^=*data++;
        for(unsigned j=0;j<8;j++) crc=(uint8_t)((crc&0x80)?(crc<<1)^0x31:crc<<1);
    }
    return crc;
}
static bool command(uint8_t ch,uint8_t addr,uint16_t cmd) {
    uint8_t data[2]={(uint8_t)(cmd>>8),(uint8_t)cmd}; return sensor_write(ch,addr,data,2);
}
static bool read_words(uint8_t ch,uint8_t addr,uint16_t *words,unsigned n,uint16_t id) {
    uint8_t data[9];
    if(!sensor_read(ch,addr,data,n*3)) { error_fn(id,SENSOR_I2C); return false; }
    for(unsigned i=0;i<n;i++) {
        if(sensirion_crc(data+i*3,2)!=data[i*3+2]) { error_fn(id,SENSOR_CRC); return false; }
        words[i]=(uint16_t)((uint16_t)data[i*3]<<8)|data[i*3+1];
    }
    return true;
}
void sensors_init(SampleFn sample,ErrorFn error) {
    sample_fn=sample; error_fn=error;
    sensors[0].period_us=5000000; sensors[1].period_us=1000000;
}
static void scd_poll(bool enabled,uint64_t now) {
    Sensor *s=&sensors[0]; uint16_t words[3];
    /* A reset can leave the powered SCD41 measuring. Always stop before setup. */
    if(!enabled) {
        if(s->active) { command(SCD_CHANNEL,0x62,0x3f86); s->due=now+500000; }
        s->active=false; s->state=0; return;
    }
    if(now<s->due) return;
    switch(s->state) {
    case 0:
        if(!command(SCD_CHANNEL,0x62,0x3f86)) goto failed;
        s->active=true; s->state=1; s->due=mono_us()+500000; break;
    case 1: {
        /* Disable automatic baseline calibration for the laboratory stand. */
        uint8_t cmd[5]={0x24,0x16,0,0,0}; cmd[4]=sensirion_crc(cmd+2,2);
        if(!sensor_write(SCD_CHANNEL,0x62,cmd,5)) goto failed;
        s->state=2; s->due=mono_us()+2000; break;
    }
    case 2:
        if(!command(SCD_CHANNEL,0x62,0x21b1)) goto failed;
        s->state=3; s->due=mono_us()+s->period_us; break;
    case 3:
        if(!command(SCD_CHANNEL,0x62,0xe4b8)) goto failed;
        s->state=4; s->due=mono_us()+2000; break;
    case 4:
        if(!read_words(SCD_CHANNEL,0x62,words,1,1)) goto retry;
        if(!(words[0]&0x7ff)) { s->state=3; s->due=mono_us()+100000; break; }
        if(!command(SCD_CHANNEL,0x62,0xec05)) goto failed;
        s->state=5; s->due=mono_us()+2000; break;
    case 5:
        if(!read_words(SCD_CHANNEL,0x62,words,3,1)) goto retry;
        s->present=true; sample_fn(1,mono_us(),words,3);
        s->state=3; s->due=mono_us()+s->period_us; break;
    }
    return;
failed: error_fn(1,SENSOR_I2C);
retry: s->present=false; s->state=0; s->due=mono_us()+1000000;
}
static void sht_poll(bool enabled,uint64_t now) {
    Sensor *s=&sensors[1]; uint16_t words[2];
    if(!enabled) { s->active=false; s->state=0; return; }
    if(now<s->due) return;
    if(!s->state) {
        uint8_t cmd=0xfd;
        if(!sensor_write(SHT_CHANNEL,0x44,&cmd,1)) {
            error_fn(2,SENSOR_I2C); s->present=false; s->due=mono_us()+1000000; return;
        }
        s->active=true; s->state=1; s->due=mono_us()+10000;
    } else {
        bool ok=read_words(SHT_CHANNEL,0x44,words,2,2); s->present=ok;
        if(ok) sample_fn(2,mono_us(),words,2);
        s->state=0; s->due=mono_us()+s->period_us-10000;
    }
}
void sensors_poll(uint16_t run_mask) {
    uint64_t now=mono_us(); scd_poll((run_mask&1)!=0,now); sht_poll((run_mask&2)!=0,mono_us());
}
