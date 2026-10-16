#include "sensors.h"
#include "i2c_bus.h"
#include "board.h"
#include "board_config.h"
Sensor sensors[SENSOR_COUNT];
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
static bool command(const Sensor &s,uint16_t cmd) {
    uint8_t data[2]={(uint8_t)(cmd>>8),(uint8_t)cmd}; return sensor_write(s.bus,s.mux,s.channel,s.address,data,2);
}
static bool read_words(const Sensor &s,uint16_t *words,unsigned n) {
    uint8_t data[9];
    if(!sensor_read(s.bus,s.mux,s.channel,s.address,data,n*3)) { error_fn(s.id,SENSOR_I2C); return false; }
    for(unsigned i=0;i<n;i++) {
        if(sensirion_crc(data+i*3,2)!=data[i*3+2]) { error_fn(s.id,SENSOR_CRC); return false; }
        words[i]=(uint16_t)((uint16_t)data[i*3]<<8)|data[i*3+1];
    }
    return true;
}
void sensors_init(SampleFn sample,ErrorFn error) {
    sample_fn=sample; error_fn=error;
    for(auto &s:sensors) s={};
    i2c_muxes[0]=i2c_muxes[1]=0;
    if(MUX_ENABLED) i2c_muxes[0]=1U<<(MUX_ADDRESS-0x70);
    sensors_assign(1,1,SCD_BUS,MUX_ENABLED&&SCD_BUS==1?MUX_ADDRESS:0,MUX_ENABLED&&SCD_BUS==1?SCD_CHANNEL:255,0x62,5000000);
    sensors_assign(2,2,SHT_BUS,MUX_ENABLED&&SHT_BUS==1?MUX_ADDRESS:0,MUX_ENABLED&&SHT_BUS==1?SHT_CHANNEL:255,0x44,1000000);
    sensors_assign(3,3,SDP_BUS,MUX_ENABLED&&SDP_BUS==1?MUX_ADDRESS:0,MUX_ENABLED&&SDP_BUS==1?SDP_CHANNEL:255,SDP_ADDRESS,100000);
}
static void scd_poll(Sensor *s,bool enabled,uint64_t now) { uint16_t words[3];
    /* A reset can leave the powered SCD41 measuring. Always stop before setup. */
    if(!enabled) {
        if(s->active) { command(*s,0x3f86); s->due=now+500000; }
        s->active=false; s->state=0; return;
    }
    if(now<s->due) return;
    switch(s->state) {
    case 0:
        if(!command(*s,0x3f86)) goto failed;
        s->active=true; s->state=1; s->due=mono_us()+500000; break;
    case 1: {
        /* Disable automatic baseline calibration for the laboratory stand. */
        uint8_t cmd[5]={0x24,0x16,0,0,0}; cmd[4]=sensirion_crc(cmd+2,2);
        if(!sensor_write(s->bus,s->mux,s->channel,s->address,cmd,5)) goto failed;
        s->state=2; s->due=mono_us()+2000; break;
    }
    case 2:
        if(!command(*s,0x21b1)) goto failed;
        s->state=3; s->due=mono_us()+s->period_us; break;
    case 3:
        if(!command(*s,0xe4b8)) goto failed;
        s->state=4; s->due=mono_us()+2000; break;
    case 4:
        if(!read_words(*s,words,1)) goto retry;
        if(!(words[0]&0x7ff)) { s->state=3; s->due=mono_us()+100000; break; }
        if(!command(*s,0xec05)) goto failed;
        s->state=5; s->due=mono_us()+2000; break;
    case 5:
        if(!read_words(*s,words,3)) goto retry;
        s->present=true; sample_fn(s->id,mono_us(),words,3);
        s->state=3; s->due=mono_us()+s->period_us; break;
    }
    return;
failed: error_fn(s->id,SENSOR_I2C);
retry: s->present=false; s->state=0; s->due=mono_us()+1000000;
}
static void sht_poll(Sensor *s,bool enabled,uint64_t now) { uint16_t words[2];
    if(!enabled) { s->active=false; s->state=0; return; }
    if(now<s->due) return;
    if(!s->state) {
        uint8_t cmd=0xfd;
        if(!sensor_write(s->bus,s->mux,s->channel,s->address,&cmd,1)) {
            error_fn(s->id,SENSOR_I2C); s->present=false; s->due=mono_us()+1000000; return;
        }
        s->active=true; s->state=1; s->due=mono_us()+10000;
    } else {
        bool ok=read_words(*s,words,2); s->present=ok;
        if(ok) sample_fn(s->id,mono_us(),words,2);
        s->state=0; s->due=mono_us()+s->period_us-10000;
    }
}
static void sdp_poll(Sensor *s,bool enabled,uint64_t now) { uint16_t words[3];
    if(!enabled) {
        if(s->active) {
            if(s->state==3 && now<s->due) return;
            if(!command(*s,0x3ff9)) {
                error_fn(s->id,SENSOR_I2C); s->present=false;
                s->state=3; s->due=mono_us()+1000000; return;
            }
            s->due=mono_us()+1000;
        }
        s->active=false; s->state=0; return;
    }
    if(now<s->due) return;
    switch(s->state) {
    case 0:
    case 3: /* Retry a failed stop before restarting acquisition. */
        /* Stop first: the sensor may still be measuring after an MCU reset. */
        if(!command(*s,0x3ff9)) goto failed;
        s->active=false; s->state=1; s->due=mono_us()+1000; break;
    case 1:
        /* Differential pressure compensation, average until read. */
        if(!command(*s,0x3615)) goto failed;
        s->active=true; s->state=2; s->due=mono_us()+20000; break;
    case 2:
        if(!read_words(*s,words,3)) goto retry;
        if(!words[2]) { error_fn(s->id,SENSOR_NOT_READY); goto retry; }
        s->present=true; sample_fn(s->id,mono_us(),words,3);
        s->due=mono_us()+s->period_us; break;
    }
    return;
failed: error_fn(s->id,SENSOR_I2C);
retry: s->present=false; s->state=0; s->due=mono_us()+1000000;
}

static void poll(Sensor &s,bool enabled) {
    switch(s.model) {
    case 1: scd_poll(&s,enabled,mono_us()); break;
    case 2: sht_poll(&s,enabled,mono_us()); break;
    case 3: sdp_poll(&s,enabled,mono_us()); break;
    }
}
bool sensors_assign(uint16_t id,uint16_t model,uint8_t bus,uint8_t mux,uint8_t channel,uint8_t address,uint32_t period) {
    if(id<1 || id>SENSOR_COUNT || model<1 || model>3 || !i2c_path_valid(bus,mux,channel)) return false;
    if((model==1 && address!=0x62) || (model==2 && address!=0x44 && address!=0x45 && address!=0x46) ||
       (model==3 && address!=0x25 && address!=0x26)) return false;
    if(period<(model==1?5000000U:10000U) || period>60000000U) return false;
    for(const auto &other:sensors) {
        if(other.model && other.id!=id && other.bus==bus && other.mux==mux && other.channel==channel && other.address==address) return false;
    }
    Sensor &s=sensors[id-1];
    if(s.active) return false;
    s={}; s.id=id; s.model=model; s.bus=bus; s.mux=mux; s.channel=channel; s.address=address;
    s.period_us=period; s.due=mono_us()+(model==3?25000:0); return true;
}
bool sensors_remove(uint16_t id) {
    if(id<1 || id>SENSOR_COUNT || sensors[id-1].active) return false;
    sensors[id-1]={}; return true;
}
uint16_t sensors_mask(void) {
    uint16_t mask=0; for(unsigned i=0;i<SENSOR_COUNT;i++) if(sensors[i].model) mask|=1U<<i;
    return mask;
}
uint16_t sensors_present(void) {
    uint16_t mask=0; for(unsigned i=0;i<SENSOR_COUNT;i++) if(sensors[i].model && sensors[i].present) mask|=1U<<i;
    return mask;
}
void sensors_poll(uint16_t run_mask) {
    for(auto &s:sensors) if(s.model) poll(s,(run_mask&(1U<<(s.id-1)))!=0);
}
static Sensor probe;
bool sensors_identify(uint8_t bus,uint8_t mux,uint8_t channel,uint8_t address) {
    if(!i2c_path_valid(bus,mux,channel)) return false;
    probe={}; probe.bus=bus; probe.mux=mux; probe.channel=channel; probe.address=address;
    probe.model=address==0x62?1:(address==0x44 || address==0x45 || address==0x46)?2:(address==0x25 || address==0x26)?3:0;
    probe.period_us=probe.model==1?5000000:100000;
    return probe.model!=0;
}
void sensors_identify_poll(void) { poll(probe,true); }
void sensors_identify_stop(void) { probe.due=0; poll(probe,false); probe={}; }
