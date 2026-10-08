#include "board.h"
#include "transport.h"
#include "sensors.h"
#include "app.h"
#include "i2c_bus.h"
#include "ch32v30x.h"
#include <string.h>
MockRCC mock_rcc;
static uint64_t now;
static bool button,led;
static uint8_t incoming[2048],outgoing[65536];
static size_t incoming_len,outgoing_len;
static uint16_t last_command[2];
static int fail_i2c,bad_crc;
static uint16_t asc=99;
uint64_t mono_us(void) { return now; }
bool board_button_pressed(void) { return button; }
void board_auto_led(bool on) { led=on; }
void board_uid(uint8_t out[12]) { memset(out,0x12,12); }
uint32_t transport_drops(void) { return 0; }
size_t transport_read(unsigned port,uint8_t *out,size_t capacity) {
    if(port) return 0;
    size_t n=incoming_len<capacity?incoming_len:capacity;
    memcpy(out,incoming,n); memmove(incoming,incoming+n,incoming_len-n); incoming_len-=n; return n;
}
bool transport_send(unsigned port,const uint8_t *data,size_t length,bool control) {
    (void)control;
    if(port || outgoing_len+length>sizeof(outgoing)) return false;
    memcpy(outgoing+outgoing_len,data,length); outgoing_len+=length; return true;
}
bool sensor_write(uint8_t channel,uint8_t addr,const uint8_t *data,size_t n) {
    (void)addr;
    if(fail_i2c) return false;
    last_command[channel]=n==1?data[0]:(uint16_t)((data[0]<<8)|data[1]);
    if(last_command[channel]==0x2416 && n==5) asc=(data[2]<<8)|data[3];
    return true;
}
bool sensor_read(uint8_t channel,uint8_t addr,uint8_t *data,size_t n) {
    (void)addr;
    if(fail_i2c) return false;
    uint16_t words[3]={0};
    if(channel==0 && last_command[channel]==0xe4b8) words[0]=1;
    else if(channel==0 && last_command[channel]==0xec05) { words[0]=600; words[1]=26000; words[2]=30000; }
    else if(channel==1 && last_command[channel]==0xfd) { words[0]=25000; words[1]=31000; }
    else return false;
    for(size_t i=0;i<n/3;i++) {
        data[i*3]=(uint8_t)(words[i]>>8); data[i*3+1]=(uint8_t)words[i];
        data[i*3+2]=sensirion_crc(data+i*3,2)^(bad_crc?1:0);
    }
    return true;
}
void host_init(void) { app_init(); }
void host_step(uint64_t time) { now=time; app_poll(); }
void host_button(int pressed) { button=pressed!=0; }
int host_led(void) { return led; }
void host_fail(int i2c,int crc) { fail_i2c=i2c; bad_crc=crc; }
uint16_t host_asc(void) { return asc; }
void host_feed(const uint8_t *data,size_t n) {
    if(n+incoming_len<=sizeof(incoming)) { memcpy(incoming+incoming_len,data,n); incoming_len+=n; }
}
size_t host_take(uint8_t *out) { size_t n=outgoing_len; memcpy(out,outgoing,n); outgoing_len=0; return n; }
