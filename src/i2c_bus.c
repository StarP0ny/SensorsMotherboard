#include "i2c_bus.h"
#include "board.h"
#include "board_config.h"
#include "delay.h"
/* Transactions run synchronously in the main loop; no ISR uses this context. */
static I2C_TypeDef *device;
static uint16_t scl,sda;
static uint8_t selected_bus;
static bool activate(uint8_t bus) {
    if(bus!=1 && bus!=2) return false;
    selected_bus=bus; device=bus==1?I2C1:I2C2;
    scl=bus==1?GPIO_Pin_6:GPIO_Pin_10; sda=bus==1?GPIO_Pin_7:GPIO_Pin_11;
    return true;
}
static void configure(void) {
    GPIO_InitTypeDef g={scl|sda,GPIO_Speed_50MHz,GPIO_Mode_AF_OD};
    GPIO_Init(GPIOB,&g);
    I2C_InitTypeDef c={0}; c.I2C_ClockSpeed=I2C_CLOCK_HZ; c.I2C_Mode=I2C_Mode_I2C;
    c.I2C_DutyCycle=I2C_DutyCycle_2; c.I2C_Ack=I2C_Ack_Enable;
    c.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
    I2C_Init(device,&c); I2C_Cmd(device,ENABLE);
}
void i2c_bus_init(void) { activate(1); configure(); activate(2); configure(); }
static bool wait_bits(uint16_t bits) {
    uint64_t deadline=mono_us()+2000;
    while((device->STAR1&bits)!=bits) {
        if((device->STAR1&(I2C_STAR1_AF|I2C_STAR1_BERR|I2C_STAR1_ARLO)) || mono_us()>=deadline) return false;
    }
    return true;
}
static void clear_addr(void) { volatile uint16_t v=device->STAR1; v=device->STAR2; (void)v; }
static void recover(void) {
    I2C_GenerateSTOP(device,ENABLE); I2C_Cmd(device,DISABLE);
    /* Isolate a wedged branch before clocking the upstream bus. */
    if(selected_bus==1 && MUX_ENABLED) { GPIO_ResetBits(MUX_RESET_PORT,MUX_RESET_PIN); delayUs(10); }
    GPIO_InitTypeDef g={scl|sda,GPIO_Speed_50MHz,GPIO_Mode_Out_OD}; GPIO_Init(GPIOB,&g);
    GPIO_SetBits(GPIOB,scl|sda);
    for(unsigned i=0;i<9;i++) {
        GPIO_ResetBits(GPIOB,scl); delayUs(5); GPIO_SetBits(GPIOB,scl); delayUs(5);
    }
    GPIO_ResetBits(GPIOB,sda); delayUs(5); GPIO_SetBits(GPIOB,sda);
    if(selected_bus==1 && MUX_ENABLED) GPIO_SetBits(MUX_RESET_PORT,MUX_RESET_PIN);
    I2C_DeInit(device); configure();
}
static bool start(uint8_t addr,bool read) {
    uint64_t deadline=mono_us()+2000;
    while(device->STAR2&I2C_STAR2_BUSY) if(mono_us()>=deadline) return false;
    I2C_AcknowledgeConfig(device,ENABLE); I2C_NACKPositionConfig(device,I2C_NACKPosition_Current);
    I2C_GenerateSTART(device,ENABLE);
    if(!wait_bits(I2C_STAR1_SB)) return false;
    I2C_Send7bitAddress(device,(uint8_t)(addr<<1),read?I2C_Direction_Receiver:I2C_Direction_Transmitter);
    return wait_bits(I2C_STAR1_ADDR);
}
static bool write_raw(uint8_t addr,const uint8_t *data,size_t len) {
    if(!start(addr,false)) return false;
    clear_addr();
    for(size_t i=0;i<len;i++) {
        if(!wait_bits(I2C_STAR1_TXE)) return false;
        I2C_SendData(device,data[i]);
    }
    if(!wait_bits(I2C_STAR1_BTF)) return false;
    I2C_GenerateSTOP(device,ENABLE); return true;
}
uint8_t i2c_muxes[2];
bool i2c_path_valid(uint8_t bus,uint8_t mux,uint8_t channel) {
    if(bus!=1 && bus!=2) return false;
    if(!mux) return channel==255;
    return mux>=0x70 && mux<=0x77 && channel<8 && (i2c_muxes[bus-1]&(1U<<(mux-0x70)));
}
bool i2c_mux_set(uint8_t bus,uint8_t address,bool enabled) {
    if((bus!=1 && bus!=2) || address<0x70 || address>0x77) return false;
    uint8_t bit=1U<<(address-0x70);
    if(!enabled && (i2c_muxes[bus-1]&bit)) {
        uint8_t mask=0;
        activate(bus);
        /* Keep the route registered until the physical branches are isolated. */
        if(!write_raw(address,&mask,1)) { recover(); return false; }
    }
    if(enabled) i2c_muxes[bus-1]|=bit; else i2c_muxes[bus-1]&=(uint8_t)~bit;
    return true;
}
static bool select_channel(uint8_t mux,uint8_t channel) {
    if(!i2c_path_valid(selected_bus,mux,channel)) return false;
    /* Isolate every configured mux, including for direct upstream access. */
    for(unsigned i=0;i<8;i++) if(i2c_muxes[selected_bus-1]&(1U<<i)) {
        uint8_t mask=mux==0x70+i?(uint8_t)(1U<<channel):0;
        if(!write_raw(0x70+i,&mask,1)) return false;
    }
    return true;
}
bool sensor_write(uint8_t bus,uint8_t mux,uint8_t channel,uint8_t addr,const uint8_t *data,size_t len) {
    if(!activate(bus)) return false;
    if(select_channel(mux,channel) && write_raw(addr,data,len)) return true;
    recover(); return false;
}
int i2c_probe(uint8_t bus,uint8_t mux,uint8_t channel,uint8_t addr) {
    if(!activate(bus)) return -1;
    if(!select_channel(mux,channel)) { recover(); return -1; }
    if(start(addr,false)) {
        clear_addr(); I2C_GenerateSTOP(device,ENABLE); return 1;
    }
    /* NACK is normal for an absent address; don't reset the whole bus. */
    if((device->STAR1&I2C_STAR1_AF) && !(device->STAR1&(I2C_STAR1_BERR|I2C_STAR1_ARLO))) {
        I2C_GenerateSTOP(device,ENABLE); device->STAR1&=(uint16_t)~I2C_STAR1_AF;
        return 0;
    }
    recover(); return -1;
}
bool sensor_read(uint8_t bus,uint8_t mux,uint8_t channel,uint8_t addr,uint8_t *data,size_t len) {
    if(!activate(bus)) return false;
    if(!len || !select_channel(mux,channel) || !start(addr,true)) goto fail;
    if(len==1) {
        I2C_AcknowledgeConfig(device,DISABLE); clear_addr(); I2C_GenerateSTOP(device,ENABLE);
        if(!wait_bits(I2C_STAR1_RXNE)) goto fail;
        *data=I2C_ReceiveData(device);
    } else if(len==2) {
        I2C_NACKPositionConfig(device,I2C_NACKPosition_Next); I2C_AcknowledgeConfig(device,DISABLE); clear_addr();
        if(!wait_bits(I2C_STAR1_BTF)) goto fail;
        I2C_GenerateSTOP(device,ENABLE);
        *data++=I2C_ReceiveData(device); *data=I2C_ReceiveData(device);
    } else {
        clear_addr();
        while(len>3) { if(!wait_bits(I2C_STAR1_RXNE)) goto fail; *data++=I2C_ReceiveData(device); len--; }
        if(!wait_bits(I2C_STAR1_BTF)) goto fail;
        I2C_AcknowledgeConfig(device,DISABLE); *data++=I2C_ReceiveData(device);
        if(!wait_bits(I2C_STAR1_BTF)) goto fail;
        I2C_GenerateSTOP(device,ENABLE); *data++=I2C_ReceiveData(device); *data=I2C_ReceiveData(device);
    }
    I2C_AcknowledgeConfig(device,ENABLE); I2C_NACKPositionConfig(device,I2C_NACKPosition_Current); return true;
fail: recover(); return false;
}
