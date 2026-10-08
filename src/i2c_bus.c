#include "i2c_bus.h"
#include "board.h"
#include "board_config.h"
#include "delay.h"
static void configure(void) {
    GPIO_InitTypeDef g={GPIO_Pin_6|GPIO_Pin_7,GPIO_Speed_50MHz,GPIO_Mode_AF_OD};
    GPIO_Init(GPIOB,&g);
    I2C_InitTypeDef c={0}; c.I2C_ClockSpeed=I2C_CLOCK_HZ; c.I2C_Mode=I2C_Mode_I2C;
    c.I2C_DutyCycle=I2C_DutyCycle_2; c.I2C_Ack=I2C_Ack_Enable;
    c.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1,&c); I2C_Cmd(I2C1,ENABLE);
}
void i2c_bus_init(void) { configure(); }
static bool wait_bits(uint16_t bits) {
    uint64_t deadline=mono_us()+2000;
    while((I2C1->STAR1&bits)!=bits) {
        if((I2C1->STAR1&(I2C_STAR1_AF|I2C_STAR1_BERR|I2C_STAR1_ARLO)) || mono_us()>=deadline) return false;
    }
    return true;
}
static void clear_addr(void) { volatile uint16_t v=I2C1->STAR1; v=I2C1->STAR2; (void)v; }
static void recover(void) {
    I2C_GenerateSTOP(I2C1,ENABLE); I2C_Cmd(I2C1,DISABLE);
    /* Isolate a wedged branch before clocking the upstream bus. */
    GPIO_ResetBits(MUX_RESET_PORT,MUX_RESET_PIN); delayUs(10);
    GPIO_InitTypeDef g={GPIO_Pin_6|GPIO_Pin_7,GPIO_Speed_50MHz,GPIO_Mode_Out_OD}; GPIO_Init(GPIOB,&g);
    GPIO_SetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7);
    for(unsigned i=0;i<9;i++) {
        GPIO_ResetBits(GPIOB,GPIO_Pin_6); delayUs(5); GPIO_SetBits(GPIOB,GPIO_Pin_6); delayUs(5);
    }
    GPIO_ResetBits(GPIOB,GPIO_Pin_7); delayUs(5); GPIO_SetBits(GPIOB,GPIO_Pin_7);
    GPIO_SetBits(MUX_RESET_PORT,MUX_RESET_PIN); I2C_DeInit(I2C1); configure();
}
static bool start(uint8_t addr,bool read) {
    uint64_t deadline=mono_us()+2000;
    while(I2C1->STAR2&I2C_STAR2_BUSY) if(mono_us()>=deadline) return false;
    I2C_AcknowledgeConfig(I2C1,ENABLE); I2C_NACKPositionConfig(I2C1,I2C_NACKPosition_Current);
    I2C_GenerateSTART(I2C1,ENABLE);
    if(!wait_bits(I2C_STAR1_SB)) return false;
    I2C_Send7bitAddress(I2C1,(uint8_t)(addr<<1),read?I2C_Direction_Receiver:I2C_Direction_Transmitter);
    return wait_bits(I2C_STAR1_ADDR);
}
static bool write_raw(uint8_t addr,const uint8_t *data,size_t len) {
    if(!start(addr,false)) return false;
    clear_addr();
    for(size_t i=0;i<len;i++) {
        if(!wait_bits(I2C_STAR1_TXE)) return false;
        I2C_SendData(I2C1,data[i]);
    }
    if(!wait_bits(I2C_STAR1_BTF)) return false;
    I2C_GenerateSTOP(I2C1,ENABLE); return true;
}
static bool select_channel(uint8_t channel) {
    uint8_t mask=(uint8_t)(1U<<channel); return channel<8 && write_raw(MUX_ADDRESS,&mask,1);
}
bool sensor_write(uint8_t channel,uint8_t addr,const uint8_t *data,size_t len) {
    if(select_channel(channel) && write_raw(addr,data,len)) return true;
    recover(); return false;
}
bool sensor_read(uint8_t channel,uint8_t addr,uint8_t *data,size_t len) {
    if(!len || !select_channel(channel) || !start(addr,true)) goto fail;
    if(len==1) {
        I2C_AcknowledgeConfig(I2C1,DISABLE); clear_addr(); I2C_GenerateSTOP(I2C1,ENABLE);
        if(!wait_bits(I2C_STAR1_RXNE)) goto fail;
        *data=I2C_ReceiveData(I2C1);
    } else if(len==2) {
        I2C_NACKPositionConfig(I2C1,I2C_NACKPosition_Next); I2C_AcknowledgeConfig(I2C1,DISABLE); clear_addr();
        if(!wait_bits(I2C_STAR1_BTF)) goto fail;
        I2C_GenerateSTOP(I2C1,ENABLE);
        *data++=I2C_ReceiveData(I2C1); *data=I2C_ReceiveData(I2C1);
    } else {
        clear_addr();
        while(len>3) { if(!wait_bits(I2C_STAR1_RXNE)) goto fail; *data++=I2C_ReceiveData(I2C1); len--; }
        if(!wait_bits(I2C_STAR1_BTF)) goto fail;
        I2C_AcknowledgeConfig(I2C1,DISABLE); *data++=I2C_ReceiveData(I2C1);
        if(!wait_bits(I2C_STAR1_BTF)) goto fail;
        I2C_GenerateSTOP(I2C1,ENABLE); *data++=I2C_ReceiveData(I2C1); *data=I2C_ReceiveData(I2C1);
    }
    I2C_AcknowledgeConfig(I2C1,ENABLE); I2C_NACKPositionConfig(I2C1,I2C_NACKPosition_Current); return true;
fail: recover(); return false;
}
