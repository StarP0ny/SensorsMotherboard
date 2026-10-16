#include "i2c_bus.h"
#include "ch32v30x.h"
#include <assert.h>
I2C_TypeDef mock_i2c[2];
static uint8_t address,values[8],buses[8];
static unsigned writes;
static int fail;
uint64_t mono_us(void) { static uint64_t now; return ++now; }
void delayUs(unsigned n) { (void)n; }
void GPIO_Init(void *g,GPIO_InitTypeDef *c) { (void)g; (void)c; }
void GPIO_ResetBits(void *g,uint16_t p) { (void)g; (void)p; }
void GPIO_SetBits(void *g,uint16_t p) { (void)g; (void)p; }
void I2C_Init(I2C_TypeDef *d,I2C_InitTypeDef *c) { (void)c; d->STAR1=0; d->STAR2=0; }
void I2C_Cmd(I2C_TypeDef *d,int v) { (void)d; (void)v; }
void I2C_DeInit(I2C_TypeDef *d) { d->STAR1=0; d->STAR2=0; }
void I2C_GenerateSTOP(I2C_TypeDef *d,int v) { (void)v; d->STAR2=0; }
void I2C_GenerateSTART(I2C_TypeDef *d,int v) { (void)v; d->STAR1=I2C_STAR1_SB; }
void I2C_AcknowledgeConfig(I2C_TypeDef *d,int v) { (void)d; (void)v; }
void I2C_NACKPositionConfig(I2C_TypeDef *d,int v) { (void)d; (void)v; }
void I2C_Send7bitAddress(I2C_TypeDef *d,uint8_t a,int v) {
    (void)v; address=a>>1;
    d->STAR1=fail?I2C_STAR1_AF:(I2C_STAR1_ADDR|I2C_STAR1_TXE|I2C_STAR1_BTF);
}
void I2C_SendData(I2C_TypeDef *d,uint8_t value) {
    assert(address==0x70); assert(writes<8);
    buses[writes]=(uint8_t)(d==I2C1?1:2); values[writes++]=value;
}
uint8_t I2C_ReceiveData(I2C_TypeDef *d) { (void)d; return 0; }
int main(void) {
    i2c_bus_init();
    assert(i2c_mux_set(2,0x70,true));
    assert(i2c_probe(2,0x70,3,0x44)==1);
    assert(writes==1 && buses[0]==2 && values[0]==8);
    assert(i2c_mux_set(2,0x70,false));
    assert(writes==2 && buses[1]==2 && values[1]==0);
    assert(!i2c_path_valid(2,0x70,3));
    assert(i2c_mux_set(1,0x70,true));
    fail=1;
    assert(!i2c_mux_set(1,0x70,false));
    assert(i2c_path_valid(1,0x70,3));
    assert(writes==2);
    fail=0;
    assert(i2c_mux_set(1,0x70,false));
    assert(writes==3 && buses[2]==1 && values[2]==0);
    assert(!i2c_path_valid(1,0x70,3));
    assert(i2c_mux_set(1,0x70,false));
    assert(writes==3);
    return 0;
}
