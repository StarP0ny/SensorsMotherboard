#pragma once
#include <stdint.h>
typedef struct { uint16_t STAR1,STAR2; } I2C_TypeDef;
extern I2C_TypeDef mock_i2c[2];
#define I2C1 (&mock_i2c[0])
#define I2C2 (&mock_i2c[1])
#define GPIOB ((void *)0)
typedef struct { uint16_t GPIO_Pin; unsigned GPIO_Speed,GPIO_Mode; } GPIO_InitTypeDef;
typedef struct { unsigned I2C_ClockSpeed,I2C_Mode,I2C_DutyCycle,I2C_Ack,I2C_AcknowledgedAddress; } I2C_InitTypeDef;
#define GPIO_Pin_0 1
#define GPIO_Pin_1 2
#define GPIO_Pin_5 32
#define GPIO_Pin_6 64
#define GPIO_Pin_7 128
#define GPIO_Pin_10 1024
#define GPIO_Pin_11 2048
#define GPIO_Speed_50MHz 0
#define GPIO_Mode_AF_OD 0
#define GPIO_Mode_Out_OD 0
#define I2C_Mode_I2C 0
#define I2C_DutyCycle_2 0
#define I2C_Ack_Enable 0
#define I2C_AcknowledgedAddress_7bit 0
#define I2C_STAR1_SB 1
#define I2C_STAR1_ADDR 2
#define I2C_STAR1_TXE 4
#define I2C_STAR1_BTF 8
#define I2C_STAR1_RXNE 16
#define I2C_STAR1_AF 32
#define I2C_STAR1_BERR 64
#define I2C_STAR1_ARLO 128
#define I2C_STAR2_BUSY 1
#define I2C_NACKPosition_Current 0
#define I2C_NACKPosition_Next 1
#define I2C_Direction_Receiver 1
#define I2C_Direction_Transmitter 0
#define ENABLE 1
#define DISABLE 0
void GPIO_Init(void *,GPIO_InitTypeDef *);
void GPIO_ResetBits(void *,uint16_t);
void GPIO_SetBits(void *,uint16_t);
void I2C_Init(I2C_TypeDef *,I2C_InitTypeDef *);
void I2C_Cmd(I2C_TypeDef *,int);
void I2C_DeInit(I2C_TypeDef *);
void I2C_GenerateSTOP(I2C_TypeDef *,int);
void I2C_GenerateSTART(I2C_TypeDef *,int);
void I2C_AcknowledgeConfig(I2C_TypeDef *,int);
void I2C_NACKPositionConfig(I2C_TypeDef *,int);
void I2C_Send7bitAddress(I2C_TypeDef *,uint8_t,int);
void I2C_SendData(I2C_TypeDef *,uint8_t);
uint8_t I2C_ReceiveData(I2C_TypeDef *);
