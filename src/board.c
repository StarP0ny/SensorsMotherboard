#include "board.h"
#include "board_config.h"
#include "rcc.h"
#include "delay.h"
#include <string.h>
static volatile uint64_t timer_high;
void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM2_IRQHandler(void) {
    TIM_ClearITPendingBit(TIM2,TIM_IT_Update); timer_high+=UINT64_C(65536);
}
uint64_t mono_us(void) {
    uint64_t high,again; uint16_t low; uint16_t pending;
    do {
        high=timer_high; low=(uint16_t)TIM2->CNT;
        pending=TIM2->INTFR & TIM_IT_Update;
        if(pending) low=(uint16_t)TIM2->CNT;
        again=timer_high;
    } while(high!=again);
    /* Timer may have wrapped before its IRQ ran. Re-read low after detecting UIF. */
    if(pending) high+=UINT64_C(65536);
    return high+low;
}
void board_init(void) {
    if(rccInit()!=MCU_OK) while(1) {} /* USB requires the external-clock PLL */
    initDelay();
    GPIO_InitTypeDef g={0}; g.GPIO_Speed=GPIO_Speed_50MHz;
    g.GPIO_Pin=AUTO_BUTTON_PIN; g.GPIO_Mode=GPIO_Mode_IPU; GPIO_Init(AUTO_BUTTON_PORT,&g);
    g.GPIO_Pin=AUTO_LED_PIN; g.GPIO_Mode=GPIO_Mode_Out_PP; GPIO_Init(AUTO_LED_PORT,&g);
    board_auto_led(false);
    g.GPIO_Pin=MUX_RESET_PIN; GPIO_Init(MUX_RESET_PORT,&g); GPIO_ResetBits(MUX_RESET_PORT,MUX_RESET_PIN);
    delayUs(10); GPIO_SetBits(MUX_RESET_PORT,MUX_RESET_PIN);
    TIM_TimeBaseInitTypeDef t={0}; t.TIM_Period=65535;
    t.TIM_CounterMode=TIM_CounterMode_Up; t.TIM_ClockDivision=TIM_CKD_DIV1;
    /* APB1=SYSCLK/2; timer clock is doubled back to 96 MHz. */
    t.TIM_Prescaler=95;
    TIM_TimeBaseInit(TIM2,&t); TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); NVIC_EnableIRQ(TIM2_IRQn); TIM_Cmd(TIM2,ENABLE);
}
bool board_button_pressed(void) { return GPIO_ReadInputDataBit(AUTO_BUTTON_PORT,AUTO_BUTTON_PIN)==Bit_RESET; }
void board_auto_led(bool on) { GPIO_WriteBit(AUTO_LED_PORT,AUTO_LED_PIN,on?Bit_SET:Bit_RESET); }
void board_uid(uint8_t out[12]) { memcpy(out,(const void*)0x1FFFF7E8U,12); }
