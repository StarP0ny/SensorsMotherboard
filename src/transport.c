#include "transport.h"
#include "protocol.h"
#include "board_config.h"
#include "ch32v30x_usbfs_device.h"
#include <string.h>
#define RX_SIZE 2048U
#define TX_SLOTS 6U
typedef struct { uint8_t data[PROTO_MAX_FRAME]; uint16_t length,offset; } Slot;
typedef struct {
    volatile uint16_t head,tail;
    uint8_t rx[RX_SIZE];
    Slot tx[TX_SLOTS];
    unsigned front,count;
} Port;
static Port ports[2];
static uint32_t dropped;
static volatile uint32_t uart_rx_drops;
static bool usb_pending;
static volatile bool usb_reset;
static bool rx_put(unsigned port,const uint8_t *data,size_t length) {
    Port *p=&ports[port]; uint16_t head=p->head;
    unsigned free=(p->tail+RX_SIZE-head-1)%RX_SIZE;
    if(length>free) return false;
    while(length--) { p->rx[head]=*data++; head=(head+1)%RX_SIZE; }
    __asm volatile("" ::: "memory"); p->head=head; return true;
}
bool usb_app_rx(const uint8_t *data,uint16_t length) {
    if(rx_put(0,data,length)) return true;
    usb_pending=true; return false;
}
void usb_app_reset(void) { usb_reset=true; }
size_t transport_read(unsigned port,uint8_t *out,size_t capacity) {
    Port *p=&ports[port]; size_t n=0;
    while(n<capacity && p->tail!=p->head) { out[n++]=p->rx[p->tail]; p->tail=(p->tail+1)%RX_SIZE; }
    return n;
}
bool transport_send(unsigned port,const uint8_t *data,size_t length,bool control) {
    Port *p=&ports[port];
    /* Reserve two slots for responses and heartbeat. Never truncate a frame. */
    if(!length || length>PROTO_MAX_FRAME || p->count>=(control?TX_SLOTS:TX_SLOTS-2)) { dropped++; return false; }
    Slot *s=&p->tx[(p->front+p->count)%TX_SLOTS]; memcpy(s->data,data,length);
    s->length=(uint16_t)length; s->offset=0; p->count++; return true;
}
uint32_t transport_drops(void) { return dropped+uart_rx_drops; }
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void) {
    uint16_t status=USART1->STATR;
    if(status&(USART_STATR_RXNE|USART_STATR_ORE)) {
        uint8_t b=(uint8_t)USART1->DATAR;
        if((status&USART_STATR_RXNE) && !rx_put(1,&b,1)) uart_rx_drops++;
        if(status&USART_STATR_ORE) uart_rx_drops++;
    }
}
void transport_init(void) {
    GPIO_InitTypeDef g={GPIO_Pin_9,GPIO_Speed_50MHz,GPIO_Mode_AF_PP}; GPIO_Init(GPIOA,&g);
    g.GPIO_Pin=GPIO_Pin_10; g.GPIO_Mode=GPIO_Mode_IN_FLOATING; GPIO_Init(GPIOA,&g);
    USART_InitTypeDef c={0}; c.USART_BaudRate=UART_BAUD; c.USART_WordLength=USART_WordLength_8b;
    c.USART_StopBits=USART_StopBits_1; c.USART_Parity=USART_Parity_No;
    c.USART_HardwareFlowControl=USART_HardwareFlowControl_None; c.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
    USART_Init(USART1,&c); USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
    NVIC_EnableIRQ(USART1_IRQn); USART_Cmd(USART1,ENABLE);
    USBFS_RCC_Init(); USBFS_Device_Init(ENABLE);
}
void transport_poll(void) {
    if(usb_reset) { ports[0].count=0; ports[0].front=0; ports[0].tail=ports[0].head; usb_pending=false; usb_reset=false; }
    if(usb_pending && rx_put(0,USBFS_EP2_Buf,USBFSD->RX_LEN)) {
        usb_pending=false; USBFSD->UEP2_RX_CTRL=(USBFSD->UEP2_RX_CTRL&~USBFS_UEP_R_RES_MASK)|USBFS_UEP_R_RES_ACK;
    }
    for(unsigned i=0;i<2;i++) {
        Port *p=&ports[i]; if(!p->count) continue; Slot *s=&p->tx[p->front];
        if(i==0) {
            if(!USBFS_DevEnumStatus || USBFS_Endp_Busy[DEF_UEP3]) continue;
            /* Retain the slot until the final USB IN packet has completed. */
            if(s->offset<s->length) {
                uint16_t n=s->length-s->offset; if(n>64) n=64;
                if(!USBFS_Endp_DataUp(DEF_UEP3,s->data+s->offset,n,DEF_UEP_CPY_LOAD)) s->offset+=n;
                continue;
            }
        } else {
            if(!USART_GetFlagStatus(USART1,USART_FLAG_TXE)) continue;
            USART_SendData(USART1,s->data[s->offset++]); if(s->offset<s->length) continue;
        }
        p->front=(p->front+1)%TX_SLOTS; p->count--;
    }
}
