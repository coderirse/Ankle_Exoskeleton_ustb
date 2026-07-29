#include <stdint.h>
#define AFIO_BASE 0x40010000
#define AFIO_MAPR (*(volatile uint32_t*)(AFIO_BASE+0x04))
#define RCC_BASE 0x40021000
#define FLASH_ACR (*(volatile uint32_t*)(0x40022000))
#define RCC_CR (*(volatile uint32_t*)(RCC_BASE+0x00))
#define RCC_CFGR (*(volatile uint32_t*)(RCC_BASE+0x04))
#define RCC_APB2ENR (*(volatile uint32_t*)(RCC_BASE+0x18))
#define GPIOA_BASE 0x40010800
#define GPIOB_BASE 0x40010C00
#define GPIO_CRL(x) (*(volatile uint32_t*)((x)+0x00))
#define GPIO_CRH(x) (*(volatile uint32_t*)((x)+0x04))
#define GPIO_IDR(x) (*(volatile uint32_t*)((x)+0x08))
#define GPIO_BSRR(x) (*(volatile uint32_t*)((x)+0x10))
#define GPIO_BRR(x) (*(volatile uint32_t*)((x)+0x14))
#define USART1_BASE 0x40013800
#define USART_SR(x) (*(volatile uint32_t*)((x)+0x00))
#define USART_DR(x) (*(volatile uint32_t*)((x)+0x04))
#define USART_BRR(x) (*(volatile uint32_t*)((x)+0x08))
#define USART_CR1(x) (*(volatile uint32_t*)((x)+0x0C))
#define HEEL (1<<5)
#define TOE  (1<<6)
#define LED1 (1<<12)
#define LED2 (1<<15)
void delay(volatile int n) { while(n--) __asm("nop"); }
void dly_s(int s) { for(int i=0;i<s;i++) delay(48000000); }

void clock_init(void) {
    RCC_CR |= 1; while(!(RCC_CR&2));
    RCC_CFGR &= ~((1<<16)|(1<<17)); RCC_CFGR |= (0xA<<18);
    RCC_CR |= 1<<24; while(!(RCC_CR&(1<<25)));
    FLASH_ACR = (1<<4)|1; RCC_CFGR |= (4<<8)|2; while((RCC_CFGR&0xC)!=8);
}
void uart_init(void) {
    RCC_APB2ENR |= (1<<2)|(1<<14);
    USART_BRR(USART1_BASE)=5000;
    USART_CR1(USART1_BASE)=(1<<13)|(1<<3)|(1<<2);
}
void uart_send(uint8_t b) { while(!(USART_SR(USART1_BASE)&(1<<7))); USART_DR(USART1_BASE)=b; }
int rd_heel(void) { return (GPIO_IDR(GPIOB_BASE)&HEEL)?1:0; }
int rd_toe(void)  { return (GPIO_IDR(GPIOB_BASE)&TOE)?1:0; }
uint8_t to_cmd(int h,int t){if(h&&!t)return 0x41;if(h&&t)return 0x42;if(!h&&t)return 0x43;return 0x44;}

int main(void) {
    clock_init();

    // GPIO: PA9=AF_PP, PA12=OUT_PP, PA15=OUT_PP, PA13/PA14 keep SWD
    // CRL: PA1..PA7 = default (0x4=floating), PBx on GPIOB
    // CRH: PA9(7:4)=0xB, PA12(19:16)=0x3, PA15(31:28)=0x3
    // Others (PA8,PA10,PA11,PA13,PA14) = 0x4 (floating input, reset default)
    RCC_APB2ENR |= (1<<2)|(1<<3)|(1<<0);  // GPIOA+B+AFIO
    AFIO_MAPR = (AFIO_MAPR & ~(7<<24)) | (2<<24);  // free PA15
    GPIO_CRL(GPIOB_BASE) = (GPIO_CRL(GPIOB_BASE)&~((0xF<<20)|(0xF<<24)))|(4<<20)|(4<<24);
    GPIO_CRH(GPIOA_BASE) = 0x344443B4;  // PA15=3, PA14=4, PA13=4, PA12=3, PA11=4, PA10=4, PA9=B, PA8=4
    // 0x3000 0B44 = PA15(3) PA14(4) PA13(4) PA12(3) PA11(4) PA10(4) PA9(B) PA8(4)
    GPIO_BRR(GPIOA_BASE) = LED1|LED2;

    uart_init();

    // Blink 3x: BSRR=HIGH=ON (active-high)
    for(int i=0;i<3;i++){
        GPIO_BSRR(GPIOA_BASE)=LED1|LED2; dly_s(1);
        GPIO_BRR(GPIOA_BASE)=LED1|LED2; dly_s(1);
    }

    int sh=rd_heel(),st=rd_toe(); uint8_t last=to_cmd(sh,st); int hc=0,tc=0;
    while(1){
        {int v=rd_heel();if(v==sh)hc=0;else{hc++;if(hc>=8){hc=0;sh=v;}}}
        {int v=rd_toe();if(v==st)tc=0;else{tc++;if(tc>=8){tc=0;st=v;}}}
        if(sh) GPIO_BSRR(GPIOA_BASE)=LED1; else GPIO_BRR(GPIOA_BASE)=LED1;
        if(st) GPIO_BSRR(GPIOA_BASE)=LED2; else GPIO_BRR(GPIOA_BASE)=LED2;
        uint8_t cmd=to_cmd(sh,st); if(cmd!=last){last=cmd;uart_send(cmd);}
    }
}
__attribute__((section(".vectors")))
const uint32_t vectors[]={[0]=0x20005000,[1]=(uint32_t)main};
