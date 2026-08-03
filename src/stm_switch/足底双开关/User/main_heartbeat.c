/**
  * 足底双开关采集 — 心跳版 (2026-08-03)
  *
  * 与 main.c 的区别: 除状态变化立即发送外,
  * 每 ~50ms 周期性重发当前状态 (心跳),
  * 使上位机能随时获知开关当前状态 (初始化站立确认需要)。
  *
  * 硬件接线 (与 main.c / 接线与通信协议.md 一致):
  *   PB12 ← 后脚跟开关 (按下接地, 内部上拉)
  *   PB1  ← 前脚掌开关 (按下接地, 内部上拉)
  *   PA9  → USART1_TX → USB转串口 RX
  *   PA10 ← USART1_RX
  *   PA1  → LED1 (后脚跟指示)
  *   PA2  → LED2 (前脚掌指示)
  *
  * 协议 (不变):
  *   0x41 = 脚跟着地  0x42 = 全掌接触
  *   0x43 = 脚跟离地  0x44 = 脚尖离地
  *
  * 纯寄存器实现, 自带向量表, 用 arm-none-eabi-gcc + link.ld 编译:
  *   arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O2 -nostdlib \
  *       -T ../link.ld main_heartbeat.c -o main_heartbeat.elf
  *   arm-none-eabi-objcopy -O binary main_heartbeat.elf main_heartbeat.bin
  *   st-flash write main_heartbeat.bin 0x8000000
  */

#include <stdint.h>

/* ── 寄存器 ── */
#define RCC_BASE        0x40021000
#define RCC_CR          (*(volatile uint32_t*)(RCC_BASE+0x00))
#define RCC_CFGR        (*(volatile uint32_t*)(RCC_BASE+0x04))
#define RCC_APB2ENR     (*(volatile uint32_t*)(RCC_BASE+0x18))
#define FLASH_ACR       (*(volatile uint32_t*)(0x40022000))

#define GPIOA_BASE      0x40010800
#define GPIOB_BASE      0x40010C00
#define GPIO_CRL(x)     (*(volatile uint32_t*)((x)+0x00))
#define GPIO_CRH(x)     (*(volatile uint32_t*)((x)+0x04))
#define GPIO_IDR(x)     (*(volatile uint32_t*)((x)+0x08))
#define GPIO_BSRR(x)    (*(volatile uint32_t*)((x)+0x10))
#define GPIO_BRR(x)     (*(volatile uint32_t*)((x)+0x14))

#define USART1_BASE     0x40013800
#define USART_SR(x)     (*(volatile uint32_t*)((x)+0x00))
#define USART_DR(x)     (*(volatile uint32_t*)((x)+0x04))
#define USART_BRR(x)    (*(volatile uint32_t*)((x)+0x08))
#define USART_CR1(x)    (*(volatile uint32_t*)((x)+0x0C))

/* ── 引脚 ── */
#define HEEL_PIN        12          /* PB12 后脚跟 */
#define TOE_PIN         1           /* PB1  前脚掌 */
#define HEEL            (1<<HEEL_PIN)
#define TOE             (1<<TOE_PIN)
#define LED1            (1<<1)      /* PA1 */
#define LED2            (1<<2)      /* PA2 */

/* ── 参数 ── */
#define DEBOUNCE_COUNT  8           /* 8ms 消抖 (每循环~1ms) */
#define HEARTBEAT_LOOPS 50          /* 50 x ~1ms = ~50ms 心跳周期 */

/* ── 延时: 48MHz 下 delay(48000) ≈ 1ms ── */
static void delay(volatile uint32_t n) { while(n--) __asm("nop"); }
static void delay_1ms(void) { delay(48000); }

/* ── 时钟: HSI 8MHz /2 x12 = 48MHz ── */
static void clock_init(void)
{
    RCC_CR |= 1;                        /* HSION */
    while(!(RCC_CR & 2));               /* 等 HSIRDY */
    RCC_CFGR &= ~((1<<16)|(1<<17));     /* PLLSRC=HSI/2 */
    RCC_CFGR |= (0xA<<18);              /* PLLMUL x12 = 48MHz */
    RCC_CR |= (1<<24);                  /* PLLON */
    while(!(RCC_CR & (1<<25)));         /* 等 PLLRDY */
    FLASH_ACR = (1<<4)|1;               /* PRFTBE + 1 wait state */
    RCC_CFGR |= (4<<8)|2;               /* APB1=/2, SYSCLK=PLL */
    while((RCC_CFGR & 0xC) != 8);
}

/* ── GPIO: PB12/PB1 上拉输入, PA1/PA2 推挽输出, PA9 AF_PP ── */
static void gpio_init(void)
{
    RCC_APB2ENR |= (1<<2)|(1<<3);       /* GPIOA + GPIOB 时钟 */

    /* GPIOB CRL: PB1(位7:4)=0x8 上拉输入 */
    GPIO_CRL(GPIOB_BASE) = (GPIO_CRL(GPIOB_BASE) & ~(0xF<<4)) | (0x8<<4);
    /* GPIOB CRH: PB12(位19:16)=0x8 上拉输入 */
    GPIO_CRH(GPIOB_BASE) = (GPIO_CRH(GPIOB_BASE) & ~(0xF<<16)) | (0x8<<16);
    /* ODR 置1 → 上拉 */
    GPIO_BSRR(GPIOB_BASE) = HEEL | TOE;

    /* GPIOA CRL: PA1(位7:4)=0x3, PA2(位11:8)=0x3 推挽输出50MHz */
    GPIO_CRL(GPIOA_BASE) = (GPIO_CRL(GPIOA_BASE) & ~((0xF<<4)|(0xF<<8)))
                          | (0x3<<4) | (0x3<<8);
    /* GPIOA CRH: PA9(位7:4)=0xB 复用推挽50MHz, PA10(位11:8)=0x4 浮空输入 */
    GPIO_CRH(GPIOA_BASE) = (GPIO_CRH(GPIOA_BASE) & ~((0xF<<4)|(0xF<<8)))
                          | (0xB<<4) | (0x4<<8);

    GPIO_BSRR(GPIOA_BASE) = LED1 | LED2; /* LED 初始灭 (低电平点亮) */
}

/* ── USART1: 9600 8N1 (48MHz/9600=5000) ── */
static void uart_init(void)
{
    RCC_APB2ENR |= (1<<14);             /* USART1 时钟 */
    USART_BRR(USART1_BASE) = 5000;
    USART_CR1(USART1_BASE) = (1<<13)|(1<<3)|(1<<2);  /* UE | TE | RE */
}

static void uart_send(uint8_t b)
{
    while(!(USART_SR(USART1_BASE) & (1<<7)));   /* 等 TXE */
    USART_DR(USART1_BASE) = b;
}

/* ── 开关读取: 按下接地=0 → 返回1 ── */
static int rd_heel(void) { return (GPIO_IDR(GPIOB_BASE) & HEEL) ? 0 : 1; }
static int rd_toe(void)  { return (GPIO_IDR(GPIOB_BASE) & TOE)  ? 0 : 1; }

static uint8_t to_cmd(int h, int t)
{
    if (h && !t) return 0x41;
    if (h &&  t) return 0x42;
    if (!h && t) return 0x43;
    return 0x44;
}

int main(void)
{
    clock_init();
    gpio_init();
    uart_init();

    int sh = rd_heel(), st = rd_toe();
    uint8_t last_sent = to_cmd(sh, st);
    int hc = 0, tc = 0;
    uint32_t heartbeat = 0;

    uart_send(last_sent);               /* 上电立即上报当前状态 */

    while (1)
    {
        /* 消抖: 连续 DEBOUNCE_COUNT 次一致才更新 */
        { int v = rd_heel(); if (v == sh) hc = 0; else { hc++; if (hc >= DEBOUNCE_COUNT) { hc = 0; sh = v; } } }
        { int v = rd_toe();  if (v == st) tc = 0; else { tc++; if (tc >= DEBOUNCE_COUNT) { tc = 0; st = v; } } }

        /* LED 指示 (低电平点亮: 按下 → 引脚拉低 → 亮) */
        if (sh) GPIO_BRR(GPIOA_BASE) = LED1;  else GPIO_BSRR(GPIOA_BASE) = LED1;
        if (st) GPIO_BRR(GPIOA_BASE) = LED2;  else GPIO_BSRR(GPIOA_BASE) = LED2;

        uint8_t cmd = to_cmd(sh, st);
        if (cmd != last_sent)
        {
            /* 状态变化 → 立即发送 */
            last_sent = cmd;
            uart_send(cmd);
            heartbeat = 0;
        }
        else if (++heartbeat >= HEARTBEAT_LOOPS)
        {
            /* 心跳: 周期性重发当前状态 */
            heartbeat = 0;
            uart_send(last_sent);
        }

        delay_1ms();
    }
}

/* ── 向量表 (link.ld .vectors 段) ── */
__attribute__((section(".vectors")))
const uint32_t vectors[] = {
    [0] = 0x20005000,           /* 栈顶 (20KB SRAM) */
    [1] = (uint32_t)main,       /* Reset_Handler */
};
