/**
  * 足底双开关采集 —— 通过 USART1 发送步态相位指令给 ROS2 上位机
  *
  * 硬件接线:
  *   PB12 ← 后脚跟开关 (按下接地, 内部上拉)
  *   PB1  ← 前脚掌开关 (按下接地, 内部上拉)
  *   PA9  → USART1_TX → USB转串口 RX
  *   PA10 ← USART1_RX ← USB转串口 TX
  *   PA1  → LED1 (后脚跟指示)
  *   PA2  → LED2 (前脚掌指示)
  *
  * 协议: 开关状态变化时发送单字节
  *   0x41 = 脚跟着地 (后脚跟ON, 前脚掌OFF)
  *   0x42 = 全掌接触 (后脚跟ON, 前脚掌ON)
  *   0x43 = 脚跟离地 (后脚跟OFF, 前脚掌ON)
  *   0x44 = 脚尖离地 (后脚跟OFF, 前脚掌OFF)
  *
  * UART: 9600 8N1 (与 ROS2 serial_sendCommand_node 一致)
  */

#include "stm32f10x.h"
#include "Delay.h"

/* ── 开关引脚 ── */
#define HEEL_PORT   GPIOB
#define HEEL_PIN    GPIO_Pin_12
#define TOE_PORT    GPIOB
#define TOE_PIN     GPIO_Pin_1

/* ── 步态相位指令 ── */
#define CMD_HEEL_STRIKE   0x41
#define CMD_FULL_CONTACT  0x42
#define CMD_HEEL_OFF      0x43
#define CMD_TOE_OFF       0x44

/* ── 消抖: 连续 N 次读取一致才确认状态变化 ── */
#define DEBOUNCE_COUNT    5

/* ═══════════════════════════════════════════════
   USART1 初始化 (9600 8N1)
   ═══════════════════════════════════════════════ */
static void USART1_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    /* PA9 = TX, 复用推挽 */
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Pin   = GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* PA10 = RX, 浮空输入 */
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    gpio.GPIO_Pin  = GPIO_Pin_10;
    GPIO_Init(GPIOA, &gpio);

    USART_InitTypeDef usart;
    usart.USART_BaudRate            = 9600;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &usart);
    USART_Cmd(USART1, ENABLE);
}

static void UART_SendByte(uint8_t byte)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, byte);
}

/* ═══════════════════════════════════════════════
   GPIO 初始化
   ═══════════════════════════════════════════════ */
static void GPIO_Init_All(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    /* PB12, PB1 上拉输入 → 开关 */
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Pin   = HEEL_PIN | TOE_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    /* PA1, PA2 推挽输出 → LED */
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_Init(GPIOA, &gpio);
    GPIO_SetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_2);
}

/* ═══════════════════════════════════════════════
   开关读取 (按下=1, 松开=0)
   ═══════════════════════════════════════════════ */
static inline uint8_t read_heel(void)
{
    return (GPIO_ReadInputDataBit(HEEL_PORT, HEEL_PIN) == Bit_RESET) ? 1 : 0;
}

static inline uint8_t read_toe(void)
{
    return (GPIO_ReadInputDataBit(TOE_PORT, TOE_PIN) == Bit_RESET) ? 1 : 0;
}

/* 2个开关 → 4种相位 → 指令 */
static uint8_t switches_to_cmd(uint8_t heel, uint8_t toe)
{
    if (heel && !toe)  return CMD_HEEL_STRIKE;   /* 0x41 */
    if (heel && toe)   return CMD_FULL_CONTACT;   /* 0x42 */
    if (!heel && toe)  return CMD_HEEL_OFF;       /* 0x43 */
    return CMD_TOE_OFF;                           /* 0x44 */
}

/* ═══════════════════════════════════════════════
   消抖: 连续 DEBOUNCE_COUNT 次读取一致才更新
   ═══════════════════════════════════════════════ */
static uint8_t debounce(uint8_t current_stable, uint8_t new_reading, uint8_t *counter)
{
    if (new_reading == current_stable) {
        *counter = 0;
        return current_stable;
    }
    (*counter)++;
    if (*counter >= DEBOUNCE_COUNT) {
        *counter = 0;
        return new_reading;
    }
    return current_stable;
}

/* ═══════════════════════════════════════════════
   时钟初始化 — 强制 HSI 8MHz（不用外部晶振）
   ═══════════════════════════════════════════════ */
static void Clock_Init(void)
{
    /* 启用 HSI (8MHz 内部 RC，不依赖外部晶振) */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    /* PLL: HSI/2 = 4MHz × 12 = 48MHz */
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);
    RCC->CFGR |= RCC_CFGR_PLLSRC_HSI_Div2;   /* HSI/2 = 4MHz */
    RCC->CFGR |= RCC_CFGR_PLLMULL12;          /* ×12 = 48MHz */

    /* 使能 PLL */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    /* Flash: 1 wait state (48MHz > 24MHz) */
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_1;

    /* HCLK=SYSCLK, PCLK2=HCLK, PCLK1=HCLK/2 */
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;  /* APB1=24MHz */

    /* 切换到 PLL */
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    /* 关闭 HSE（省电、避免漂移干扰） */
    RCC->CR &= ~RCC_CR_HSEON;
    /* 更新全局时钟变量（USART 库函数依赖此值计算波特率） */
    SystemCoreClock = 48000000;
}

/* ═══════════════════════════════════════════════
   主函数
   ═══════════════════════════════════════════════ */
int main(void)
{
    Clock_Init();  /* 先初始化时钟！ */
    GPIO_Init_All();
    USART1_Init();

    /* 初始稳定状态 */
    uint8_t stable_heel = read_heel();
    uint8_t stable_toe  = read_toe();
    uint8_t last_cmd    = switches_to_cmd(stable_heel, stable_toe);
    uint8_t heel_counter = 0;
    uint8_t toe_counter  = 0;

    while (1)
    {
        /* 读取 + 消抖 */
        stable_heel = debounce(stable_heel, read_heel(), &heel_counter);
        stable_toe  = debounce(stable_toe,  read_toe(),  &toe_counter);

        /* LED 指示 */
        if (stable_heel) GPIO_ResetBits(GPIOA, GPIO_Pin_1);
        else             GPIO_SetBits(GPIOA, GPIO_Pin_1);

        if (stable_toe)  GPIO_ResetBits(GPIOA, GPIO_Pin_2);
        else             GPIO_SetBits(GPIOA, GPIO_Pin_2);

        /* 相位变化 → 发送指令 */
        uint8_t cmd = switches_to_cmd(stable_heel, stable_toe);
        if (cmd != last_cmd) {
            last_cmd = cmd;
            UART_SendByte(cmd);
        }
    }
}
