#include "VOFA.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

static char txBuffer[128];

/* ── 串口接收缓冲区（PC 回传电机实测值）── */
static char   rxBuf[64];
static uint8_t rxIdx = 0;
static float  motorTorque   = NAN;
static float  motorSpeed    = NAN;
static float  motorPosition = NAN;

void VOFA_Init(uint32_t baudrate)
{
    /* 1. 使能时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    /* 2. 配置 PA9(TX) 为复用推挽输出 */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;       /* USART1_TX */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 3. 配置 PA10(RX) 为浮空输入 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;      /* USART1_RX */
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 4. 配置USART1 */
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate            = baudrate;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);

    /* 5. 使能USART1 */
    USART_Cmd(USART1, ENABLE);
}

/* 发送单个字节 */
static void VOFA_SendByte(uint8_t byte)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, byte);
}

/* 发送字符串 */
void VOFA_SendStr(const char *str)
{
    while (*str) {
        VOFA_SendByte((uint8_t)*str++);
    }
}

/*
 * VOFA+ firewater 协议：每帧为逗号分隔的浮点数，以 \n 结尾
 */
void VOFA_SendFrame(float *data, uint8_t count)
{
    int pos = 0;
    for (uint8_t i = 0; i < count; i++) {
        int len = sprintf(txBuffer + pos, "%.2f", data[i]);
        pos += len;
        if (i < count - 1) {
            txBuffer[pos++] = ',';
        }
    }
    txBuffer[pos++] = '\n';
    txBuffer[pos] = '\0';

    VOFA_SendStr(txBuffer);
}

void VOFA_Printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(txBuffer, sizeof(txBuffer), fmt, args);
    va_end(args);
    VOFA_SendStr(txBuffer);
}

/* ═══════════════════════════════════════════════
   串口接收（非阻塞轮询）
   PC 端协议格式（每行一条，\n 结尾）：
     T12.34    → 电机实际扭矩 (N·m)
     S300.5    → 电机实际转速 (deg/s)
     P15.2     → 电机实际位置 (deg)
   ═══════════════════════════════════════════════ */
void VOFA_PollRX(void)
{
    /* 读取所有可用字节 */
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET) {
        char c = (char)USART_ReceiveData(USART1);

        if (c == '\n' || c == '\r') {
            if (rxIdx > 0) {
                rxBuf[rxIdx] = '\0';
                float val = NAN;

                /* 解析: 首字母=类型，后面=数值 */
                if (rxIdx > 1) {
                    val = (float)atof(rxBuf + 1);
                }

                switch (rxBuf[0]) {
                case 'T': motorTorque   = val; break;
                case 'S': motorSpeed    = val; break;
                case 'P': motorPosition = val; break;
                default: break;
                }
                rxIdx = 0;
            }
        } else if (rxIdx < sizeof(rxBuf) - 1) {
            rxBuf[rxIdx++] = c;
        }
    }
}

float VOFA_GetMotorTorque(void)   { return motorTorque; }
float VOFA_GetMotorSpeed(void)    { return motorSpeed; }
float VOFA_GetMotorPosition(void) { return motorPosition; }
