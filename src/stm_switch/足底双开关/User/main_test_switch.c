/**
  * 开关测试程序 —— 只做一件事：读 PB12，控制 LED1(PA1)
  * 接线：开关一端接 PB12，另一端接 GND
  *       LED1 正极接 PA1，负极经 1k 电阻接 GND
  * 现象：按下开关 LED1 亮，松开 LED1 灭
  */

#include "stm32f10x.h"

int main(void)
{
    /* 使能 GPIOA、GPIOB 时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    /* PA1 推挽输出 → LED1 */
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Pin   = GPIO_Pin_1;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* PB12 上拉输入 → 开关 */
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Pin  = GPIO_Pin_12;
    GPIO_Init(GPIOB, &gpio);

    GPIO_SetBits(GPIOA, GPIO_Pin_1);  /* LED1 初始灭 */

    while (1)
    {
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) == 0)
            GPIO_ResetBits(GPIOA, GPIO_Pin_1);   /* 按下 → 亮 */
        else
            GPIO_SetBits(GPIOA, GPIO_Pin_1);     /* 松开 → 灭 */
    }
}
