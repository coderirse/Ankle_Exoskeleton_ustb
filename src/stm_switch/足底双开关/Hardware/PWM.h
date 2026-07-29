#ifndef __PWM_H
#define __PWM_H

#include "stm32f10x.h"

/**
 * @brief  PWM初始化
 * @param  无
 * @retval 无
 */
void PWM_Init(void);

/**
 * @brief  设置PWM占空比
 * @param  Compare 占空比数值 (500~2500 对应 0.5ms~2.5ms)
 * @retval 无
 */
void PWM_SetCompare1(uint16_t Compare);

#endif
