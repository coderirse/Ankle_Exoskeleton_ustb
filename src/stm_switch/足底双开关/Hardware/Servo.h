#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f10x.h"

/**
 * @brief  舵机初始化
 * @param  无
 * @retval 无
 */
void Servo_Init(void);

/**
 * @brief  设置舵机角度
 * @param  Angle 角度值 (0.0~180.0)
 * @retval 无
 */
void Servo_SetAngle(float Angle);

#endif
