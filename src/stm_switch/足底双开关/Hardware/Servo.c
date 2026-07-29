#include "Servo.h"
#include "PWM.h"

/**
 * @brief  舵机初始化
 * @param  无
 * @retval 无
 */
void Servo_Init(void)
{
	PWM_Init();
}

/**
 * @brief  设置舵机角度
 * @param  Angle 角度值 (0.0~180.0)
 * @retval 无
 */
void Servo_SetAngle(float Angle)
{
	/* 角度范围限制在0~180度 */
	if (Angle < 0.0f) Angle = 0.0f;
	if (Angle > 180.0f) Angle = 180.0f;
	
	/* 映射到PWM占空比：0度->0.5ms(500), 180度->2.5ms(2500) */
	uint16_t compare = (uint16_t)(Angle / 180.0f * 2000.0f + 500.0f);
	PWM_SetCompare1(compare);
}
