#include "PID.h"

/**
 * @brief  PID初始化
 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float min, float max)
{
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
	pid->Target = 0.0f;
	pid->SumError = 0.0f;
	pid->LastError = 0.0f;
	pid->OutMin = min;
	pid->OutMax = max;
}

/**
 * @brief  PID计算
 */
float PID_Compute(PID_TypeDef *pid, float measure)
{
	float error = pid->Target - measure;
	float out;
	
	/* 比例项 */
	out = pid->Kp * error;
	
	/* 积分项 (带有抗饱和逻辑) */
	pid->SumError += error;
	if (pid->SumError > pid->OutMax) pid->SumError = pid->OutMax;
	if (pid->SumError < pid->OutMin) pid->SumError = pid->OutMin;
	out += pid->Ki * pid->SumError;
	
	/* 微分项 */
	out += pid->Kd * (error - pid->LastError);
	pid->LastError = error;
	
	/* 输出限幅 */
	if (out > pid->OutMax) out = pid->OutMax;
	if (out < pid->OutMin) out = pid->OutMin;
	
	return out;
}
