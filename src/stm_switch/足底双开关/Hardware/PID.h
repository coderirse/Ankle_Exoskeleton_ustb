#ifndef __PID_H
#define __PID_H

#include "stm32f10x.h"

/**
 * @brief  PID结构体定义
 */
typedef struct {
	float Kp;			/* 比例系数 */
	float Ki;			/* 积分系数 */
	float Kd;			/* 微分系数 */
	float Target;		/* 目标值 */
	float SumError;		/* 误差积分 */
	float LastError;	/* 上次误差 */
	float OutMax;		/* 输出上限 */
	float OutMin;		/* 输出下限 */
} PID_TypeDef;

/**
 * @brief  PID初始化
 * @param  pid: PID结构体指针
 * @param  kp, ki, kd: 系数
 * @param  min, max: 输出范围
 * @retval 无
 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float min, float max);

/**
 * @brief  PID计算
 * @param  pid: PID结构体指针
 * @param  measure: 当前测量值
 * @retval 计算得到的控制量
 */
float PID_Compute(PID_TypeDef *pid, float measure);

#endif
