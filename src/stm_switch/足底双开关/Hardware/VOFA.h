#ifndef __VOFA_H
#define __VOFA_H

#include "stm32f10x.h"

void VOFA_Init(uint32_t baudrate);
void VOFA_SendFrame(float *data, uint8_t count);
void VOFA_SendStr(const char *str);
void VOFA_Printf(const char *fmt, ...);

/* 从 PC 端接收的电机反馈数据（PC 从 CAN 总线读取后回传） */
void   VOFA_PollRX(void);            /* 非阻塞轮询，每次主循环调用 */
float  VOFA_GetMotorTorque(void);    /* 电机实际扭矩 (N·m)，无数据时返回 NAN */
float  VOFA_GetMotorSpeed(void);     /* 电机实际转速 (deg/s) */
float  VOFA_GetMotorPosition(void);  /* 电机实际位置 (deg) */

#endif
