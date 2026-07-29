#ifndef __MICROSWITCH_H
#define __MICROSWITCH_H

#include "stm32f10x.h"

/* 开关 */
#define SW_FS1  0   /* FS1=PB12 脚跟着地 */
#define SW_FS2  1   /* FS2=PB1  脚跟离地 */

/* 步态相位 */
typedef enum {
    PHASE_IDLE = 0,
    PHASE_FORWARD_DELAY,    /* 正转前延时 0.3s */
    PHASE_DRIVING_FW,       /* 正转驱动：脚跟→脚掌着地 */
    PHASE_DELAY,            /* 换向延时 0.1s */
    PHASE_DRIVING_RV        /* 反转驱动：脚掌→脚尖离地 */
} GaitPhase;

/* 开关读取 */
void     MicroSwitch_Init(void);
uint8_t  MicroSwitch_GetState(uint8_t sw);
uint8_t  MicroSwitch_IsPressed(uint8_t sw);
uint32_t MicroSwitch_GetPressCount(uint8_t sw);

/* 步态时序 */
float    MicroSwitch_GetStanceTime(void);        /* FS1→FS2 时间差 (s) */
float    MicroSwitch_GetStepPeriod(void);        /* 完整步态周期 (s) */
float    MicroSwitch_GetRealPace(void);          /* 实时步速 (km/h) */

/* 状态机 + 扭矩 */
void      MicroSwitch_Update(void);                  /* 每 10ms 主循环更新 */
GaitPhase MicroSwitch_GetPhase(void);
float     MicroSwitch_GetTargetTorque(void);     /* 目标扭矩 N·m（含限幅） */
float     MicroSwitch_GetTargetSpeed(void);      /* 目标转速 deg/s */
float     MicroSwitch_GetPhaseElapsed(void);     /* 当前相位已用时间 s */
uint32_t  MicroSwitch_GetValidSteps(void);
uint32_t  MicroSwitch_GetErrorSteps(void);

/* 电机 CAN 指令字节（供串口发送到 PC→CAN→电机） */
void     MicroSwitch_GetTorqueCAN(uint8_t out[8]);  /* 8字节扭矩指令 */
void     MicroSwitch_GetSpeedCAN(uint8_t out[5]);   /* 5字节速度指令 */

#endif
