/**
  ******************************************************************************
  * @file    AHRS.h
  * @author  STM32 Team
  * @version V1.0.0
  * @date    2024-01-01
  * @brief   姿态解算模块头文件
  * 
  * 本文件包含姿态解算模块的函数声明和全局变量定义。
  * 使用基于四元数的互补滤波算法实现姿态估计。
  ******************************************************************************
  */

#ifndef __AHRS_H
#define __AHRS_H

/** @defgroup AHRS_Global_Variables 全局变量声明
  * @{
  */
extern float Yaw;    /* 偏航角(单位:度) - 绕Z轴旋转 */
extern float Pitch;  /* 俯仰角(单位:度) - 绕Y轴旋转 */
extern float Roll;   /* 滚转角(单位:度) - 绕X轴旋转 */
/**
  * @}
  */

/** @defgroup AHRS_Functions 函数声明
  * @{
  */

/**
  * @brief   初始化姿态解算模块
  * @param   无
  * @retval  无
  */
void AHRS_Init(void);

/**
  * @brief   更新姿态解算(核心算法)
  * @param   gx: X轴陀螺仪角速度(单位:弧度/秒)
  * @param   gy: Y轴陀螺仪角速度(单位:弧度/秒)
  * @param   gz: Z轴陀螺仪角速度(单位:弧度/秒)
  * @param   ax: X轴加速度计读数(单位:g)
  * @param   ay: Y轴加速度计读数(单位:g)
  * @param   az: Z轴加速度计读数(单位:g)
  * @param   dt: 采样时间间隔(单位:秒)
  * @retval  无
  */
void AHRS_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

/**
  * @brief   从四元数计算欧拉角
  * @param   yaw: 偏航角指针(输出)
  * @param   pitch: 俯仰角指针(输出)
  * @param   roll: 滚转角指针(输出)
  * @retval  无
  */
void AHRS_GetEulerAngles(float *yaw, float *pitch, float *roll);

/**
  * @brief   从MPU6050获取数据并更新姿态
  * @param   dt: 采样时间间隔(单位:秒)
  * @retval  无
  */
void AHRS_UpdateFromMPU6050(float dt);

/**
  * @}
  */

#endif