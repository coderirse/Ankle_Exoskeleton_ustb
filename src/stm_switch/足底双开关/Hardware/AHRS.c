/**
  ******************************************************************************
  * @file    AHRS.c
  * @author  STM32 Team
  * @version V1.0.0
  * @date    2024-01-01
  * @brief   姿态解算模块 - 基于四元数的互补滤波算法(Mahony滤波器)
  * 
  * 本模块实现了基于四元数的姿态解算算法，通过融合MPU6050陀螺仪和加速度计数据，
  * 解算出三个姿态角：偏航角(Yaw)、滚转角(Roll)、俯仰角(Pitch)。
  * 
  * 算法原理：
  * 1. 使用四元数表示三维姿态，避免万向锁问题
  * 2. 陀螺仪积分预测姿态变化
  * 3. 加速度计提供重力向量作为参考，修正姿态估计
  * 4. 通过互补滤波融合两者数据，获得稳定准确的姿态
  ******************************************************************************
  */

#include "AHRS.h"
#include "MPU6050.h"
#include <math.h>

/** @defgroup AHRS_Constants 姿态解算常量定义
  * @{
  */
#define PI              3.141592653589793f    /* 圆周率 */
#define GYRO_MEAS_ERROR PI * (5.0f / 180.0f)  /* 陀螺仪测量误差(5度) */
#define BETA            sqrt(3.0f / 4.0f) * GYRO_MEAS_ERROR  /* 滤波系数 */
/**
  * @}
  */

/** @defgroup AHRS_Global_Variables 全局变量
  * @{
  */
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  /* 四元数变量 q0~q3 */
float Yaw = 0.0f, Pitch = 0.0f, Roll = 0.0f;        /* 姿态角(单位:度) */
/**
  * @}
  */

/**
  * @brief   快速平方根倒数算法 (Quake III引擎经典算法)
  * @param   x: 输入值
  * @retval  1/sqrt(x) 的近似值
  */
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;          /* 将float类型强制转换为long类型 */
    i = 0x5f3759df - (i >> 1);    /* 神奇的数字，牛顿迭代初始值 */
    y = *(float*)&i;              /* 将long类型强制转换回float类型 */
    y = y * (1.5f - (halfx * y * y));  /* 一次牛顿迭代 */
    return y;
}

/**
  * @brief   初始化姿态解算模块
  * @param   无
  * @retval  无
  * @note    初始化MPU6050传感器，并将四元数和姿态角初始化为默认值
  */
void AHRS_Init(void) {
    MPU6050_Init();   /* 初始化MPU6050传感器 */
    
    /* 初始化四元数为单位四元数，表示初始姿态为水平静止状态 */
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    
    /* 初始化姿态角为0 */
    Yaw = 0.0f;
    Pitch = 0.0f;
    Roll = 0.0f;
}

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
  * @note    使用Mahony互补滤波算法融合陀螺仪和加速度计数据
  */
void AHRS_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float norm;     /* 归一化因子 */
    float vx, vy, vz;  /* 从四元数推导的重力向量 */
    float ex, ey, ez;  /* 误差向量 */
    
    /* 步骤1: 归一化加速度计数据 */
    norm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    /* 步骤2: 根据当前四元数计算重力向量在机体坐标系中的投影 */
    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    /* 步骤3: 计算误差向量(加速度计测量值与估计值的叉积) */
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    /* 步骤4: 应用滤波系数，计算误差修正量 */
    ex *= BETA;
    ey *= BETA;
    ez *= BETA;

    /* 步骤5: 将误差修正量叠加到陀螺仪数据上 */
    gx += ex;
    gy += ey;
    gz += ez;

    /* 步骤6: 保存当前四元数副本用于计算 */
    float qa = q0;
    float qb = q1;
    float qc = q2;

    /* 步骤7: 四元数微分方程更新(龙格-库塔一阶近似) */
    q0 += (-qb * gx - qc * gy - q3 * gz) * (0.5f * dt);
    q1 += (qa * gx + qc * gz - q3 * gy) * (0.5f * dt);
    q2 += (qa * gy - qb * gz + q3 * gx) * (0.5f * dt);
    q3 += (qa * gz + qb * gy - qc * gx) * (0.5f * dt);

    /* 步骤8: 归一化四元数，保证其模为1 */
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;
}

/**
  * @brief   从四元数计算欧拉角
  * @param   yaw: 偏航角指针(输出，单位:度)
  * @param   pitch: 俯仰角指针(输出，单位:度)
  * @param   roll: 滚转角指针(输出，单位:度)
  * @retval  无
  * @note    四元数转欧拉角公式
  */
void AHRS_GetEulerAngles(float *yaw, float *pitch, float *roll) {
    /* 计算滚转角(Roll): 绕X轴旋转角度 */
    *roll = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / PI;
    
    /* 计算俯仰角(Pitch): 绕Y轴旋转角度 */
    *pitch = asin(2.0f * (q0 * q2 - q1 * q3)) * 180.0f / PI;
    
    /* 计算偏航角(Yaw): 绕Z轴旋转角度 */
    *yaw = atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / PI;
}

/**
  * @brief   从MPU6050获取数据并更新姿态
  * @param   dt: 采样时间间隔(单位:秒)
  * @retval  无
  * @note    此函数是AHRS_Update的封装，直接从MPU6050读取原始数据并进行姿态解算
  */
void AHRS_UpdateFromMPU6050(float dt) {
    int16_t AX, AY, AZ, GX, GY, GZ;  /* MPU6050原始数据(16位有符号整数) */
    float ax, ay, az, gx, gy, gz;     /* 转换后的工程单位数据 */
    
    /* 从MPU6050读取六轴原始数据 */
    MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
    
    /* 将加速度计原始数据转换为g单位(量程±16g,比例因子2048 LSB/g) */
    ax = (float)AX / 2048.0f;
    ay = (float)AY / 2048.0f;
    az = (float)AZ / 2048.0f;
    
    /* 将陀螺仪原始数据转换为弧度/秒(量程±2000°/s,比例因子16.4 LSB/(°/s)) */
    gx = (float)GX / 16.4f * PI / 180.0f;
    gy = (float)GY / 16.4f * PI / 180.0f;
    gz = (float)GZ / 16.4f * PI / 180.0f;
    
    /* 更新姿态解算 */
    AHRS_Update(gx, gy, gz, ax, ay, az, dt);
    
    /* 计算欧拉角并更新全局变量 */
    AHRS_GetEulerAngles(&Yaw, &Pitch, &Roll);
}