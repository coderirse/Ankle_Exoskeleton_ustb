#include "MicroSwitch.h"
#include <math.h>

#ifndef M_PI
#define M_PI  3.14159265358979323846f
#endif

/* ═══════════════════════════════════════════════
   电机安全限幅（测试用，额定值 1/10 以内）
   ═══════════════════════════════════════════════ */
#define TORQUE_MAX_TEST   3.0f     /* N·m  (额定 30) */
#define TORQUE_MIN_TEST   0.0f
#define SPEED_MAX_TEST    300.0f   /* deg/s → 50 rpm (额定 2500) */
#define SPEED_MIN_TEST   -300.0f

/* 驱动时长限幅 */
#define DURATION_MIN      0.3f
#define DURATION_MAX      0.9f

/* CAN 指令换算参数（电机驱动协议） */
#define TORQUE_SCALE       4096.0f  /* 12位扭矩值 */
#define SPEED_SCALE        4096.0f  /* 12位速度值 */

/* PD 参数 */
#define KP                0.5f
#define KD                0.02f

/* 状态机延时 */
#define FORWARD_DELAY_S   0.3f
#define SWITCH_DELAY_S    0.1f
#define LOOP_DT           0.01f    /* 主循环 10ms */

/* ── 开关状态 ── */
static uint32_t pressCount[2];
static uint8_t  lastState[2] = {1, 1};

/* ── 步态时序 ── */
static float    fs1Time = 0.0f;      /* FS1 按下时的相位时间戳 */
static float    fs2Time = 0.0f;      /* FS2 按下时的相位时间戳 */
static float    stanceTime = 0.0f;   /* FS2 - FS1 (支撑相阶段2时间) */
static float    stepPeriod = 0.0f;   /* 完整步态周期 */
static float    realPace   = 0.0f;   /* 实时步速 km/h */
static float    driveDuration = 0.625f;  /* 自适应驱动时长 */

/* ── 状态机 ── */
static GaitPhase currentPhase  = PHASE_IDLE;
static float     phaseElapsed  = 0.0f;
static float     targetTorque  = 0.0f;
static float     targetSpeed   = 0.0f;
static uint8_t   pendingCmd    = 0;
static float     globalTime    = 0.0f;  /* 全局累计时间 */

/* 前向声明 */
static void AdaptiveSpeed(float time2);

/* ── PD ── */
static float     prevError     = 0.0f;

/* ── 步态校验 ── */
static uint8_t   lastValidCmd  = 0;
static uint32_t  validSteps    = 0;
static uint32_t  errorSteps    = 0;

/* ── 上一周期记录 ── */
static float     prevFs1Time   = 0.0f;

/* ═══════════════════════════════════════════════
   初始化
   ═══════════════════════════════════════════════ */
void MicroSwitch_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef s;
    s.GPIO_Mode  = GPIO_Mode_IPU;
    s.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_12;
    s.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &s);

    lastState[SW_FS1] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12);
    lastState[SW_FS2] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1);
}

/* ═══════════════════════════════════════════════
   开关读取
   ═══════════════════════════════════════════════ */
uint8_t MicroSwitch_GetState(uint8_t sw)
{
    uint16_t pin = (sw == SW_FS1) ? GPIO_Pin_12 : GPIO_Pin_1;
    return GPIO_ReadInputDataBit(GPIOB, pin);
}

uint8_t MicroSwitch_IsPressed(uint8_t sw)
{
    uint8_t cur = MicroSwitch_GetState(sw);

    if (lastState[sw] == 1 && cur == 0) {
        pressCount[sw]++;
        lastState[sw] = cur;

        /* ── 记录开关按下时间 ── */
        if (sw == SW_FS1) {
            prevFs1Time = fs1Time;
            fs1Time = globalTime;
        } else {
            fs2Time = globalTime;
        }

        /* ── 步态时序计算 ──
         * FS1→FS2 = 支撑相阶段2时间 (time2)
         * 用于自适应步速解算
         */
        if (sw == SW_FS2 && fs1Time > 0.0f) {
            stanceTime = fs2Time - fs1Time;
            if (stanceTime > 0.05f && stanceTime < 3.0f) {
                /* 自适应步速 (复现 AdaptiveSpeed) */
                AdaptiveSpeed(stanceTime);
            }
            /* 步态周期 ≈ 当前FS2 - 上一FS2 */
            if (prevFs1Time > 0.0f) {
                stepPeriod = fs1Time - prevFs1Time;
            }
        }

        /* ── 序列校验 ── */
        if (sw == SW_FS1) {
            lastValidCmd = 0x41;
            validSteps++;
        } else {
            if (lastValidCmd == 0x41) {
                lastValidCmd = 0x43;
                validSteps++;
            } else {
                errorSteps++;
            }
        }

        /* ── 状态机 ── */
        switch (currentPhase) {
        case PHASE_IDLE:
            if (sw == SW_FS1) {
                currentPhase = PHASE_FORWARD_DELAY;
                phaseElapsed = 0.0f;
                pendingCmd = 0;
            } else {
                currentPhase = PHASE_DRIVING_RV;
                phaseElapsed = 0.0f;
                pendingCmd = 0;
            }
            break;
        case PHASE_FORWARD_DELAY:
            if (sw == SW_FS2) pendingCmd = 2;
            break;
        case PHASE_DRIVING_FW:
            if (sw == SW_FS2) pendingCmd = 2;
            break;
        case PHASE_DELAY:
            if (sw == SW_FS1) pendingCmd = 1;
            if (sw == SW_FS2) pendingCmd = 2;
            break;
        case PHASE_DRIVING_RV:
            if (sw == SW_FS1) pendingCmd = 1;
            break;
        }
        return 1;
    }

    if (lastState[sw] == 0 && cur == 1) {
        lastState[sw] = cur;
    }
    return 0;
}

uint32_t MicroSwitch_GetPressCount(uint8_t sw) { return pressCount[sw]; }

/* ═══════════════════════════════════════════════
   自适应步速解算（复现 can_ankleControl_node.cpp AdaptiveSpeed）
   ═══════════════════════════════════════════════ */
static void AdaptiveSpeed(float time2)
{
    /*
     * 从支撑相阶段2时间反算实时步速
     *   x = time2 (支撑相阶段2时间)
     *   模型: x = a1 * exp(-b1 * pace) + c1
     *   反解: pace = -ln((x - c1) / a1) / b1
     */
    const float a1 = 1.45022f;
    const float b1 = 0.4695f;
    const float c1 = 0.06345f;

    float x = time2;
    if (x <= c1) x = c1 + 0.001f;
    float i1 = (x - c1) / a1;
    if (i1 <= 0.0f) i1 = 0.001f;
    float pace = -logf(i1) / b1;
    realPace = pace;

    /*
     * 根据步速自适应调整驱动时长和扭矩
     * 复现 AdaptiveSpeed() 的分段逻辑
     */
    if (pace >= 0.0f && pace <= 6.0f) {
        if (pace <= 2.1f) {
            driveDuration = 1.2f * (-0.1908f * pace + 0.8061f);
        } else if (pace <= 4.8f) {
            driveDuration = 1.2f * (0.03754f * pace * pace - 0.3f * pace + 0.87f);
        } else {
            driveDuration = 0.2949216f * 1.2f;
        }
    } else {
        driveDuration = (pace > 6.0f) ? (0.2949216f * 1.3f) : 0.8f;
    }

    /* 限幅 */
    if (driveDuration < DURATION_MIN) driveDuration = DURATION_MIN;
    if (driveDuration > DURATION_MAX) driveDuration = DURATION_MAX;
}

/* ═══════════════════════════════════════════════
   扭矩曲线（复现 can_ankleControl_node.cpp calculateTargetTorque）
   余弦上升 → 余弦下降，上升占比 0.5
   ═══════════════════════════════════════════════ */
static float torqueCurve(float t, float duration, float peak)
{
    float riseTime  = duration * 0.5f;
    float fallTime  = duration - riseTime;

    if (t < riseTime) {
        float phase = M_PI * t / riseTime;
        return (1.0f - cosf(phase)) * peak / 2.0f;
    } else if (t < duration) {
        float t_fall = t - riseTime;
        float phase = M_PI * t_fall / fallTime;
        return (1.0f + cosf(phase)) / 2.0f * peak;
    }
    return 0.0f;
}

/*
 * 简化版峰值扭矩：基于体重+步速
 * 完整版需要坡度/步长等参数，测试用此简化版
 */
static float calcPeakTorque(void)
{
    /* 基值 ~0.3×体重(假设70kg) + 步速补偿 */
    float base  = 21.0f;       /* 0.3 * 70 */
    float extra = 0.0f;

    if (realPace > 2.1f && realPace <= 4.8f) {
        extra = 0.0f;
    } else if (realPace > 4.8f) {
        extra = (realPace - 4.8f) * 5.0f / 1.2f;
    } else {
        extra = (realPace - 2.1f) * 5.0f / 2.1f;
    }

    float peak = base + extra;

    /* 安全限幅 */
    if (peak > TORQUE_MAX_TEST) peak = TORQUE_MAX_TEST;
    if (peak < 0.0f)           peak = 0.0f;

    return peak;
}

/* PD 控制器 */
static float pdCompute(float setpoint, float dt)
{
    if (dt <= 1e-6f) return 0.0f;
    float error = setpoint;          /* measurement=0 无传感器 */
    float derivative = (error - prevError) / dt;
    float output = KP * error + KD * derivative;
    if (output > SPEED_MAX_TEST)  output = SPEED_MAX_TEST;
    if (output < SPEED_MIN_TEST)  output = SPEED_MIN_TEST;
    prevError = error;
    return output;
}

/* ═══════════════════════════════════════════════
   状态机更新
   ═══════════════════════════════════════════════ */
void MicroSwitch_Update(void)
{
    globalTime += LOOP_DT;

    switch (currentPhase) {

    case PHASE_IDLE:
        targetTorque = 0.0f;
        targetSpeed  = 0.0f;
        break;

    case PHASE_FORWARD_DELAY:
        targetTorque = 0.0f;
        targetSpeed  = 0.0f;
        phaseElapsed += LOOP_DT;
        if (phaseElapsed >= FORWARD_DELAY_S) {
            currentPhase = PHASE_DRIVING_FW;
            phaseElapsed = 0.0f;
        }
        break;

    case PHASE_DRIVING_FW: {
        float peak = calcPeakTorque();
        targetTorque = torqueCurve(phaseElapsed, driveDuration, peak);
        targetSpeed  = pdCompute(targetTorque, LOOP_DT);
        phaseElapsed += LOOP_DT;
        if (phaseElapsed >= driveDuration) {
            targetTorque = 0.0f;
            targetSpeed  = 0.0f;
            currentPhase = (pendingCmd != 0) ? PHASE_DELAY : PHASE_IDLE;
            phaseElapsed = 0.0f;
        }
        break;
    }

    case PHASE_DELAY:
        targetTorque = 0.0f;
        targetSpeed  = 0.0f;
        phaseElapsed += LOOP_DT;
        if (phaseElapsed >= SWITCH_DELAY_S) {
            if (pendingCmd == 1) {
                currentPhase = PHASE_DRIVING_FW;
            } else {
                currentPhase = PHASE_DRIVING_RV;
            }
            pendingCmd = 0;
            phaseElapsed = 0.0f;
        }
        break;

    case PHASE_DRIVING_RV: {
        float peak = calcPeakTorque();
        targetTorque = -torqueCurve(phaseElapsed, driveDuration, peak);
        targetSpeed  = pdCompute(targetTorque, LOOP_DT);
        phaseElapsed += LOOP_DT;
        if (phaseElapsed >= driveDuration) {
            targetTorque = 0.0f;
            targetSpeed  = 0.0f;
            currentPhase = (pendingCmd != 0) ? PHASE_DELAY : PHASE_IDLE;
            phaseElapsed = 0.0f;
        }
        break;
    }
    }
}

/* ═══════════════════════════════════════════════
   查询接口
   ═══════════════════════════════════════════════ */
GaitPhase MicroSwitch_GetPhase(void)        { return currentPhase; }
float     MicroSwitch_GetTargetTorque(void)  { return targetTorque; }
float     MicroSwitch_GetTargetSpeed(void)   { return targetSpeed; }
float     MicroSwitch_GetPhaseElapsed(void)  { return phaseElapsed; }
float     MicroSwitch_GetStanceTime(void)    { return stanceTime; }
float     MicroSwitch_GetStepPeriod(void)    { return stepPeriod; }
float     MicroSwitch_GetRealPace(void)      { return realPace; }
uint32_t  MicroSwitch_GetValidSteps(void)    { return validSteps; }
uint32_t  MicroSwitch_GetErrorSteps(void)    { return errorSteps; }

/* ═══════════════════════════════════════════════
   CAN 指令封装（供串口发送到 PC → USB-CAN → 电机）
   ═══════════════════════════════════════════════ */

/* 8字节扭矩指令 (CANopen 扭矩模式)
 * [0]=0x7F, [1]=0xFF, [2]=0x7F, [3]=0xF0,
 * [4]=0x00, [5]=0x00, [6:7]=12位扭矩值(大端) */
void MicroSwitch_GetTorqueCAN(uint8_t out[8])
{
    /* 扭矩映射到 0~4095 (0 → TORQUE_MIN_TEST, 4095 → TORQUE_MAX_TEST) */
    float t = targetTorque;
    if (t < 0.0f) t = 0.0f;
    uint16_t raw = (uint16_t)(t / TORQUE_MAX_TEST * TORQUE_SCALE);
    if (raw > 4095) raw = 4095;

    out[0] = 0x7F; out[1] = 0xFF; out[2] = 0x7F; out[3] = 0xF0;
    out[4] = 0x00; out[5] = 0x00;
    out[6] = (uint8_t)((raw >> 8) & 0xFF);
    out[7] = (uint8_t)(raw & 0xFF);
}

/* 5字节速度指令 (CANopen 速度模式)
 * [0]=0x03, [1:4]=32位速度值(小端, deg/s → rpm 换算) */
void MicroSwitch_GetSpeedCAN(uint8_t out[5])
{
    float s = targetSpeed;
    if (s > SPEED_MAX_TEST)  s = SPEED_MAX_TEST;
    if (s < SPEED_MIN_TEST)  s = SPEED_MIN_TEST;

    /* deg/s → rpm */
    float rpm = s / 6.0f;
    /* CAN 速度原始值: 0x00010000 (65536) = 0.15 rpm */
    int32_t raw = (int32_t)(rpm * (65536.0f / 0.15f));

    out[0] = 0x03;
    out[1] = (uint8_t)(raw & 0xFF);
    out[2] = (uint8_t)((raw >> 8) & 0xFF);
    out[3] = (uint8_t)((raw >> 16) & 0xFF);
    out[4] = (uint8_t)((raw >> 24) & 0xFF);
}
