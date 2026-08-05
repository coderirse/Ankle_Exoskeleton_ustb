#pragma once

#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cstdio>
#include "signal.h"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <serial/serial.h>
#include <fstream>
#include <iostream>
#include <cstring>
#include <cmath>
#include <chrono>
#include <ratio>
#include <ctime>
#include <thread>
#include <mutex>
#include <atomic>
#include <queue>
#include <vector>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

extern "C" {
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <cstdlib>
}

#include "can_ankle/controlcan.h"

using namespace std;
using namespace chrono;

// ====== CAN native VCI I/O ======
void can_init(void);    // open libcontrolcan device
void can_send(DWORD id, const BYTE* data, int len);
int  can_recv(DWORD* id, BYTE* data, int timeout_ms);
void can_close(void);

// ====== CAN API wrappers (compatible with original code) ======
void Init_Can(void);
void SendData(VCI_CAN_OBJ &h, const int id, const BYTE *data);
void SendData_five(VCI_CAN_OBJ &h, const int id, const BYTE *data);
void SendData_two(VCI_CAN_OBJ &h, const int id, const BYTE *data);
void *receive_func(void *param);

// ====== Global data ======
VCI_INIT_CONFIG config;
VCI_CAN_OBJ send_motor_torque;
VCI_CAN_OBJ config_node;

BYTE config_motor1[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
BYTE config_motor2[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
BYTE config_motor3[8] = {0x7F,0xFF,0x7F,0xF0,0x00,0x00,0x07,0xFF};
BYTE config_motor4[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};

uint16_t output[8];
uint16_t velocity_output[5];
double torque_value, velocity_value, kd, kp;
double P_max=95.5, P_min=-95.5, T_max=40, T_min=0, V_max=180, V_min=-180;
double min_duration=0.3, max_duration=0.9;
double Output_TorqueValue, Output_VelocityValue, Output_Angle;
float  SensorTorque;
double adjusted_torque, adjusted_velocity;

pthread_t threadid;

// ====== Function declarations ======
bool checkCommandSequence();
void commandCallback(const std_msgs::msg::UInt8::SharedPtr msg);
void torqueCallback(const std_msgs::msg::Float32::SharedPtr msg);
void encoderCallback(const std_msgs::msg::Float64::SharedPtr msg);
void one_support_timeCallback(const std_msgs::msg::Float64::SharedPtr msg);
void two_support_timeCallback(const std_msgs::msg::Float64::SharedPtr msg);
void three_support_timeCallback(const std_msgs::msg::Float64::SharedPtr msg);
void swing_timeCallback(const std_msgs::msg::Float64::SharedPtr msg);
void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

double calculateTargetPosition();
double calculateTargetTorque();
double TargetTorque();
double calculateSupportPosition();

void motor_on_V();
void motor_on_P();
void motor_on_T();
void sendMSG();
void processHoming();
void AdaptiveSpeed(double x);
double updateTorquePeak();
void getUserWeight();
void updateCompensateTorque();
void sendVelocityCommand(double y);
void sendTorTransVelCommand(double y);
vector<uint8_t> speed_to_command(double speed);

// ====== CAN native libcontrolcan implementation ======
static bool can_opened = false;
static std::recursive_mutex can_mutex;
// 2026-08-05: SDO 响应捕获 (启动自诊断用)
static volatile bool sdo_resp_ready = false;
static uint8_t sdo_resp_data[8] = {0};
#define CAN_DEV_TYPE VCI_USBCAN2
#define CAN_DEV_IND  0
#define CAN_CH       0

inline void can_init(void) {
    if (can_opened) return;
    std::lock_guard<std::recursive_mutex> lk(can_mutex);
    memset(&config, 0, sizeof(config));
    config.AccCode = 0;
    config.AccMask = 0xFFFFFFFF;
    config.Filter = 0;
    config.Timing0 = 0x00;   // 1Mbps
    config.Timing1 = 0x14;
    config.Mode = 0;
    if (std::system("which usbreset >/dev/null 2>&1") != 0) {
        fprintf(stderr, "[can] 警告: 未找到 usbreset，CAN 设备可能因 'Device or resource busy' 无法打开\n");
        fprintf(stderr, "[can] 请安装 usbreset (如: sudo apt install usbutils 或从源码编译)\n");
    }
    std::system("usbreset 04d8:0053 2>/dev/null");
    usleep(200000);
    if (VCI_OpenDevice(CAN_DEV_TYPE, CAN_DEV_IND, 0) != STATUS_OK) {
        fprintf(stderr, "[can] VCI_OpenDevice failed\n"); return;
    }
    if (VCI_InitCAN(CAN_DEV_TYPE, CAN_DEV_IND, CAN_CH, &config) != STATUS_OK) {
        fprintf(stderr, "[can] VCI_InitCAN failed\n"); VCI_CloseDevice(CAN_DEV_TYPE, CAN_DEV_IND); return;
    }
    if (VCI_StartCAN(CAN_DEV_TYPE, CAN_DEV_IND, CAN_CH) != STATUS_OK) {
        fprintf(stderr, "[can] VCI_StartCAN failed\n"); VCI_CloseDevice(CAN_DEV_TYPE, CAN_DEV_IND); return;
    }
    VCI_ClearBuffer(CAN_DEV_TYPE, CAN_DEV_IND, CAN_CH);
    can_opened = true;
    usleep(200000);
    // NMT pre-op + standard CANopen enable
    BYTE nmt[]={0x80,0x53}; can_send(0x000,nmt,2);
    usleep(200000);
    BYTE set_pv[]={0x2F,0x60,0x60,0x00,0x03,0x00,0x00,0x00}; can_send(0x653,set_pv,8); usleep(50000);
    // 2026-08-05: 显式设置轮廓加减速 — 与 motor_remote.py 一致 (50000)。
    // 驱动器默认值太温柔时 PV 模式下电机缓慢爬坡, 200Hz指令再快也跟不上
    // (实测: 蹬地指令500°/s收线76mm, 线张力纹丝不动)
    BYTE set_acc[]={0x23,0x83,0x60,0x00,0x50,0xC3,0x00,0x00}; can_send(0x653,set_acc,8); usleep(50000);
    BYTE set_dec[]={0x23,0x84,0x60,0x00,0x50,0xC3,0x00,0x00}; can_send(0x653,set_dec,8); usleep(50000);
    BYTE shutdown[]={0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00}; can_send(0x653,shutdown,8); usleep(80000);
    BYTE swon[]={0x2B,0x40,0x60,0x00,0x07,0x00,0x00,0x00}; can_send(0x653,swon,8); usleep(80000);
    BYTE enable[]={0x2B,0x40,0x60,0x00,0x0F,0x00,0x00,0x00}; can_send(0x653,enable,8); usleep(80000);
    fprintf(stderr, "[can] native CAN opened, motor enabled\n");
    pthread_create(&threadid, NULL, receive_func, NULL);
    pthread_detach(threadid);
}

inline void can_send(DWORD id, const BYTE* data, int len) {
    if (!can_opened) return;
    std::lock_guard<std::recursive_mutex> lk(can_mutex);
    VCI_CAN_OBJ obj;
    memset(&obj, 0, sizeof(obj));
    obj.ID = id;
    obj.ExternFlag = 0;
    obj.RemoteFlag = 0;
    obj.SendType = 0;
    obj.DataLen = (len > 8 ? 8 : len);
    for (int i = 0; i < obj.DataLen; i++) obj.Data[i] = data[i];
    VCI_Transmit(CAN_DEV_TYPE, CAN_DEV_IND, CAN_CH, &obj, 1);
}

inline int can_recv(DWORD* id, BYTE* data, int timeout_ms) {
    if (!can_opened) return -1;
    std::lock_guard<std::recursive_mutex> lk(can_mutex);
    VCI_CAN_OBJ obj;
    memset(&obj, 0, sizeof(obj));
    int n = VCI_Receive(CAN_DEV_TYPE, CAN_DEV_IND, CAN_CH, &obj, 1, timeout_ms);
    if (n <= 0) return 0;
    *id = obj.ID;
    int len = obj.DataLen > 8 ? 8 : obj.DataLen;
    for (int i = 0; i < len; i++) data[i] = obj.Data[i];
    return len;
}

inline void can_close(void) {
    if (can_opened) {
        std::lock_guard<std::recursive_mutex> lk(can_mutex);
        VCI_CloseDevice(CAN_DEV_TYPE, CAN_DEV_IND);
        can_opened = false;
    }
}

// ====== VCI-compatible wrappers ======
inline void Init_Can(void) { can_init(); }
inline void SendData(VCI_CAN_OBJ &h, const int id, const BYTE *data) { can_send((DWORD)id, data, 8); }
inline void SendData_five(VCI_CAN_OBJ &h, const int id, const BYTE *data) { can_send((DWORD)id, data, 5); }
inline void SendData_two(VCI_CAN_OBJ &h, const int id, const BYTE *data) { can_send((DWORD)id, data, 2); }

inline void *receive_func(void *param) {
    (void)param;
    while (rclcpp::ok()) {
        DWORD id; BYTE data[8];
        // 2026-08-05: 改为非阻塞接收 + 无数据时让出 1ms。
        // 原 10ms 阻塞接收在持锁状态下空调用, 解锁-重锁间隙为纳秒级,
        // 主线程 can_send 几乎永远抢不到 can_mutex → 主循环被饿死 (实测跌至 2.5Hz)
        int len = can_recv(&id, data, 0);
        if (len <= 0) { usleep(1000); continue; }
        // 2026-08-05: 捕获 SDO 响应 (0x580+0x53), 供启动自诊断读取驱动器参数
        if (id == 0x00000583 && len >= 8) {
            memcpy((void*)sdo_resp_data, data, 8);
            sdo_resp_ready = true;
            continue;
        }
        if (id == 0x000001D3 && len >= 8) {
            uint16_t ct = (data[3]<<8)|data[2];
            uint32_t cp = (data[7]<<24)|(data[6]<<16)|(data[5]<<8)|data[4];
            Output_TorqueValue = (int16_t)ct * (0.12/0x0010) * 0.001;
            Output_Angle = (double)(int32_t)cp * 360.0 / (0x00040000 * 100);
        } else if (id == 0x000003D3 && len >= 5) {
            uint32_t cv = (data[4]<<24)|(data[3]<<16)|(data[2]<<8)|data[1];
            double rpm = (0.6/0x00040000) * (int32_t)cv;
            Output_VelocityValue = 6.0 * rpm;
        }
    }
    return NULL;
}
