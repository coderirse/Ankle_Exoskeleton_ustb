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
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <unistd.h>
#include <spawn.h>
}

using namespace std;
using namespace chrono;

// ====== CAN types (compatible with original VCI API) ======
typedef uint8_t  BYTE;
typedef uint32_t DWORD;

typedef struct {
    DWORD ID;
    BYTE  Data[8];
    BYTE  DataLen;
    BYTE  SendType;
    BYTE  RemoteFlag;
    BYTE  ExternFlag;
    BYTE  Reserved[3];
} VCI_CAN_OBJ;

typedef struct {
    DWORD AccCode, AccMask, Reserved;
    BYTE  Filter, Timing0, Timing1, Mode;
} VCI_INIT_CONFIG;

// CAN globals

// ====== CAN subprocess I/O ======
void can_init(void);    // open popen to python-can helper
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


// ====== CAN subprocess implementation (replaces VCI_USBCAN2) ======
// Uses pipe+fork to python-can helper (bidirectional)

static int can_tx_fd = -1, can_rx_fd = -1;
static FILE* can_tx_fp = nullptr;
static FILE* can_rx_fp = nullptr;
static std::mutex can_mutex;

inline void can_init(void) {
    if (can_tx_fd >= 0) return;
    int tx_p[2], rx_p[2];
    pipe(tx_p); pipe(rx_p);
    posix_spawn_file_actions_t fa;
    posix_spawn_file_actions_init(&fa);
    posix_spawn_file_actions_addclose(&fa, tx_p[1]);
    posix_spawn_file_actions_addclose(&fa, rx_p[0]);
    posix_spawn_file_actions_adddup2(&fa, tx_p[0], 0);
    posix_spawn_file_actions_adddup2(&fa, rx_p[1], 1);
    char* argv[] = {(char*)"sudo", (char*)"-n", (char*)"python3", (char*)"/tmp/can_helper.py", NULL};
    extern char** environ;
    pid_t pid;
    posix_spawn(&pid, "/usr/bin/sudo", &fa, NULL, argv, environ);
    posix_spawn_file_actions_destroy(&fa);
    /* posix_spawn used instead of fork+exec */
    close(tx_p[0]); close(rx_p[1]);
    can_tx_fd = tx_p[1]; can_rx_fd = rx_p[0];
    can_tx_fp = fdopen(can_tx_fd, "w");
    can_rx_fp = fdopen(can_rx_fd, "r");
    setvbuf(can_tx_fp, NULL, _IONBF, 0);
    usleep(500000);
    // NMT pre-op + standard CANopen enable
    BYTE nmt[]={0x80,0x53}; can_send(0x000,nmt,2);
    usleep(200000);
    // Set PV mode and enable via SDO-like commands
    BYTE set_pv[]={0x2F,0x60,0x60,0x00,0x03,0x00,0x00,0x00}; can_send(0x653,set_pv,8); usleep(50000);
    BYTE shutdown[]={0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00}; can_send(0x653,shutdown,8); usleep(80000);
    BYTE swon[]={0x2B,0x40,0x60,0x00,0x07,0x00,0x00,0x00}; can_send(0x653,swon,8); usleep(80000);
    BYTE enable[]={0x2B,0x40,0x60,0x00,0x0F,0x00,0x00,0x00}; can_send(0x653,enable,8); usleep(80000);
    fprintf(stderr, "[can] subprocess started\n");
}

inline void can_send(DWORD id, const BYTE* data, int len) {
    if (!can_tx_fp) return;
    std::lock_guard<std::mutex> lk(can_mutex);
    fprintf(can_tx_fp, "TX %08X %d", (unsigned)id, len);
    for (int i = 0; i < len && i < 8; i++) fprintf(can_tx_fp, " %02X", data[i]);
    fprintf(can_tx_fp, "\n"); fflush(can_tx_fp);
    char ok[8]; fgets(ok, sizeof(ok), can_rx_fp);
}

inline int can_recv(DWORD* id, BYTE* data, int timeout_ms) {
    if (!can_rx_fp) return -1;
    std::lock_guard<std::mutex> lk(can_mutex);
    fprintf(can_tx_fp, "RX\n"); fflush(can_tx_fp);
    char buf[256]; int pos = 0;
    auto t0 = std::chrono::steady_clock::now();
    while (pos < (int)sizeof(buf)-1) {
        int c = fgetc(can_rx_fp);
        if (c == EOF) {
            clearerr(can_rx_fp);
            if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-t0).count() > timeout_ms) return -1;
            usleep(1000); continue;
        }
        if (c == '\n') { buf[pos] = '\0'; break; }
        buf[pos++] = (char)c;
    }
    if (strncmp(buf, "TIMEOUT", 7) == 0) return 0;
    unsigned rid, rdlc, d[8] = {};
    int n = sscanf(buf, "%X %u %x %x %x %x %x %x %x %x", &rid,&rdlc,&d[0],&d[1],&d[2],&d[3],&d[4],&d[5],&d[6],&d[7]);
    if (n >= 2) { *id = rid; for (int i=0; i<(int)rdlc&&i<8; i++) data[i]=(BYTE)d[i]; return (int)rdlc; }
    return -1;
}

inline void can_close(void) {
    if (can_tx_fp) { std::lock_guard<std::mutex> lk(can_mutex); fprintf(can_tx_fp, "QUIT\n"); fflush(can_tx_fp); fclose(can_tx_fp); fclose(can_rx_fp); can_tx_fp=nullptr; can_rx_fp=nullptr; }
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
        int len = can_recv(&id, data, 500);
        if (len <= 0) continue;
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
