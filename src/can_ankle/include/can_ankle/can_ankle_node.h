#pragma once

#include <rclcpp/rclcpp.hpp>
#include <algorithm>
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
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

extern "C"
{
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "controlcan.h"
}

using namespace std;
using namespace chrono;

VCI_INIT_CONFIG config;

VCI_CAN_OBJ send_motor_torque;
VCI_CAN_OBJ config_node;

BYTE config_motor1[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
BYTE config_motor2[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
BYTE config_motor3[8] = {0x7F, 0xFF, 0x7F, 0xF0,0x00, 0x00, 0x07, 0xFF};
BYTE config_motor4[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};

uint16_t output[8];
uint16_t velocity_output[5];
double torque_value;
double velocity_value;
double kd;
double kp;
double P_max=95.5;
double P_min=-95.5;
double T_max=40;
double T_min=0;
double V_max=180;
double V_min=-180;
double min_duration = 0.3;
double max_duration = 0.9;
double Output_TorqueValue;
double Output_VelocityValue;
double Output_Angle;
float SensorTorque;
double adjusted_torque;
double adjusted_velocity;

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
void sendTorTransVelCommand(double y);
void sendVelocityCommand(double y);
vector<uint8_t> speed_to_command(double speed);

void Init_Can(void);
void *receive_func(void *param);
void SendData(VCI_CAN_OBJ &handle_obj, const int id, const BYTE *data);
void SendData_five(VCI_CAN_OBJ &handle_obj, const int id, const BYTE *data);
void SendData_two(VCI_CAN_OBJ &handle_obj, const int id, const BYTE *data);

pthread_t threadid;
