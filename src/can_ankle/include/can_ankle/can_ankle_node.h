#include <ros/ros.h>
#include "signal.h"
#include "geometry_msgs/Twist.h"
#include <serial/serial.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <chrono>
#include <ratio>
#include <ctime>
#include <thread>
#include <mutex>
#include <geometry_msgs/TransformStamped.h> 
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <atomic>
#include <queue>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include<std_msgs/String.h>
#include<sensor_msgs/Imu.h>
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

BYTE config_motor1[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}; //  启动电机
BYTE config_motor2[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD}; //  终止电机
BYTE config_motor3[8] = {0x7F, 0xFF, 0x7F, 0xF0,0x00, 0x00, 0x07, 0xFF}; //  V、P、T均为0
BYTE config_motor4[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}; //  设置机械零位

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

bool checkCommandSequence(); //开关顺序安全检查

void commandCallback(const std_msgs::UInt8::ConstPtr& msg);  //开关指令接收函数

void torqueCallback(const std_msgs::Float32::ConstPtr& msg); //力传感器接收函数

void encoderCallback(const std_msgs::Float64::ConstPtr& msg); //编码器接收函数

void one_support_timeCallback(const std_msgs::Float64::ConstPtr& msg);//支撑相阶段1时间回调函数

void two_support_timeCallback(const std_msgs::Float64::ConstPtr& msg);//支撑相阶段2时间回调函数

void three_support_timeCallback(const std_msgs::Float64::ConstPtr& msg);//支撑相阶段3时间回调函数

void swing_timeCallback(const std_msgs::Float64::ConstPtr& msg);//摆动相时间回调函数

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);//imu角速度和线加速度回调函数

double calculateTargetPosition(); //目标扭矩函数

double calculateTargetTorque(); //目标位置函数

double TargetTorque(); //目标位置函数(无速度自适应)

double calculateSupportPosition(); //电机启动位置补偿函数

void motor_on_V();  //速度使能函数

void motor_on_P();  //位置使能函数

void motor_on_T();  //扭矩使能函数

void sendMSG(); //can总线配置

void processHoming();// 退出归零函数

void AdaptiveSpeed(double x);//实时速度解算函数(自适应步速)

double updateTorquePeak();//动态调整峰值扭矩函数

void getUserWeight();//体重转换峰值扭矩函数

void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);//imu线速度回调函数

vector<uint8_t> speed_to_command(double speed); //速度指令转换函数

void sendVelocityCommand(double y);//发送速度指令函数

void updateCompensateTorque();//补偿位置输出函数

void sendTorTransVelCommand(double y);//发送扭矩跟踪转换电机速度指令函数

void Init_Can(void);                                                    // CAN初始化设置
void *receive_func(void *param);                                        // CAN接收函数，线程开启
void SendData(VCI_CAN_OBJ &handle_obj, const int id, const BYTE *data); // 下发数据封装的函数，id是CAN完整ID，数据是要发送的八位数据
void SendData_five(VCI_CAN_OBJ &handle_obj, const int id, const BYTE *data);//五位数据
void SendData_two(VCI_CAN_OBJ &handle_obj, const int id, const BYTE *data);//二位数据

pthread_t threadid;
