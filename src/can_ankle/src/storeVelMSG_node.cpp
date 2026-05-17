#include <ros/ros.h>
#include <fstream>
#include <chrono>
#include <sstream>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include "signal.h"
#include "can_ankle/Torque.h" 

bool is_running = true;

// 信号处理函数，用于优雅退出
void signalHandler(int signum) 
{
    is_running = false;
    ros::shutdown();
}

// 写入编码器数据到encoder.txt
void encoderCallback(const std_msgs::Float64::ConstPtr& msg) 
{
    if (!is_running) return;
    std::ofstream file;
    file.open("encoder.txt", std::ios::app);
    if (file.is_open()) 
    {
        std::ostringstream oss;
        auto now = std::chrono::steady_clock::now();
        oss << now.time_since_epoch().count() << "," 
            << "编码器角度: " << msg->data << " 度" << std::endl;
        file << oss.str();
        file.close();
    }
}


// 写入扭矩信息到torque.txt
void torqueInfoCallback(const can_ankle::Torque::ConstPtr& msg) 
{
    if (!is_running) return;
    std::ofstream file;
    file.open("torque.txt", std::ios::app);
    if (file.is_open()) 
    {
        std::ostringstream oss;
        auto now = std::chrono::steady_clock::now();
        oss << now.time_since_epoch().count() << "," 
            << "目标速度: " << msg->VelocityValue << " 度/s, "
            << "返回速度: " << msg->ReturnVelocity << " 度/s" << std::endl
            << "目标扭矩: " << msg->TorqueValue << " Nm, "
            << "返回扭矩: " << msg->ForceSensortorque << " Nm" << std::endl;
        file << oss.str();
        file.close();
    }
}

// 写入IMU角速度数据到imu.txt
/*void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) 
{
    if (!is_running) return;
    std::ofstream file;
    file.open("imu.txt", std::ios::app);
    if (file.is_open()) 
    {
        std::ostringstream oss;
        auto now = std::chrono::steady_clock::now();
        oss << now.time_since_epoch().count() << "," 
            << "角速度X: " << msg->angular_velocity.x << " rad/s, "
            << "角速度Y: " << msg->angular_velocity.y << " rad/s, "
            << "角速度Z: " << msg->angular_velocity.z << " rad/s" << std::endl;
        file << oss.str();
        file.close();
    }
}*/



int main(int argc, char **argv) 
{
    ros::init(argc, argv, "storeVelMSG_node");
    ros::NodeHandle n;
    signal(SIGINT, signalHandler);
    
    // 订阅各个话题
    ros::Subscriber encoder_sub = n.subscribe("angle", 10, encoderCallback);
    ros::Subscriber torque_info_sub = n.subscribe("torque_info", 10, torqueInfoCallback);
    //ros::Subscriber imu_sub = n.subscribe("imu", 10, imuCallback);

    ros::spin();
    return 0;
}