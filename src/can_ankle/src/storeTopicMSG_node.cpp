#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <fstream>
#include <sstream>
#include "signal.h"
#include <chrono>

volatile bool is_running = true;

void signalHandler(int signum) 
{
    is_running = false;
    ros::shutdown();
}

// 定义回调函数，用于处理不同阶段的时间消息
void writeTimeToFile(const std_msgs::Float64::ConstPtr& msg, const std::string& phase_name) 
{
    if (!is_running) return;
    std::ofstream file;
    file.open("time_data.txt", std::ios::app);
    if (file.is_open()) 
    {
        std::ostringstream oss;
        auto now = std::chrono::steady_clock::now();
        oss << now.time_since_epoch().count() << "," << phase_name << "时间: " << msg->data << " 秒" << std::endl;
        file << oss.str();
        file.close();
    }
}

void writeVelocityToFile(double result, const std::string& phase_name) 
{
    if (!is_running) return;
    std::ofstream file;
    file.open("time_data.txt", std::ios::app);
    if (file.is_open()) 
    {
        std::ostringstream oss;
        auto now = std::chrono::steady_clock::now();
        oss << now.time_since_epoch().count() << "," << phase_name << "：" << result << std::endl;
        file << oss.str();
        file.close();
    }
}

void onesupportCallback(const std_msgs::Float64::ConstPtr& msg)
{
    writeTimeToFile(msg, "支撑相阶段1");
}

void twosupportCallback(const std_msgs::Float64::ConstPtr& msg)
{
    writeTimeToFile(msg, "支撑相阶段2");
    const double a1 = 1.45022;
    const double b1 = 0.469537;
    const double c1 = 0.06345;
    double i1 = a1 / (msg->data - c1);
    double j1 = log(i1) / b1;
    double realpace = j1; //解算实时速度
    writeVelocityToFile(realpace, "解算得到的步速");
}

void threesupportCallback(const std_msgs::Float64::ConstPtr& msg)
{
    writeTimeToFile(msg, "支撑相阶段3");
}

void fullsupportCallback(const std_msgs::Float64::ConstPtr& msg)
{
    writeTimeToFile(msg, "支撑相总时间");
}

void swingCallback(const std_msgs::Float64::ConstPtr& msg)
{
    writeTimeToFile(msg, "摆动相时间");
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "topic_recorder");
    ros::NodeHandle n;
    signal(SIGINT,signalHandler);
    // 订阅各个时间话题，并为每个话题指定对应的中文标注
    ros::Subscriber one_support_sub = n.subscribe("one_support_time", 10, onesupportCallback);
    ros::Subscriber two_support_sub = n.subscribe("two_support_time", 10, twosupportCallback);
    ros::Subscriber three_support_sub = n.subscribe("three_support_time", 10, threesupportCallback);
    ros::Subscriber swing_sub = n.subscribe("swing_time", 10, swingCallback);
    ros::Subscriber support_sub = n.subscribe("support_time", 10, fullsupportCallback);

    ros::spin();
    return 0;
}

