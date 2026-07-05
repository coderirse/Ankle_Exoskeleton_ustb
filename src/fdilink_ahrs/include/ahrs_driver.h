#ifndef BASE_DRIVER_H_
#define BASE_DRIVER_H_

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <serial/serial.h>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <string>

#define PI 3.141592653589793

namespace FDILink {

class ahrsBringup : public rclcpp::Node
{
public:
    ahrsBringup();
    ~ahrsBringup();
    void processLoop();

private:
    bool if_debug_;
    int device_type_ = 1;

    serial::Serial serial_;
    std::string serial_port_;
    int serial_baud_;
    int serial_timeout_;
    bool frist_sn_;

    std::string imu_topic_, mag_pose_2d_topic_;
    std::string Euler_angles_topic_, Magnetic_topic_;
    std::string gps_topic_, twist_topic_, NED_odom_topic_;
    std::string imu_frame_id_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr mag_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr Euler_angles_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr Magnetic_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr NED_odom_pub_;
};

} // namespace FDILink
#endif
