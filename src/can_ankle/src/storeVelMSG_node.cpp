#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <can_ankle/msg/torque.hpp>
#include <fstream>
#include <sstream>
#include <chrono>
#include <signal.h>
#include <atomic>

std::atomic<bool> is_running{true};

void signalHandler(int signum)
{
    is_running = false;
    rclcpp::shutdown();
}

class StoreDataSubscriber : public rclcpp::Node
{
public:
    StoreDataSubscriber() : Node("storeVelMSG_node")
    {
        encoder_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "angle", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
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
            });

        torque_info_sub_ = this->create_subscription<can_ankle::msg::Torque>(
            "torque_info", 10,
            [this](const can_ankle::msg::Torque::SharedPtr msg) {
                if (!is_running) return;
                std::ofstream file;
                file.open("torque.txt", std::ios::app);
                if (file.is_open())
                {
                    std::ostringstream oss;
                    auto now = std::chrono::steady_clock::now();
                    oss << now.time_since_epoch().count() << ","
                        << "目标速度: " << msg->velocity_value << " 度/s, "
                        << "返回速度: " << msg->return_velocity << " 度/s" << std::endl
                        << "目标扭矩: " << msg->torque_value << " Nm, "
                        << "返回扭矩: " << msg->force_sensortorque << " Nm" << std::endl;
                    file << oss.str();
                    file.close();
                }
            });
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr encoder_sub_;
    rclcpp::Subscription<can_ankle::msg::Torque>::SharedPtr torque_info_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    signal(SIGINT, signalHandler);
    auto node = std::make_shared<StoreDataSubscriber>();
    RCLCPP_INFO(node->get_logger(), "StoreVelMSG node started, writing to encoder.txt / torque.txt");
    rclcpp::spin(node);
    RCLCPP_INFO(node->get_logger(), "StoreVelMSG node shutdown");
    rclcpp::shutdown();
    return 0;
}
