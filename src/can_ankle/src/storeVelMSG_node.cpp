#include <rclcpp/rclcpp.hpp>
#include <can_ankle/msg/torque.hpp>
#include <std_msgs/msg/float64.hpp>
#include <fstream>
#include <chrono>
#include <signal.h>

std::atomic<bool> is_running{true};
void sigHandler(int) { is_running = false; rclcpp::shutdown(); }

class StoreData : public rclcpp::Node {
public:
    StoreData() : Node("storeVelMSG_node") {
        enc_sub_ = this->create_subscription<std_msgs::msg::Float64>("angle", 10,
            [this](const std::shared_ptr<const std_msgs::msg::Float64> m) {
                if (!is_running) return;
                std::ofstream f("encoder.txt", std::ios::app);
                if (f.is_open()) f << std::chrono::steady_clock::now().time_since_epoch().count()
                    << ",编码器角度: " << m->data << " 度" << std::endl;
            });
        tor_sub_ = this->create_subscription<can_ankle::msg::Torque>("torque_info", 10,
            [this](const std::shared_ptr<const can_ankle::msg::Torque> m) {
                if (!is_running) return;
                std::ofstream f("torque.txt", std::ios::app);
                if (f.is_open()) f << std::chrono::steady_clock::now().time_since_epoch().count()
                    << ",目标速度: " << m->velocity_value << " 度/s, "
                    << "返回速度: " << m->return_velocity << " 度/s" << std::endl
                    << "目标扭矩: " << m->torque_value << " Nm, "
                    << "返回扭矩: " << m->force_sensortorque << " Nm" << std::endl;
            });
    }
private:
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr enc_sub_;
    rclcpp::Subscription<can_ankle::msg::Torque>::SharedPtr tor_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    signal(SIGINT, sigHandler);
    rclcpp::spin(std::make_shared<StoreData>());
    rclcpp::shutdown();
    return 0;
}
