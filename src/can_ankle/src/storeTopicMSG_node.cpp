#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <fstream>
#include <sstream>
#include <signal.h>
#include <chrono>
#include <atomic>

std::atomic<bool> is_running{true};

void signalHandler(int signum)
{
    is_running = false;
    rclcpp::shutdown();
}

void writeTimeToFile(const std_msgs::msg::Float64::SharedPtr msg, const std::string& phase_name)
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

class TimeDataSubscriber : public rclcpp::Node
{
public:
    TimeDataSubscriber() : Node("storeTopicMSG_node")
    {
        one_support_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "one_support_time", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                writeTimeToFile(msg, "支撑相阶段1");
            });

        two_support_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "two_support_time", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                writeTimeToFile(msg, "支撑相阶段2");
                const double a1 = 1.45022;
                const double b1 = 0.469537;
                const double c1 = 0.06345;
                double i1 = a1 / (msg->data - c1);
                double j1 = log(i1) / b1;
                double realpace = j1;
                writeVelocityToFile(realpace, "解算得到的步速");
            });

        three_support_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "three_support_time", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                writeTimeToFile(msg, "支撑相阶段3");
            });

        swing_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "swing_time", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                writeTimeToFile(msg, "摆动相时间");
            });

        support_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "support_time", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                writeTimeToFile(msg, "支撑相总时间");
            });
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr one_support_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr two_support_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr three_support_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr swing_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr support_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    signal(SIGINT, signalHandler);
    auto node = std::make_shared<TimeDataSubscriber>();
    RCLCPP_INFO(node->get_logger(), "Topic recorder node started, writing to time_data.txt");
    rclcpp::spin(node);
    RCLCPP_INFO(node->get_logger(), "Topic recorder node shutdown");
    rclcpp::shutdown();
    return 0;
}
