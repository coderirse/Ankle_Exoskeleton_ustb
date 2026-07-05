#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <fstream>
#include <sstream>
#include <chrono>
#include <signal.h>

std::atomic<bool> is_running{true};
void sigHandler(int) { is_running = false; rclcpp::shutdown(); }

void writeFile(const std::string& s) {
    if (!is_running) return;
    std::ofstream f("time_data.txt", std::ios::app);
    if (f.is_open()) f << std::chrono::steady_clock::now().time_since_epoch().count() << "," << s << std::endl;
}

static void onesupport(const std_msgs::msg::Float64::SharedPtr m) { writeFile("支撑相1: " + std::to_string(m->data)); }
static void twosupport(const std_msgs::msg::Float64::SharedPtr m) {
    writeFile("支撑相2: " + std::to_string(m->data));
    double a=1.45022, b=0.469537, c=0.06345;
    double pace = log(a/(m->data-c)) / b;
    writeFile("步速: " + std::to_string(pace));
}
static void threesupport(const std_msgs::msg::Float64::SharedPtr m) { writeFile("支撑相3: " + std::to_string(m->data)); }
static void fullsupport(const std_msgs::msg::Float64::SharedPtr m) { writeFile("支撑相总计: " + std::to_string(m->data)); }
static void swing(const std_msgs::msg::Float64::SharedPtr m) { writeFile("摆动相: " + std::to_string(m->data)); }

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    signal(SIGINT, sigHandler);
    auto node = rclcpp::Node::make_shared("topic_recorder");
    auto s1 = node->create_subscription<std_msgs::msg::Float64>("one_support_time", 10, onesupport);
    auto s2 = node->create_subscription<std_msgs::msg::Float64>("two_support_time", 10, twosupport);
    auto s3 = node->create_subscription<std_msgs::msg::Float64>("three_support_time", 10, threesupport);
    auto s4 = node->create_subscription<std_msgs::msg::Float64>("support_time", 10, fullsupport);
    auto s5 = node->create_subscription<std_msgs::msg::Float64>("swing_time", 10, swing);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
