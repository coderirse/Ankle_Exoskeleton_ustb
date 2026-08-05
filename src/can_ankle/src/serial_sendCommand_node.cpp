#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float64.hpp>
#include <serial/serial.h>
#include <iostream>
#include <iomanip>
#include <signal.h>
#include <unistd.h>
#include <chrono>
#include <string>

serial::Serial ser;
std::chrono::steady_clock::time_point phase_start_time;
std::chrono::steady_clock::time_point support_start_time;

// State variables
uint8_t prev_command = 0x00;
bool waiting_for_start = false;
bool valid_command_received = false;
uint8_t current_valid_command = 0x00;

// 2026-08-03: 心跳固件适配
// STM32 心跳版固件会周期性重发当前状态, 需去重;
// 开关当前状态通过 /switch_state 发布 (供控制节点初始化站立确认使用)
uint8_t last_raw_byte = 0x00;   // 上一个收到的原始字节
bool    first_byte = true;      // 是否尚未收到任何字节

void signalHandler(int signum)
{
    std::cout << "Closing..." << std::endl;
    usleep(100000);
    rclcpp::shutdown();
}

void publishPhaseTime(rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub,
                      const std::string& phase_name, double duration)
{
    auto msg = std_msgs::msg::Float64();
    msg.data = duration;
    pub->publish(msg);
    RCLCPP_INFO(rclcpp::Node::make_shared("dummy")->get_logger(),
                "%s time: %.9f seconds", phase_name.c_str(), duration);
}

void resetToStart()
{
    prev_command = 0x00;
    waiting_for_start = true;
    valid_command_received = false;
    RCLCPP_WARN(rclcpp::Node::make_shared("dummy")->get_logger(),
                "Sequence broken, waiting for next 0x41 to restart");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("serial_sendCommand_node");

    auto resolution = std::chrono::steady_clock::period::num /
                     static_cast<double>(std::chrono::steady_clock::period::den);
    std::cout << "system clock resolution:" << resolution << "seconds" << std::endl;

    // Publishers
    auto command_pub = node->create_publisher<std_msgs::msg::UInt8>("command_topic", 10);
    auto switch_state_pub = node->create_publisher<std_msgs::msg::UInt8>("switch_state", 10); // 2026-08-03: 开关原始状态(含心跳)
    auto one_support_pub = node->create_publisher<std_msgs::msg::Float64>("one_support_time", 10);
    auto two_support_pub = node->create_publisher<std_msgs::msg::Float64>("two_support_time", 10);
    auto three_support_pub = node->create_publisher<std_msgs::msg::Float64>("three_support_time", 10);
    auto swing_pub = node->create_publisher<std_msgs::msg::Float64>("swing_time", 10);
    auto support_pub = node->create_publisher<std_msgs::msg::Float64>("support_time", 10);

    rclcpp::WallRate loop_rate(std::chrono::milliseconds(1)); // ~1000 Hz polling
    signal(SIGINT, signalHandler);

    node->declare_parameter<std::string>("port", "/dev/ankle_switch");
    std::string port = node->get_parameter("port").as_string();

    try
    {
        ser.setPort(port); // 脚底开关 (2026-07-29 经拓展坞实测确认)
        ser.setBaudrate(9600);
        ser.setBytesize(serial::eightbits);
        ser.setParity(serial::parity_none);
        ser.setStopbits(serial::stopbits_one);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(timeout);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Port open failed: " << e.what());
        return -1;
    }

    if (ser.isOpen())
    {
        RCLCPP_INFO_STREAM(node->get_logger(), "Serial port initialized on " << port);
        uint8_t byte;

        while (rclcpp::ok())
        {
            if (ser.available())
            {
                ser.read(&byte, 1);

                // 2026-08-05: 合法指令集校验 — 非法字节(线路噪声/帧错误)只告警,
                // 不更新 last_raw_byte、不发布, 避免污染心跳去重基准:
                // 否则干扰字节会让随后的心跳重发被误判为"新跳变", 触发 resetToStart 打断步态序列
                if (byte < 0x41 || byte > 0x45)
                {
                    RCLCPP_WARN(node->get_logger(), "Invalid byte 0x%02X ignored (not in 0x41~0x45)", byte);
                    rclcpp::spin_some(node);
                    loop_rate.sleep();
                    continue;
                }

                // 2026-08-03: 心跳固件适配 — 合法字节都发布到 /switch_state,
                // 但与上一字节相同视为心跳重复, 跳过步态序列处理
                {
                    auto state_msg = std_msgs::msg::UInt8();
                    state_msg.data = byte;
                    switch_state_pub->publish(state_msg);
                }
                if (!first_byte && byte == last_raw_byte)
                {
                    rclcpp::spin_some(node);
                    loop_rate.sleep();
                    continue;  // 心跳重复, 不进入步态序列
                }
                first_byte = false;
                last_raw_byte = byte;

                // Special command 0x45 — bypass ordering
                if (byte == 0x45)
                {
                    RCLCPP_WARN(node->get_logger(), "Received special command: E");
                    auto cmd_msg = std_msgs::msg::UInt8();
                    cmd_msg.data = byte;
                    command_pub->publish(cmd_msg);
                    continue;
                }

                // Handle 0x41 (start)
                if (byte == 0x41)
                {
                    waiting_for_start = false;
                    prev_command = 0x41;
                    valid_command_received = true;
                    current_valid_command = byte;

                    std::cout << "接收指令：" << (int)byte << std::endl;
                    auto now = std::chrono::steady_clock::now();
                    std::chrono::duration<double> swing_duration = now - phase_start_time;

                    auto swing_msg = std_msgs::msg::Float64();
                    swing_msg.data = swing_duration.count();
                    swing_pub->publish(swing_msg);
                    RCLCPP_INFO(node->get_logger(), "Swing Phase time: %.9f seconds", swing_duration.count());

                    phase_start_time = now;
                    support_start_time = now;
                    std::cout << "摆动相时间：" << std::fixed << std::setprecision(9)
                              << swing_duration.count() << std::endl;
                    RCLCPP_INFO(node->get_logger(), "FULL_SUPPORT started");
                }
                // Handle 0x42 (needs 0x41 first)
                else if (byte == 0x42)
                {
                    if (prev_command == 0x41 && !waiting_for_start)
                    {
                        prev_command = 0x42;
                        valid_command_received = true;
                        current_valid_command = byte;

                        std::cout << "接收指令：" << (int)byte << std::endl;
                        auto now = std::chrono::steady_clock::now();
                        std::chrono::duration<double> onesupport_duration = now - phase_start_time;

                        auto msg = std_msgs::msg::Float64();
                        msg.data = onesupport_duration.count();
                        one_support_pub->publish(msg);
                        RCLCPP_INFO(node->get_logger(), "Support Phase One time: %.9f seconds", onesupport_duration.count());

                        phase_start_time = now;
                        RCLCPP_INFO(node->get_logger(), "TENSION_TORQUE started");
                    }
                    else
                    {
                        resetToStart();
                    }
                }
                // Handle 0x43 (needs 0x42 first)
                else if (byte == 0x43)
                {
                    if (prev_command == 0x42 && !waiting_for_start)
                    {
                        prev_command = 0x43;
                        valid_command_received = true;
                        current_valid_command = byte;

                        std::cout << "接收指令：" << (int)byte << std::endl;
                        auto now = std::chrono::steady_clock::now();
                        std::chrono::duration<double> twosupport_duration = now - phase_start_time;

                        auto msg = std_msgs::msg::Float64();
                        msg.data = twosupport_duration.count();
                        two_support_pub->publish(msg);
                        RCLCPP_INFO(node->get_logger(), "Support Phase Two time: %.9f seconds", twosupport_duration.count());

                        phase_start_time = now;
                        RCLCPP_INFO(node->get_logger(), "TORQUE_CURVE started");
                    }
                    else
                    {
                        resetToStart();
                    }
                }
                // Handle 0x44 (needs 0x43 first)
                else if (byte == 0x44)
                {
                    if (prev_command == 0x43 && !waiting_for_start)
                    {
                        prev_command = 0x44;
                        valid_command_received = true;
                        current_valid_command = byte;

                        std::cout << "接收指令：" << (int)byte << std::endl;
                        auto now = std::chrono::steady_clock::now();
                        std::chrono::duration<double> threesupport_duration = now - phase_start_time;

                        auto msg3 = std_msgs::msg::Float64();
                        msg3.data = threesupport_duration.count();
                        three_support_pub->publish(msg3);
                        RCLCPP_INFO(node->get_logger(), "Support Phase Three time: %.9f seconds", threesupport_duration.count());

                        phase_start_time = now;
                        RCLCPP_INFO(node->get_logger(), "SWING_PHASE started");

                        std::chrono::duration<double> support_duration = now - support_start_time;
                        auto msg_support = std_msgs::msg::Float64();
                        msg_support.data = support_duration.count();
                        support_pub->publish(msg_support);
                        RCLCPP_INFO(node->get_logger(), "Support Phase total: %.9f seconds", support_duration.count());

                        std::cout << "支撑相时间：" << std::fixed << std::setprecision(9)
                                  << support_duration.count() << std::endl;
                    }
                    else
                    {
                        resetToStart();
                    }
                }
                // 注: 0x41~0x44 均由上方分支处理, 0x45 提前 continue,
                // 非法字节已在读取处被过滤, 此处不再需要 Unknown command 分支

                // Publish valid command if received
                if (valid_command_received)
                {
                    auto cmd_msg = std_msgs::msg::UInt8();
                    cmd_msg.data = current_valid_command;
                    command_pub->publish(cmd_msg);
                    valid_command_received = false;
                }
            }
            rclcpp::spin_some(node);
            loop_rate.sleep();
        }
    }

    ser.close();
    RCLCPP_INFO(node->get_logger(), "Serial SendCommand Node shutdown");
    rclcpp::shutdown();
    return 0;
}
