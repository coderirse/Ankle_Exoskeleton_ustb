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
    auto one_support_pub = node->create_publisher<std_msgs::msg::Float64>("one_support_time", 10);
    auto two_support_pub = node->create_publisher<std_msgs::msg::Float64>("two_support_time", 10);
    auto three_support_pub = node->create_publisher<std_msgs::msg::Float64>("three_support_time", 10);
    auto swing_pub = node->create_publisher<std_msgs::msg::Float64>("swing_time", 10);
    auto support_pub = node->create_publisher<std_msgs::msg::Float64>("support_time", 10);

    rclcpp::WallRate loop_rate(std::chrono::milliseconds(1)); // ~1000 Hz polling
    signal(SIGINT, signalHandler);

    try
    {
        ser.setPort("/dev/ttyCH341USB1"); // 脚底开关 - 实际端口待确认
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
        RCLCPP_INFO_STREAM(node->get_logger(), "Serial port initialized");
        uint8_t byte;

        while (rclcpp::ok())
        {
            if (ser.available())
            {
                ser.read(&byte, 1);

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
                else
                {
                    RCLCPP_WARN(node->get_logger(), "Unknown command: 0x%02X", byte);
                }

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
