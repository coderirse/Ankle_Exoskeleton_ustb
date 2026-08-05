#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <serial/serial.h>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <signal.h>
#include <unistd.h>

serial::Serial ser;

void signalHandler(int signum)
{
    std::cout << "close" << std::endl;
    usleep(100000);
    rclcpp::shutdown();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("serial_encoder_node");
    auto angle_pub = node->create_publisher<std_msgs::msg::Float64>("angle", 100);
    rclcpp::WallRate loop_rate(std::chrono::milliseconds(5)); // 200 Hz
    signal(SIGINT, signalHandler);

    node->declare_parameter<std::string>("port", "/dev/ankle_encoder");
    std::string port = node->get_parameter("port").as_string();

    // Open serial port
    try
    {
        ser.setPort(port);
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
        RCLCPP_ERROR_STREAM(node->get_logger(), "无法打开串口: " << e.what());
        return -1;
    }

    if (ser.isOpen())
    {
        RCLCPP_INFO_STREAM(node->get_logger(), "Serial Port initialized (Modbus RTU, 9600) on " << port);

        while (rclcpp::ok())
        {
            // Modbus RTU request: read holding register 0x0000, 1 register
            uint8_t senddata[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};
            ser.write(senddata, 8);
            usleep(1000);

            uint8_t receivedata[7] = {0};
            ser.read(receivedata, 7);

            // Extract angle from response bytes [3] and [4]
            uint16_t combined_value = (receivedata[3] << 8) | receivedata[4];
            double encodervalue = static_cast<double>(combined_value) * 360.0 / 32768.0;
            if (encodervalue > 200)
            {
                encodervalue = encodervalue - 360;
            }

            // 2026-08-05: 不再打印到终端 (200Hz 刷屏淹没控制状态), 角度仅从 /angle 话题获取

            auto msg = std_msgs::msg::Float64();
            msg.data = encodervalue;
            angle_pub->publish(msg);

            rclcpp::spin_some(node);
            loop_rate.sleep();
        }

        ser.close();
        return 0;
    }
    else
    {
        return -1;
    }
}
