// Force sensor node — D056 display + DYMH113 sensor via Modbus RTU
// Protocol: Modbus RTU, 19200 baud, slave 1, register 0x0FA1 (signed int16)

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <serial/serial.h>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <sstream>

serial::Serial ser;

static uint16_t modbus_crc(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

std::string to_hex_string(const uint8_t* data, size_t len) {
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (size_t i = 0; i < len; i++) {
        ss << std::setw(2) << static_cast<int>(data[i]) << " ";
    }
    return ss.str();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("serial_ForceSensor_node");

    // Declare parameters
    node->declare_parameter<std::string>("port", "/dev/ankle_force");
    node->declare_parameter<int>("baudrate", 19200);
    node->declare_parameter<int>("slave_id", 1);
    node->declare_parameter<double>("scale_factor", 0.01);
    node->declare_parameter<bool>("debug", false);

    std::string port = node->get_parameter("port").as_string();
    int baudrate = node->get_parameter("baudrate").as_int();
    int slave_id = node->get_parameter("slave_id").as_int();
    double scale_factor = node->get_parameter("scale_factor").as_double();
    bool debug = node->get_parameter("debug").as_bool();

    auto force_pub = node->create_publisher<std_msgs::msg::Float32>("Force", 10);
    // 2026-08-06: 20Hz→100Hz。电机斜坡关闭后响应即时, 50ms力反馈成为控制瓶颈
    // (实测: 驱动收线时100ms内张力可冲10kg+, 反馈太慢拦不住)。
    // 19200波特一单帧约5ms, 100Hz 完全够用
    rclcpp::WallRate loop_rate(std::chrono::milliseconds(10));

    try {
        ser.setPort(port);
        ser.setBaudrate(baudrate);
        ser.setBytesize(serial::eightbits);
        ser.setParity(serial::parity_none);
        ser.setStopbits(serial::stopbits_one);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
        ser.setTimeout(timeout);
        ser.open();
    }
    catch (serial::IOException& e) {
        RCLCPP_ERROR(node->get_logger(), "无法打开串口 %s: %s", port.c_str(), e.what());
        return -1;
    }

    if (!ser.isOpen()) {
        RCLCPP_ERROR(node->get_logger(), "串口未打开: %s", port.c_str());
        return -1;
    }
    RCLCPP_INFO(node->get_logger(), "Force sensor serial port initialized: %s @ %d", port.c_str(), baudrate);

    while (rclcpp::ok()) {
        // Build Modbus RTU request: SlaveID 03 0F A0 00 02 CRC_L CRC_H
        uint8_t req[8] = {static_cast<uint8_t>(slave_id), 0x03, 0x0F, 0xA0, 0x00, 0x02, 0x00, 0x00};
        uint16_t crc = modbus_crc(req, 6);
        req[6] = crc & 0xFF;
        req[7] = (crc >> 8) & 0xFF;

        ser.flushInput();
        ser.write(req, 8);
        
        if (debug) {
            RCLCPP_DEBUG(node->get_logger(), "TX: %s", to_hex_string(req, 8).c_str());
        }

        // Wait for response (9 bytes expected)
        // Slave(1) + Func(1) + Count(1) + Data(4) + CRC(2) = 9 bytes
        size_t expected_len = 9;
        uint8_t resp[32] = {0};
        size_t n = 0;
        
        // Simple polling for 9 bytes with timeout (2026-08-06: 100Hz 适配, 1ms轮询, 25ms超时)
        auto start_time = node->now();
        while ((node->now() - start_time).seconds() < 0.025) {
            if (ser.available() >= expected_len) {
                n = ser.read(resp, expected_len);
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        if (n >= expected_len) {
            if (debug) {
                RCLCPP_DEBUG(node->get_logger(), "RX: %s", to_hex_string(resp, n).size() > 0 ? to_hex_string(resp, n).c_str() : "TIMEOUT");
            }
            
            if (resp[0] == slave_id && resp[1] == 0x03 && resp[2] == 0x04) {
                uint16_t resp_crc = modbus_crc(resp, n - 2);
                uint16_t received_crc = resp[n - 2] | (resp[n - 1] << 8);
                
                if (resp_crc == received_crc) {
                    // Force value is in the second register (Reg 0x0FA1)
                    // resp[3,4] is Reg 0x0FA0, resp[5,6] is Reg 0x0FA1
                    int16_t raw = (static_cast<int16_t>(resp[5]) << 8) | resp[6];
                    float force_n = static_cast<float>(raw * scale_factor);
                    
                    auto msg = std_msgs::msg::Float32();
                    msg.data = force_n;
                    force_pub->publish(msg);
                    
                    if (debug) {
                        RCLCPP_DEBUG(node->get_logger(), "Force: %.2f N (raw: %d)", force_n, raw);
                    }
                } else {
                    RCLCPP_WARN(node->get_logger(), "CRC Error: expected %04X, got %04X", resp_crc, received_crc);
                }
            } else {
                RCLCPP_WARN(node->get_logger(), "Unexpected response header: %02X %02X %02X", resp[0], resp[1], resp[2]);
            }
        } else {
            if (debug) {
                RCLCPP_DEBUG(node->get_logger(), "No response from sensor (received %zu bytes)", n);
            }
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    
    ser.close();
    rclcpp::shutdown();
    return 0;
}
