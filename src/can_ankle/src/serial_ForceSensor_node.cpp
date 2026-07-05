// Force sensor node — D056 display + DYMH113 sensor via Modbus RTU
// Protocol: Modbus RTU, 19200 baud, slave 1, register 0x0FA1 (signed int16)

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <serial/serial.h>
#include <cstdint>
#include <cstring>

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

void signalHandler(int signum) {
    std::cout << "close" << std::endl;
    usleep(4000);
    rclcpp::shutdown();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("serial_ForceSensor_node");
    auto force_pub = node->create_publisher<std_msgs::msg::Float32>("Force", 10);
    rclcpp::WallRate loop_rate(std::chrono::milliseconds(5));
    signal(SIGINT, signalHandler);

    try {
        ser.setPort("/dev/ttyCH341USB1");
        ser.setBaudrate(19200);
        ser.setBytesize(serial::eightbits);
        ser.setParity(serial::parity_none);
        ser.setStopbits(serial::stopbits_one);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(200);
        ser.setTimeout(timeout);
        ser.open();
    }
    catch (serial::IOException& e) {
        RCLCPP_ERROR_STREAM(node->get_logger(), "无法打开串口: " << e.what());
        return -1;
    }

    if (!ser.isOpen()) { RCLCPP_ERROR_STREAM(node->get_logger(), "串口未打开"); return -1; }
    RCLCPP_INFO_STREAM(node->get_logger(), "Force sensor serial port initialized (Modbus RTU, 19200)");

    while (rclcpp::ok()) {
        uint8_t req[8] = {0x01, 0x03, 0x0F, 0xA0, 0x00, 0x02, 0x00, 0x00};
        uint16_t crc = modbus_crc(req, 6);
        req[6] = crc & 0xFF;
        req[7] = crc >> 8;
        ser.flushInput();
        ser.write(req, 8);
        usleep(60000);
        uint8_t resp[32] = {0};
        size_t n = ser.available();
        if (n >= 7) {
            n = ser.read(resp, n > 31 ? 31 : n);
            if (n >= 7 && resp[0] == 0x01 && resp[1] == 0x03) {
                uint16_t resp_crc = modbus_crc(resp, n - 2);
                uint16_t expected = resp[n-2] | (resp[n-1] << 8);
                if (resp_crc == expected) {
                    int16_t raw = (resp[5] << 8) | resp[6];
                    float force_n = raw * 0.01f;
                    auto msg = std_msgs::msg::Float32();
                    msg.data = force_n;
                    force_pub->publish(msg);
                }
            }
        }
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    ser.close();
    rclcpp::shutdown();
    return 0;
}
