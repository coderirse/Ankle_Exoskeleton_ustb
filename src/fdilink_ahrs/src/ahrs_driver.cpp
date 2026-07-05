// FDILink AHRS driver — 5A A5 binary protocol (firmware v1.6.8+)
// Frame: 5A A5 + uint16_LE_len(76) + 76B_payload + 2B_trailer = 82 bytes
// Baud: 115200, continuous output ~138 Hz

#include <ahrs_driver.h>
#include <Eigen/Eigen>
#include <cstring>

namespace FDILink {

ahrsBringup::ahrsBringup()
    : rclcpp::Node("ahrs_bringup"), frist_sn_(false), serial_timeout_(20)
{
    this->declare_parameter("debug", false);
    this->declare_parameter("device_type", 1);
    this->declare_parameter("imu_topic", std::string("/imu"));
    this->declare_parameter("imu_frame", std::string("gyro_link"));
    this->declare_parameter("mag_pose_2d_topic", std::string("/mag_pose_2d"));
    this->declare_parameter("Euler_angles_pub_", std::string("/euler_angles"));
    this->declare_parameter("Magnetic_pub_", std::string("/magnetic"));
    this->declare_parameter("gps_topic_", std::string("/gps/fix"));
    this->declare_parameter("twist_topic_", std::string("/system_speed"));
    this->declare_parameter("NED_odom_topic_", std::string("/NED_odometry"));
    this->declare_parameter("port", std::string("/dev/ttyACM0"));
    this->declare_parameter("baud", 115200);

    if_debug_ = this->get_parameter("debug").as_bool();
    device_type_ = this->get_parameter("device_type").as_int();
    imu_topic_ = this->get_parameter("imu_topic").as_string();
    imu_frame_id_ = this->get_parameter("imu_frame").as_string();
    mag_pose_2d_topic_ = this->get_parameter("mag_pose_2d_topic").as_string();
    Euler_angles_topic_ = this->get_parameter("Euler_angles_pub_").as_string();
    Magnetic_topic_ = this->get_parameter("Magnetic_pub_").as_string();
    gps_topic_ = this->get_parameter("gps_topic_").as_string();
    twist_topic_ = this->get_parameter("twist_topic_").as_string();
    NED_odom_topic_ = this->get_parameter("NED_odom_topic_").as_string();
    serial_port_ = this->get_parameter("port").as_string();
    serial_baud_ = this->get_parameter("baud").as_int();

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
    mag_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>(mag_pose_2d_topic_, 10);
    Euler_angles_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(Euler_angles_topic_, 10);
    Magnetic_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(Magnetic_topic_, 10);
    gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(gps_topic_, 10);
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(twist_topic_, 10);
    NED_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(NED_odom_topic_, 10);

    try {
        serial_.setPort(serial_port_);
        serial_.setBaudrate(serial_baud_);
        serial_.setFlowcontrol(serial::flowcontrol_none);
        serial_.setParity(serial::parity_none);
        serial_.setStopbits(serial::stopbits_one);
        serial_.setBytesize(serial::eightbits);
        serial::Timeout time_out = serial::Timeout::simpleTimeout(serial_timeout_);
        serial_.setTimeout(time_out);
        serial_.open();
    }
    catch (serial::IOException &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to open port " << serial_port_);
        exit(1);
    }

    if (serial_.isOpen()) {
        RCLCPP_INFO_STREAM(this->get_logger(),
            "5A A5 IMU driver ready — " << serial_port_ << " @ " << serial_baud_ << " baud");
    } else {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to open serial port");
        exit(1);
    }
    processLoop();
}

ahrsBringup::~ahrsBringup() {
    if (serial_.isOpen()) serial_.close();
}

static void quatToEuler(double qw, double qx, double qy, double qz,
                         double &roll, double &pitch, double &yaw)
{
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll = atan2(sinr_cosp, cosr_cosp);
    double sinp = 2.0 * (qw * qy - qz * qx);
    if (fabs(sinp) >= 1.0) pitch = copysign(M_PI / 2.0, sinp);
    else pitch = asin(sinp);
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = atan2(siny_cosp, cosy_cosp);
}

void ahrsBringup::processLoop()
{
    RCLCPP_INFO(this->get_logger(), "5A A5 driver running, waiting for frames...");
    uint8_t buffer[1024];
    size_t buf_pos = 0;

    while (rclcpp::ok()) {
        size_t avail = serial_.available();
        if (avail == 0) { usleep(1000); continue; }
        if (avail > sizeof(buffer) - buf_pos) avail = sizeof(buffer) - buf_pos;
        size_t n = serial_.read(buffer + buf_pos, avail);
        buf_pos += n;

        size_t search_pos = 0;
        while (search_pos + 82 <= buf_pos) {
            uint8_t *p = buffer + search_pos;
            uint8_t *end = buffer + buf_pos - 81;
            bool found = false;
            while (p <= end) {
                if (p[0] == 0x5A && p[1] == 0xA5) { found = true; break; }
                p++;
            }
            if (!found) {
                if (buf_pos > 81) { memmove(buffer, buffer + buf_pos - 81, 81); buf_pos = 81; }
                break;
            }
            size_t offset = p - buffer;
            if (offset + 82 > buf_pos) {
                if (offset > 0) { memmove(buffer, p, buf_pos - offset); buf_pos -= offset; }
                break;
            }
            uint8_t *frame = p;
            uint16_t length = frame[2] | (frame[3] << 8);
            if (length != 76) {
                search_pos = offset + 2;
                continue;
            }
            uint8_t *payload = frame + 4;
            float qw, qx, qy, qz;
            memcpy(&qw, payload + 18, 4);
            memcpy(&qx, payload + 22, 4);
            memcpy(&qy, payload + 26, 4);
            memcpy(&qz, payload + 30, 4);

            if (if_debug_) {
                static int fc = 0;
                if (++fc % 100 == 0)
                    RCLCPP_INFO(this->get_logger(), "Frame #%d: q=(%.4f, %.4f, %.4f, %.4f)", fc, qw, qx, qy, qz);
            }

            Eigen::Quaterniond q_ahrs(qw, qx, qy, qz);
            Eigen::Quaterniond q_r =
                Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
            Eigen::Quaterniond q_rr =
                Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitX());
            Eigen::Quaterniond q_out = q_r * q_ahrs * q_rr;

            auto imu_data = sensor_msgs::msg::Imu();
            imu_data.header.stamp = this->get_clock()->now();
            imu_data.header.frame_id = imu_frame_id_;

            if (device_type_ == 0) {
                imu_data.orientation.w = qw; imu_data.orientation.x = qx;
                imu_data.orientation.y = qy; imu_data.orientation.z = qz;
            } else {
                imu_data.orientation.w = q_out.w(); imu_data.orientation.x = q_out.x();
                imu_data.orientation.y = q_out.y(); imu_data.orientation.z = q_out.z();
            }

            imu_data.angular_velocity.x = 0.0; imu_data.angular_velocity.y = 0.0; imu_data.angular_velocity.z = 0.0;
            imu_data.linear_acceleration.x = 0.0; imu_data.linear_acceleration.y = 0.0; imu_data.linear_acceleration.z = 0.0;
            imu_data.orientation_covariance[0] = -1.0;
            imu_data.angular_velocity_covariance[0] = -1.0;
            imu_data.linear_acceleration_covariance[0] = -1.0;
            imu_pub_->publish(imu_data);

            double roll, pitch, yaw;
            quatToEuler(q_out.w(), q_out.x(), q_out.y(), q_out.z(), roll, pitch, yaw);
            auto euler_msg = geometry_msgs::msg::Vector3();
            euler_msg.x = roll; euler_msg.y = pitch; euler_msg.z = yaw;
            Euler_angles_pub_->publish(euler_msg);

            auto pose_msg = geometry_msgs::msg::Pose2D();
            pose_msg.theta = yaw;
            mag_pose_pub_->publish(pose_msg);

            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = 0.0; twist_msg.linear.y = 0.0; twist_msg.linear.z = 0.0;
            twist_pub_->publish(twist_msg);

            search_pos = offset + 82;
        }
        if (search_pos >= buf_pos) buf_pos = 0;
        else if (search_pos > 0) { memmove(buffer, buffer + search_pos, buf_pos - search_pos); buf_pos -= search_pos; }
    }
}

} // namespace FDILink

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FDILink::ahrsBringup>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
