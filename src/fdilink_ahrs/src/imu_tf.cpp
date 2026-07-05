#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <string>

class ImuTfBroadcaster : public rclcpp::Node
{
public:
  ImuTfBroadcaster() : Node("imu_data_to_tf")
  {
    this->declare_parameter("imu_topic", std::string("/imu"));
    this->declare_parameter("world_frame_id", std::string("/world"));
    this->declare_parameter("imu_frame_id", std::string("/gyro_link"));
    this->declare_parameter("position_x", 0);
    this->declare_parameter("position_y", 0);
    this->declare_parameter("position_z", 0);

    imu_topic_ = this->get_parameter("imu_topic").as_string();
    world_frame_id_ = this->get_parameter("world_frame_id").as_string();
    imu_frame_id_ = this->get_parameter("imu_frame_id").as_string();
    position_x_ = this->get_parameter("position_x").as_int();
    position_y_ = this->get_parameter("position_y").as_int();
    position_z_ = this->get_parameter("position_z").as_int();

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 10,
        std::bind(&ImuTfBroadcaster::imuCallback, this, std::placeholders::_1));
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_data)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = world_frame_id_;
    transform.child_frame_id = imu_frame_id_;
    transform.transform.translation.x = position_x_;
    transform.transform.translation.y = position_y_;
    transform.transform.translation.z = position_z_;
    transform.transform.rotation.x = imu_data->orientation.x;
    transform.transform.rotation.y = imu_data->orientation.y;
    transform.transform.rotation.z = imu_data->orientation.z;
    transform.transform.rotation.w = imu_data->orientation.w;
    tf_broadcaster_->sendTransform(transform);
  }

  std::string imu_topic_, world_frame_id_, imu_frame_id_;
  int position_x_, position_y_, position_z_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuTfBroadcaster>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
