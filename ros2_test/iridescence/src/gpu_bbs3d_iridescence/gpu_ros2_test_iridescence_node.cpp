#include <ros2_test.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ROS2Test>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}