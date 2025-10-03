#include <memory>
#include <rclcpp/rclcpp.hpp>

// TODO: 创建一个 publisher, 调用你实现的 receiver，将受到的 imu 数据以 sensor_msgs/msg/imu 格式发布到一个 topic 中

class YourNode : public rclcpp::Node {
public:
  YourNode() : rclcpp::Node("node_name") {}
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<YourNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}