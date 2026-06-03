#include "joint_controller.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node_ptr = std::make_shared<JointControllerNode>();
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node_ptr);

  executor->spin();
  rclcpp::shutdown();
  return 0;
}