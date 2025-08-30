#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "biped_core/ros_interface_node.hpp"

// Choose which robot interface to use
#include "biped_core/g1_hardware_interface.hpp"
// #include "biped_core/g1_simulation_interface.hpp"
// #include "biped_core/cassie_hardware_interface.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  // Create the robot-specific interface
  auto interface = std::make_shared<G1HardwareInterface>();

  // Wrap in ROS interface node
  auto node = std::make_shared<RosInterfaceNode>(interface);

  // Spin
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
