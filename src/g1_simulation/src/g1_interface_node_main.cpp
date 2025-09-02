#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "biped_ros/interface_node.hpp"


#include "g1_mujoco_sim.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  // Create the robot-specific interface
  std::string package_folder = ament_index_cpp::get_package_share_directory("g1_simulation");
    std::string config_folder = package_folder + "/config_18dof";


  std::shared_ptr<InterfaceBase> interface = std::make_shared<G1MujocoSim>(config_folder);

  // Wrap in ROS interface node
  auto node = std::make_shared<RosInterfaceNode>(interface);

  // Spin
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
