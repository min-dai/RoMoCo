#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "biped_ros/ros_interface_node.hpp"

#include "g1_hardware_interface.hpp"

#include "g1_model_leg.hpp"

#include "biped_ros/ros_load_config.hpp"
int main(int argc, char *argv[])
{
   rclcpp::init(argc, argv);

   std::string networkInterface = "enp4s0";

   RosLoadConfig ros_config("g1_simulation", "config_18dof_hardware", "ROBOTLOG/G1");


   std::unique_ptr<RobotBasePinocchio> robot_ptr = std::make_unique<G1ModelLeg>(ros_config.config_folder);

   std::shared_ptr<InterfaceBase> interface = std::make_shared<G1HardwareInterface>(networkInterface, ros_config.config_folder, ros_config.log_path, std::move(robot_ptr));

   // Wrap in ROS interface node
   auto node = std::make_shared<RosInterfaceNode>(ros_config.config_folder, ros_config.log_path, std::move(interface));

   // Spin
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}
