#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "romoco_ros/ros_interface_node.hpp"

#include "cassie_hardware_interface.hpp"

#include "cassie_model.hpp"

#include "romoco_ros/ros_load_config.hpp"

using namespace romoco;
int main(int argc, char *argv[])
{
   rclcpp::init(argc, argv);

   RosLoadConfig ros_config("cassie_stack", "config", "ROBOTLOG/CASSIE");

   std::unique_ptr<romoco::robot::RobotBasePinocchio> robot_ptr = std::make_unique<romoco::robot::CassieModel>(ros_config.config_folder);

   std::shared_ptr<InterfaceBase> interface = std::make_shared<CassieHardwareInterface>(ros_config.config_folder, ros_config.log_path, std::move(robot_ptr));

   // Wrap in ROS interface node
   auto node = std::make_shared<RosInterfaceNode>(ros_config.config_folder, ros_config.log_path, std::move(interface));

   // Spin
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}
