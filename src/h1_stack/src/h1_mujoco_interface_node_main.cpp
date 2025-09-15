#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "romoco_ros/ros_interface_node.hpp"

#include "h1_mujoco_interface.hpp"

#include "h1_model_leg.hpp"

#include "romoco_ros/ros_load_config.hpp"

using namespace romoco;
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  bool use_estimation = false;
  if (argc > 1) {
    use_estimation = std::string(argv[1]) == "true" || std::string(argv[1]) == "1";
  }

  RosLoadConfig ros_config("h1_stack", "config_18dof", "ROBOTLOG/H1");

  std::unique_ptr<romoco::robot::RobotBasePinocchio> robot_ptr = std::make_unique<romoco::robot::H1ModelLeg>(ros_config.config_folder);

  std::shared_ptr<InterfaceBase> interface;
  
  if (use_estimation)
      interface = std::make_shared<H1MujocoInterface>(ros_config.config_folder, ros_config.log_path, std::move(robot_ptr));
  else
      interface = std::make_shared<H1MujocoInterface>(ros_config.config_folder, ros_config.log_path);

  // Wrap in ROS interface node
  auto node = std::make_shared<RosInterfaceNode>(ros_config.config_folder, ros_config.log_path, std::move(interface));

  // Spin
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
