#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "biped_ros/interface_node.hpp"

#include "g1_hardware_interface.hpp"

#include "g1_model_leg.hpp"

int main(int argc, char *argv[])
{

   std::string networkInterface = "enp4s0";

   rclcpp::init(argc, argv);

   // Create the robot-specific interface
   std::string package_folder = ament_index_cpp::get_package_share_directory("g1_simulation");
   std::string config_folder = package_folder + "/config_18dof";
   std::string home = std::string(getenv("HOME"));

   // if timestamp env variable not set, use default
   std::string timestamp;
   if (getenv("LOG_FOLDER_TIMESTAMP") == NULL)
   {
      timestamp = "default";
   }
   else
   {
      timestamp = std::string(getenv("LOG_FOLDER_TIMESTAMP"));
   }

   std::string log_path = home + "/ROBOTLOG/G1/" + timestamp;

   std::string mujoco_config_file = config_folder + "/interface_config.yaml";
   YAMLParser yaml_parser(mujoco_config_file);
   std::string urdf_name = yaml_parser.get_string("urdf_name");

   std::string urdf_path = package_folder + "/model_files/" + urdf_name;
   std::vector<std::string> locked_encoder_names = yaml_parser.get_string_vector("locked_encoder_names");

   VectorXd locked_joints_q = yaml_parser.get_VectorXd("qdes_locked_joints");

   std::unique_ptr<RobotBasePinocchio> robot_ptr = std::make_unique<G1ModelLeg>(urdf_path, locked_encoder_names, locked_joints_q);

   std::shared_ptr<InterfaceBase> interface = std::make_shared<G1HardwareInterface>(networkInterface, config_folder, log_path, std::move(robot_ptr));

   // Wrap in ROS interface node
   auto node = std::make_shared<RosInterfaceNode>(config_folder, log_path, interface);

   // Spin
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}
