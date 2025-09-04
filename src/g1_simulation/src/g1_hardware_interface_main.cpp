#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "biped_ros/interface_node.hpp"

#include "g1_hardware_interface.hpp"

#include "g1_model_leg.hpp"

#include "biped_utils/log_utils.hpp"
int main(int argc, char *argv[])
{

   if (argc < 2)
   {
      std::cout << "Usage: g1_ankle_swing_example network_interface" << std::endl;
      exit(0);
   }
   std::string networkInterface = argv[1];

   rclcpp::init(argc, argv);

   // Create the robot-specific interface
   std::string package_folder = ament_index_cpp::get_package_share_directory("g1_simulation");
   std::string config_folder = package_folder + "/config_18dof";
   std::string home = std::string(getenv("HOME"));
   std::string log_path = home + "/ROBOTLOG/G1/" + getCurrentTimeString();

   std::string mujoco_config_file = config_folder + "/interface_config.yaml";
   YAMLParser yaml_parser(mujoco_config_file);
   std::string urdf_name = yaml_parser.get_string("urdf_name");

   std::string urdf_path = package_folder + "/model_files/" + urdf_name;
   std::vector<std::string> locked_encoder_names = yaml_parser.get_string_vector("locked_encoder_names");
   int n_locked_joints = locked_encoder_names.size();
   VectorXd locked_joints_q = yaml_parser.get_VectorXd("qdes_locked_joints");

   std::unique_ptr<RobotBasePinocchio> robot_ptr = std::make_unique<G1ModelLeg>(urdf_path, locked_encoder_names, locked_joints_q);

   std::shared_ptr<InterfaceBase> interface = std::make_shared<G1HardwareInterface>(networkInterface, config_folder, log_path, std::move(robot_ptr));

   // Wrap in ROS interface node
   auto node = std::make_shared<RosInterfaceNode>(interface);

   // Spin
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}
