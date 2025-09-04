#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "biped_ros/controller_node.hpp"
#include "g1_model_leg.hpp"

int main(int argc, char *argv[])
{
   rclcpp::init(argc, argv);


   std::string package_folder = ament_index_cpp::get_package_share_directory("g1_simulation");
   std::string config_folder = package_folder + "/config_18dof";
   // TODO: get time and save it as string so log folder include time
   std::string home = std::string(getenv("HOME"));
   std::string timestamp = std::string(getenv("LOG_FOLDER_TIMESTAMP"));
   std::string log_path = home + "/ROBOTLOG/G1/" + timestamp;

   std::string mujoco_config_file = config_folder + "/interface_config.yaml";
   YAMLParser yaml_parser(mujoco_config_file);
   std::string urdf_name = yaml_parser.get_string("urdf_name");

   std::string urdf_path = package_folder + "/model_files/" + urdf_name;
   std::vector<std::string> locked_encoder_names = yaml_parser.get_string_vector("locked_encoder_names");
   int n_locked_joints = locked_encoder_names.size();
   VectorXd locked_joints_q = yaml_parser.get_VectorXd("qdes_locked_joints");

   std::shared_ptr<RobotBasePinocchio> robot_ptr = std::make_shared<G1ModelLeg>(urdf_path, locked_encoder_names, locked_joints_q);

   // Wrap in ROS controller node
   auto node = std::make_shared<RosControllerNode>(config_folder, log_path, robot_ptr);

   // Spin
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}
