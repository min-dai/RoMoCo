#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "romoco_ros/ros_controller_node.hpp"
#include "cassie_model.hpp"

#include "romoco_screen_radio/screen_radio.hpp"
int main(int argc, char *argv[])
{
   rclcpp::init(argc, argv);

   std::string package_folder = ament_index_cpp::get_package_share_directory("cassie_stack");
   std::string config_folder = package_folder + "/config";

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
   std::string log_path = home + "/ROBOTLOG/Cassie/" + timestamp;

   std::string mujoco_config_file = config_folder + "/interface_config.yaml";
   YAMLParser yaml_parser(mujoco_config_file);
   std::string urdf_name = yaml_parser.get_string("urdf_name");

   std::string urdf_path = package_folder + "/model_files/" + urdf_name;
   std::vector<std::string> locked_encoder_names = yaml_parser.get_string_vector("locked_encoder_names");


   std::shared_ptr<RobotBasePinocchio> robot_ptr = std::make_shared<CassieModel>(urdf_path, locked_encoder_names);



   // Wrap in ROS controller node
   auto node = std::make_shared<RosControllerNode>(config_folder, log_path, robot_ptr,"screen_radio_values", 
      std::function<DesiredCommand(const Eigen::VectorXd&)>(getScreenCommand));

   // Spin
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}
