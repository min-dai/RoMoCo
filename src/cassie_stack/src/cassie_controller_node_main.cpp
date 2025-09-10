#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "romoco_ros/ros_controller_node.hpp"
#include "cassie_model.hpp"

#include "romoco_ros/ros_load_config.hpp"

#include "romoco_screen_radio/screen_radio_conversion.hpp"
int main(int argc, char *argv[])
{
   rclcpp::init(argc, argv);

   bool use_sim_gains = false;
   if (argc > 1) {
     use_sim_gains = std::string(argv[1]) == "true" || std::string(argv[1]) == "1";
   }
   std::cout << "Using simulation gains: " << (use_sim_gains ? "true" : "false") << std::endl;
   std::string config_folder_name;
   if (use_sim_gains)
     config_folder_name = "config";
   else
     config_folder_name = "config_hardware";


   RosLoadConfig ros_config("cassie_stack", config_folder_name, "ROBOTLOG/Cassie");

   std::shared_ptr<RobotBasePinocchio> robot_ptr = std::make_shared<CassieModel>(ros_config.config_folder);

   // Wrap in ROS controller node
   auto node = std::make_shared<RosControllerNode>(ros_config.config_folder, ros_config.log_path, std::move(robot_ptr),"screen_radio_values",
      std::function<DesiredCommand(const Eigen::VectorXd&)>(ConvertScreenRadioToDesiredCommand));

   // Spin
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}
