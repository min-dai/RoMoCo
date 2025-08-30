#ifndef BIPED_ROS_CONTROLLER_NODE_HPP
#define BIPED_ROS_CONTROLLER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <biped_state_machine/basic_controller_state_machine.hpp>
#include "biped_msgs/msg/biped_motor_commands.hpp"
#include "biped_msgs/msg/biped_proprioception.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>

class RosControllerNode : public rclcpp::Node
{
public:
   explicit RosControllerNode(const std::string &config_folder,
                              const std::string &log_path,
                              std::shared_ptr<RobotBasePinocchio> robot);

private:
   void proprioCallback(const biped_msgs::msg::BipedProprioception::SharedPtr msg);
   void loop();

   std::shared_ptr<BasicControllerStateMachine> controller_;

   rclcpp::Publisher<biped_msgs::msg::BipedMotorCommands>::SharedPtr ctrl_pub_;
   rclcpp::Subscription<biped_msgs::msg::BipedProprioception>::SharedPtr proprio_sub_;
   rclcpp::TimerBase::SharedPtr timer_;

   BipedMotorCommands ctrl_cmd_;

   bool has_proprio_{false};
   BipedProprioception loco_proprioception_;

   // controller dependency
   std::shared_ptr<RobotBasePinocchio> robot_;
   std::shared_ptr<OutputBase> output_;
   std::unique_ptr<TorqueSolverBase> torque_solver_;

   rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr radio_sub_;
   DesiredCommand desired_cmd_;
   bool has_command_{false};
};

#endif // BIPED_ROS_CONTROLLER_NODE_HPP