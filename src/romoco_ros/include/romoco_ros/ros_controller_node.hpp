#ifndef BIPED_ROS_CONTROLLER_NODE_HPP
#define BIPED_ROS_CONTROLLER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <atomic>
#include "romoco_state_machine/basic_controller_state_machine.hpp"
#include "romoco_msgs/msg/biped_motor_commands.hpp"
#include "romoco_msgs/msg/biped_proprioception.hpp"
#include "romoco_types/biped_proprioception.hpp"
#include "romoco_types/biped_motor_commands.hpp"
#include "romoco_types/biped_commands.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>

class RosControllerNode : public rclcpp::Node
{
public:
   explicit RosControllerNode(const std::string &config_folder,
                              const std::string &log_path,
                              std::shared_ptr<RobotBasePinocchio> robot,
                              const std::string &raw_radio_topic_name,
                              std::function<DesiredCommand(const Eigen::VectorXd&)> getCommand);

private:
   void ProprioCallback(const romoco_msgs::msg::BipedProprioception::SharedPtr msg);
   void Loop();

   std::unique_ptr<BasicControllerStateMachine> controller_;

   double dt_ = 0.0005; // default, will be overwritten by YAML

   rclcpp::Publisher<romoco_msgs::msg::BipedMotorCommands>::SharedPtr motor_pub_;
   rclcpp::Subscription<romoco_msgs::msg::BipedProprioception>::SharedPtr proprio_sub_;
   rclcpp::TimerBase::SharedPtr timer_;

   BipedMotorCommands ctrl_cmd_;

   // Thread-safe proprioception data
   std::mutex proprio_mutex_;
   std::atomic<bool> has_proprio_{false};
   std::atomic<bool> has_new_proprio_{false};
   BipedProprioception loco_proprioception_;

   // controller dependency
   std::shared_ptr<RobotBasePinocchio> robot_;
   std::shared_ptr<OutputBase> output_;
   std::unique_ptr<TorqueSolverBase> torque_solver_;

   // Thread-safe command data
   std::mutex command_mutex_;
   rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr radio_sub_;
   DesiredCommand desired_cmd_;
   std::atomic<bool> has_command_{false};
   std::function<DesiredCommand(const Eigen::VectorXd&)> get_command_func_;
};

#endif // BIPED_ROS_CONTROLLER_NODE_HPP
