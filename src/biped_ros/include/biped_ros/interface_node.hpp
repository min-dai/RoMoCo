#ifndef BIPED_ROS_INTERFACE_NODE_HPP
#define BIPED_ROS_INTERFACE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <biped_core/interface_base.hpp>
#include "biped_msgs/msg/biped_motor_commands.hpp"
#include "biped_msgs/msg/biped_proprioception.hpp"


class RosInterfaceNode : public rclcpp::Node {
 public:
  explicit RosInterfaceNode(std::shared_ptr<InterfaceBase> interface);

 private:

  void Init();
  void Loop();

  std::shared_ptr<InterfaceBase> interface_;
  rclcpp::Subscription<biped_msgs::msg::BipedMotorCommands>::SharedPtr ctrl_sub_;
  rclcpp::Publisher<biped_msgs::msg::BipedProprioception>::SharedPtr proprio_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::TimerBase::SharedPtr init_timer_;


  BipedMotorCommands ctrl_cmd_;
  bool has_ctrl_cmd_{false};

  BipedProprioception loco_proprioception_;


};

#endif  // BIPED_ROS_INTERFACE_NODE_HPP