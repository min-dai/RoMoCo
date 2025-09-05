#ifndef BIPED_ROS_INTERFACE_NODE_HPP
#define BIPED_ROS_INTERFACE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <biped_core/interface_base.hpp>
#include "biped_msgs/msg/biped_motor_commands.hpp"
#include "biped_msgs/msg/biped_proprioception.hpp"

class RosInterfaceNode : public rclcpp::Node
{
public:
  explicit RosInterfaceNode(const std::string &config_folder,
                            const std::string &log_path,
                            std::shared_ptr<InterfaceBase> interface);
  ~RosInterfaceNode() override;

private:
  void Loop();

  std::shared_ptr<InterfaceBase> interface_;
  double dt_ = 0.0005; // default, will be overwritten by YAML

  std::mutex ctrl_mutex_;
  rclcpp::Subscription<biped_msgs::msg::BipedMotorCommands>::SharedPtr ctrl_sub_;
  rclcpp::Publisher<biped_msgs::msg::BipedProprioception>::SharedPtr proprio_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  BipedMotorCommands loco_ctrl_cmd_;
  BipedProprioception loco_proprioception_;

};

#endif // BIPED_ROS_INTERFACE_NODE_HPP