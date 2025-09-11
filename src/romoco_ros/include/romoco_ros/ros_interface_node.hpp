#ifndef BIPED_ROS_INTERFACE_NODE_HPP
#define BIPED_ROS_INTERFACE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <romoco_core/interface_base.hpp>
#include "romoco_msgs/msg/biped_motor_commands.hpp"
#include "romoco_msgs/msg/biped_proprioception.hpp"
#include <atomic>

namespace romoco
{
  /**
   * @class RosInterfaceNode
   * @ingroup group_interface
   * @brief A ROS2 node for interfacing with a biped robot using romoco interface.
   * @details This class implements a ROS2 node that integrates with the Romoco framework to
   * interface with a biped robot. It subscribes to motor command data, processes it through
   * an interface, and publishes proprioception data.
   * @param config_folder The folder containing configuration files for the interface.
   * @param log_path The path to save log files.
   * @param interface A shared pointer to the robot interface (InterfaceBase), which handles communication with the robot hardware or simulator.
   */
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
    rclcpp::Subscription<romoco_msgs::msg::BipedMotorCommands>::SharedPtr ctrl_sub_;
    rclcpp::Publisher<romoco_msgs::msg::BipedProprioception>::SharedPtr proprio_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    BipedMotorCommands loco_ctrl_cmd_;
    std::atomic<bool> has_ctrl_cmd_{false};
    BipedProprioception loco_proprioception_;
  };
} // namespace romoco
#endif // BIPED_ROS_INTERFACE_NODE_HPP