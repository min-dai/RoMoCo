#include "biped_ros/interface_node.hpp"
#include "biped_ros/conversions.hpp"
#include "biped_utils/yaml_parser.hpp"
RosInterfaceNode::RosInterfaceNode(const std::string &config_folder,
                                   const std::string &log_path,
                                   std::shared_ptr<InterfaceBase> interface)
    : Node("ros_interface_node"), interface_(interface)
{
  if (!interface_)
  {
    throw std::invalid_argument("RosInterfaceNode: interface is null");
  }

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  // Subscriber: controller motor commands
  ctrl_sub_ = create_subscription<biped_msgs::msg::BipedMotorCommands>(
      "controller_cmds", qos,
      [this](const biped_msgs::msg::BipedMotorCommands::SharedPtr msg)
      {
        {
          std::lock_guard<std::mutex> lock(ctrl_mutex_);
          loco_ctrl_cmd_ = fromRosMsg(*msg); // convert to struct
        }
        has_ctrl_cmd_ = true;
      });

  // Publisher: proprioception
  proprio_pub_ = create_publisher<biped_msgs::msg::BipedProprioception>("proprioception", qos);

  // Timer: control loop
  YAMLParser parser(config_folder + "/interface_config.yaml");
  dt_ = parser.get_double("dt");
  timer_ = create_wall_timer(std::chrono::duration<double>(dt_), std::bind(&RosInterfaceNode::Loop, this));

  std::cout << "ROS Interface Node created." << std::endl;

  loco_ctrl_cmd_.ZeroAll(interface_->loco_motor_dof());
}

RosInterfaceNode::~RosInterfaceNode()
{
  timer_.reset();
  ctrl_sub_.reset();
  proprio_pub_.reset();
}
void RosInterfaceNode::Loop()
{
  if (has_ctrl_cmd_)
  {
    BipedMotorCommands loco_ctrl_cmd;
    {
      std::lock_guard<std::mutex> lock(ctrl_mutex_);
      loco_ctrl_cmd = loco_ctrl_cmd_;
    }
    interface_->ProcessMotorCommands(loco_ctrl_cmd);
  }

  loco_proprioception_ = interface_->ReadAndEstimate();
  

  if (interface_->IsInterfaceRunning())
  {
    // Publish proprioception
    auto ros_proprio_msg = toRosMsg(loco_proprioception_); // Pre-allocate message
    proprio_pub_->publish(ros_proprio_msg);
  }
  interface_->SendPacket();

}