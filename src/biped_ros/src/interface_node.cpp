#include "biped_ros/interface_node.hpp"
#include "biped_ros/conversions.hpp"
#include "biped_utils/yaml_parser.hpp"
RosInterfaceNode::RosInterfaceNode(const std::string &config_folder,
                                   const std::string &log_path,
                                   std::shared_ptr<InterfaceBase> interface)
    : Node("ros_interface_node"), interface_(interface)
{
  using namespace std::chrono_literals;

  // Subscriber: controller motor commands
  ctrl_sub_ = create_subscription<biped_msgs::msg::BipedMotorCommands>(
      "controller_cmds", 1,
      [this](const biped_msgs::msg::BipedMotorCommands::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(ctrl_mutex_);
        loco_ctrl_cmd_ = fromRosMsg(*msg); // convert to struct
        std::cout << "Received controller commands." << std::endl;
        std::cout << loco_ctrl_cmd_ << std::endl;
      });

  // Publisher: proprioception
  proprio_pub_ =
      create_publisher<biped_msgs::msg::BipedProprioception>("proprioception", 1);

  // Timer: control loop
  YAMLParser parser(config_folder + "/interface_config.yaml");
  dt_ = parser.get_double("dt");
  timer_ = create_wall_timer(std::chrono::duration<double>(dt_), std::bind(&RosInterfaceNode::Loop, this));

  std::cout << "ROS Interface Node created." << std::endl;

  loco_ctrl_cmd_.ZeroAll(interface_->loco_motor_dof());
}



void RosInterfaceNode::Loop()
{

  BipedMotorCommands loco_ctrl_cmd;
  {
    std::lock_guard<std::mutex> lock(ctrl_mutex_);
    loco_ctrl_cmd = loco_ctrl_cmd_;
  }
  std::cout << "Interface Loop Running" << std::endl;
  std::cout << loco_ctrl_cmd << std::endl;
  loco_proprioception_ = interface_->Update(loco_ctrl_cmd);

  if (interface_->IsInterfaceRunning())
  {
    // Publish proprioception
    ros_proprio_msg_ = toRosMsg(loco_proprioception_); // Pre-allocate message
    proprio_pub_->publish(ros_proprio_msg_);
  }

  interface_->SendPacket();
}