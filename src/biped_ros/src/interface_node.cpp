#include "biped_ros/interface_node.hpp"
#include "biped_ros/conversions.hpp"

RosInterfaceNode::RosInterfaceNode(std::shared_ptr<InterfaceBase> interface)
    : Node("ros_interface_node"), interface_(interface)
{
  using namespace std::chrono_literals;

  // Subscriber: controller motor commands
  ctrl_sub_ = create_subscription<biped_msgs::msg::BipedMotorCommands>(
      "controller_cmds", 1,
      [this](const biped_msgs::msg::BipedMotorCommands::SharedPtr msg)
      {
        ctrl_cmd_ = fromRosMsg(*msg); // convert to struct
        has_ctrl_cmd_ = true;
      });

  // Publisher: proprioception
  proprio_pub_ =
      create_publisher<biped_msgs::msg::BipedProprioception>("proprioception", 1);

  // Timer: control loop
  timer_ = create_wall_timer(0.5ms, std::bind(&RosInterfaceNode::Loop, this));

  init_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), // Small delay to ensure node is fully ready
      [this]()
      {
        Init();
        init_timer_->cancel(); // Cancel timer after first execution
      });

      std::cout << "ROS Interface Node created." << std::endl;
}

void RosInterfaceNode::Init()
{
  std::cout << "Initializing ROS Interface Node..." << std::endl;

  loco_proprioception_ = interface_->Update();

  proprio_pub_->publish(toRosMsg(loco_proprioception_));
}

void RosInterfaceNode::Loop()
{
  loco_proprioception_ = interface_->Update(ctrl_cmd_);

  if (interface_->IsInterfaceRunning())
  {
    // Publish proprioception
    proprio_pub_->publish(toRosMsg(loco_proprioception_));
  }

    interface_->SendPacket();
    
  
}