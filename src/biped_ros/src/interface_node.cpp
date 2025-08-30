#include "biped_ros/interface_node.hpp"
#include "biped_ros/conversions.hpp"

RosInterfaceNode::RosInterfaceNode(std::shared_ptr<InterfaceBase> interface)
      : Node("ros_interface_node"), interface_(interface) {
    using namespace std::chrono_literals;

    // Subscriber: controller motor commands
    ctrl_sub_ = create_subscription<biped_msgs::msg::BipedMotorCommands>(
        "controller_cmds", 10,
        [this](const biped_msgs::msg::BipedMotorCommands::SharedPtr msg) {
          ctrl_cmd_ = fromRosMsg(*msg);  // convert to struct
          has_ctrl_cmd_ = true;
        });

    // Publisher: proprioception
    proprio_pub_ =
        create_publisher<biped_msgs::msg::BipedProprioception>("proprioception", 10);

    // Timer: control loop
    timer_ = create_wall_timer(1ms, std::bind(&RosInterfaceNode::loop, this));
  }

void RosInterfaceNode::loop() {

   if (has_ctrl_cmd_) {
      loco_proprioception_ = interface_->Update(ctrl_cmd_);
   }

    // Publish proprioception
    proprio_pub_->publish(toRosMsg(loco_proprioception_));
  }