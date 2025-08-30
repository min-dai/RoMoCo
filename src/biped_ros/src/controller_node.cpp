#include "biped_ros/controller_node.hpp"
#include "biped_ros/conversions.hpp"
#include "screen_radio/screen_radio.hpp"

RosControllerNode::RosControllerNode(const std::string &config_folder,
                                     const std::string &log_path,
                                     std::shared_ptr<RobotBasePinocchio> robot)
    : Node("controller_node"),
      controller_(config_folder, log_path, robot),
      robot_(robot)
{
  using namespace std::chrono_literals;

  // Sub: proprioception from interface node
  proprio_sub_ = create_subscription<biped_msgs::msg::BipedProprioception>(
      "proprioception", 10,
      std::bind(&ControllerNode::proprioCallback, this, std::placeholders::_1));

  // Sub: radio commands
  radio_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "radio_topic", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
      {
        Eigen::VectorXd raw_radio = Eigen::Map<Eigen::VectorXd>(msg->data.data(), msg->data.size());
        desired_cmd_ = getScreenCommand(raw_radio);
        has_command_ = true;
      });

  // Pub: motor commands back to interface node
  motor_pub_ = create_publisher<biped_msgs::msg::BipedMotorCommands>(
      "controller_cmds", 10);

  // Timer: control loop
  timer_ = create_wall_timer(1ms, std::bind(&RosControllerNode::loop, this));
}

void ControllerNode::proprioCallback(
    const biped_msgs::msg::BipedProprioception::SharedPtr msg)
{
  loco_proprioception_ = fromRosMsg(*msg);
  has_proprio_ = true;
}

void ControllerNode::controlLoop()
{
  if (!has_proprio_)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Waiting for proprioception...");
    return;
  }

  Eigen::VectorXd q_loco = loco_proprioception_.q;
  Eigen::VectorXd dq_loco = loco_proprioception_.qdot;

  // Run state machine controller
  auto motor_cmd = controller_.UpdateControl(desired_cmd_, robot_, output_,
                                             torque_solver_, q_loco, dq_loco);

  // Publish ROS message
  motor_pub_->publish(toRosMsg(motor_cmd));
}