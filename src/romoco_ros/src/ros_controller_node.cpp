#include "romoco_ros/ros_controller_node.hpp"
#include "romoco_ros/conversions.hpp"
#include "romoco_utils/yaml_parser.hpp"


namespace romoco
{

RosControllerNode::RosControllerNode(const std::string &config_folder,
                                     const std::string &log_path,
                                     std::shared_ptr<RobotBasePinocchio> robot,
                                     const std::string &raw_radio_topic_name,
                                     std::function<DesiredCommand(const Eigen::VectorXd &)> ConvertRadioToCommand)
    : Node("ros_controller_node"),
      controller_(std::make_unique<BasicControllerStateMachine>(config_folder, log_path, robot)),
      robot_(robot),
      get_command_func_(std::move(ConvertRadioToCommand))
{
  using namespace std::chrono_literals;

  // Sub: proprioception from interface node
  proprio_sub_ = create_subscription<romoco_msgs::msg::BipedProprioception>(
      "proprioception", 1,
      std::bind(&RosControllerNode::ProprioCallback, this, std::placeholders::_1));

  // Sub: radio commands
  radio_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      raw_radio_topic_name, 1,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
      {
        Eigen::VectorXd raw_radio = Eigen::Map<Eigen::VectorXd>(msg->data.data(), msg->data.size());

        // Thread-safe command update
        {
          std::lock_guard<std::mutex> lock(command_mutex_);
          desired_cmd_ = get_command_func_(raw_radio);
        }
        has_command_ = true;
      });

  // Pub: motor commands back to interface node
  motor_pub_ = create_publisher<romoco_msgs::msg::BipedMotorCommands>(
      "controller_cmds", 1);

  YAMLParser parser(config_folder + "/interface_config.yaml");
  dt_ = parser.get_double("dt");
  // Timer: control loop
  timer_ = create_wall_timer(std::chrono::duration<double>(dt_), std::bind(&RosControllerNode::Loop, this));
}

void RosControllerNode::ProprioCallback(
    const romoco_msgs::msg::BipedProprioception::SharedPtr msg)
{
  // Thread-safe proprioception update
  {
    std::lock_guard<std::mutex> lock(proprio_mutex_);
    loco_proprioception_ = fromRosMsg(*msg);
  }

  // Set flags atomically
  has_proprio_ = true;
  has_new_proprio_ = true;
}

void RosControllerNode::Loop()
{
  // Check if we have proprioception data
  if (!has_proprio_)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Waiting for proprioception...");
    return;
  }

  // Only run control if we have new proprioception data
  if (!has_new_proprio_)
  {
    return; // Skip this iteration, no new data
  }

  // Reset the new data flag
  has_new_proprio_ = false;

  // Thread-safe copy of proprioception data
  BipedProprioception current_proprio;
  {
    std::lock_guard<std::mutex> lock(proprio_mutex_);
    current_proprio = loco_proprioception_;
  }

  // Thread-safe copy of command data
  DesiredCommand current_cmd;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    current_cmd = desired_cmd_;
  }

  Eigen::VectorXd q_loco = current_proprio.q;
  Eigen::VectorXd dq_loco = current_proprio.qdot;

  // Run state machine controller
  BipedMotorCommands motor_cmd = controller_->UpdateControl(current_cmd, robot_, output_,
                                                            torque_solver_, q_loco, dq_loco);

  // Publish ROS message
  if (motor_cmd.joint_positions.size() == 0)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Motor command is empty, skipping publish.");
    return;
  }
  motor_pub_->publish(toRosMsg(motor_cmd));
}

} // namespace romoco