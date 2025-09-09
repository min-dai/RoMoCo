#ifndef ROS_MSG_TO_BIPED_TYPES_CONVERSION
#define ROS_MSG_TO_BIPED_TYPES_CONVERSION

#include "romoco_msgs/msg/biped_motor_commands.hpp"
#include "romoco_msgs/msg/biped_proprioception.hpp"
#include "romoco_types/biped_motor_commands.hpp"
#include "romoco_types/biped_proprioception.hpp"



inline BipedProprioception fromRosMsg(const romoco_msgs::msg::BipedProprioception& msg) {
  BipedProprioception proprioception;
  proprioception.q = Eigen::Map<const Eigen::VectorXd>(msg.q.data(), msg.q.size());
  proprioception.qdot = Eigen::Map<const Eigen::VectorXd>(msg.qdot.data(), msg.qdot.size());
  return proprioception;
}

inline romoco_msgs::msg::BipedProprioception toRosMsg(const BipedProprioception& proprioception) {
  romoco_msgs::msg::BipedProprioception msg;
  // Resize and copy values
  msg.q.assign(proprioception.q.data(), proprioception.q.data() + proprioception.q.size());
  msg.qdot.assign(proprioception.qdot.data(), proprioception.qdot.data() + proprioception.qdot.size());
  return msg;
}

inline BipedMotorCommands fromRosMsg(const romoco_msgs::msg::BipedMotorCommands& msg) {
  BipedMotorCommands cmd;
  cmd.joint_positions = Eigen::Map<const Eigen::VectorXd>(msg.joint_positions.data(), msg.joint_positions.size());
  cmd.joint_velocities = Eigen::Map<const Eigen::VectorXd>(msg.joint_velocities.data(), msg.joint_velocities.size());
  cmd.joint_kp = Eigen::Map<const Eigen::VectorXd>(msg.joint_kp.data(), msg.joint_kp.size());
  cmd.joint_kd = Eigen::Map<const Eigen::VectorXd>(msg.joint_kd.data(), msg.joint_kd.size());
  cmd.joint_torques_ff = Eigen::Map<const Eigen::VectorXd>(msg.joint_torques_ff.data(), msg.joint_torques_ff.size());
  return cmd;
}

inline romoco_msgs::msg::BipedMotorCommands toRosMsg(const BipedMotorCommands& cmd) {
  romoco_msgs::msg::BipedMotorCommands msg;
  // Resize and copy values
  msg.joint_positions.assign(cmd.joint_positions.data(), cmd.joint_positions.data() + cmd.joint_positions.size());
  msg.joint_velocities.assign(cmd.joint_velocities.data(), cmd.joint_velocities.data() + cmd.joint_velocities.size());
  msg.joint_kp.assign(cmd.joint_kp.data(), cmd.joint_kp.data() + cmd.joint_kp.size());
  msg.joint_kd.assign(cmd.joint_kd.data(), cmd.joint_kd.data() + cmd.joint_kd.size());
  msg.joint_torques_ff.assign(cmd.joint_torques_ff.data(), cmd.joint_torques_ff.data() + cmd.joint_torques_ff.size());

  return msg;
}

#endif  // ROS_MSG_TO_BIPED_TYPES_CONVERSION