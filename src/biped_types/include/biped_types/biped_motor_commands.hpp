#ifndef BIPED_MOTOR_COMMANDS_HPP
#define BIPED_MOTOR_COMMANDS_HPP

#include <Eigen/Dense>

class BipedMotorCommands
{
public:
   Eigen::VectorXd joint_positions;
   Eigen::VectorXd joint_velocities;
   Eigen::VectorXd joint_kp;
   Eigen::VectorXd joint_kd;
   Eigen::VectorXd joint_torques;

   BipedMotorCommands() = default;

   explicit BipedMotorCommands(std::size_t dof)
       : joint_positions(Eigen::VectorXd::Zero(dof)),
         joint_velocities(Eigen::VectorXd::Zero(dof)),
         joint_kp(Eigen::VectorXd::Zero(dof)),
         joint_kd(Eigen::VectorXd::Zero(dof)),
         joint_torques(Eigen::VectorXd::Zero(dof)) {};
   // Export to std::vector (handy for IPC/ROS messages).
   static std::vector<double> ToStdVector(const Eigen::VectorXd &v)
   {
      return std::vector<double>(v.data(), v.data() + v.size());
   }

   // Import from std::vector (resizes Eigen vector).
   static void FromStdVector(const std::vector<double> &src, Eigen::VectorXd *dst)
   {
      if (dst == nullptr)
         return;
      dst->resize(static_cast<int>(src.size()));
      if (!src.empty())
      {
         Eigen::Map<const Eigen::VectorXd> m(src.data(), static_cast<int>(src.size()));
         *dst = m;
      }
   }
};

#endif // BIPED_MOTOR_COMMANDS_HPP