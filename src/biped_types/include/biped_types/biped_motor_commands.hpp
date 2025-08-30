#ifndef BIPED_MOTOR_COMMANDS_HPP
#define BIPED_MOTOR_COMMANDS_HPP

#include <Eigen/Dense>

struct BipedMotorCommands
{

   Eigen::VectorXd joint_positions;
   Eigen::VectorXd joint_velocities;
   Eigen::VectorXd joint_kp;
   Eigen::VectorXd joint_kd;
   Eigen::VectorXd joint_torques;
   std::vector<int> joint_indices;

   BipedMotorCommands() = default;

   explicit BipedMotorCommands(int dof)
       : joint_positions(Eigen::VectorXd::Zero(dof)),
         joint_velocities(Eigen::VectorXd::Zero(dof)),
         joint_kp(Eigen::VectorXd::Zero(dof)),
         joint_kd(Eigen::VectorXd::Zero(dof)),
         joint_torques(Eigen::VectorXd::Zero(dof)) {};

   void ResizeAll(int dof)
   {
      joint_positions.resize(dof);
      joint_velocities.resize(dof);
      joint_kp.resize(dof);
      joint_kd.resize(dof);
      joint_torques.resize(dof);
   }

   void ZeroAll()
   {
      joint_positions.setZero();
      joint_velocities.setZero();
      joint_kp.setZero();
      joint_kd.setZero();
      joint_torques.setZero();
   }

   void ZeroAll(int dof)
   {
      joint_positions = Eigen::VectorXd::Zero(dof);
      joint_velocities = Eigen::VectorXd::Zero(dof);
      joint_kp = Eigen::VectorXd::Zero(dof);
      joint_kd = Eigen::VectorXd::Zero(dof);
      joint_torques = Eigen::VectorXd::Zero(dof);
   }

   void ResizeTorque(int dof)
   {
      joint_torques.resize(dof);
   }

   BipedMotorCommands ConcatenateWithIndices(const BipedMotorCommands &other, int length) const
   {
       BipedMotorCommands result;
       result.ZeroAll(length);
       for (int i : joint_indices)
       {
           result.joint_positions[i] = joint_positions[i];
           result.joint_velocities[i] = joint_velocities[i];
           result.joint_kp[i] = joint_kp[i];
           result.joint_kd[i] = joint_kd[i];
           result.joint_torques[i] = joint_torques[i];
       }
       for (int i : other.joint_indices)
       {
           result.joint_positions[i] = other.joint_positions[i];
           result.joint_velocities[i] = other.joint_velocities[i];
           result.joint_kp[i] = other.joint_kp[i];
           result.joint_kd[i] = other.joint_kd[i];
           result.joint_torques[i] = other.joint_torques[i];
       }
       return result;
   }

   void UpdatePartialWithIndices(const BipedMotorCommands &other)
   {
       for (int i : joint_indices)
       {
           joint_positions[i] = other.joint_positions[i];
           joint_velocities[i] = other.joint_velocities[i];
           joint_kp[i] = other.joint_kp[i];
           joint_kd[i] = other.joint_kd[i];
           joint_torques[i] = other.joint_torques[i];
       }
   }

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