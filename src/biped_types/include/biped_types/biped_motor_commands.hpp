#ifndef BIPED_MOTOR_COMMANDS_HPP
#define BIPED_MOTOR_COMMANDS_HPP

#include <Eigen/Dense>

struct BipedMotorCommands
{

   Eigen::VectorXd joint_positions;
   Eigen::VectorXd joint_velocities;
   Eigen::VectorXd joint_kp;
   Eigen::VectorXd joint_kd;
   Eigen::VectorXd joint_torques_ff;
   std::vector<int> joint_indices;

   BipedMotorCommands() = default;

   explicit BipedMotorCommands(int dof)
       : joint_positions(Eigen::VectorXd::Zero(dof)),
         joint_velocities(Eigen::VectorXd::Zero(dof)),
         joint_kp(Eigen::VectorXd::Zero(dof)),
         joint_kd(Eigen::VectorXd::Zero(dof)),
         joint_torques_ff(Eigen::VectorXd::Zero(dof)) {};

   void ResizeAll(int dof)
   {
      joint_positions.resize(dof);
      joint_velocities.resize(dof);
      joint_kp.resize(dof);
      joint_kd.resize(dof);
      joint_torques_ff.resize(dof);
   }

   void ZeroAll()
   {
      joint_positions.setZero();
      joint_velocities.setZero();
      joint_kp.setZero();
      joint_kd.setZero();
      joint_torques_ff.setZero();
   }

   void ZeroAll(int dof)
   {
      joint_positions = Eigen::VectorXd::Zero(dof);
      joint_velocities = Eigen::VectorXd::Zero(dof);
      joint_kp = Eigen::VectorXd::Zero(dof);
      joint_kd = Eigen::VectorXd::Zero(dof);
      joint_torques_ff = Eigen::VectorXd::Zero(dof);
   }

   void ResizeTorque(int dof)
   {
      joint_torques_ff.resize(dof);
   }

   void UpdatePartialWithIndices(const BipedMotorCommands &other)
   {
      if (other.joint_indices.size() > 0){
         for (int i : other.joint_indices)
       {
           joint_positions[i] = other.joint_positions[i];
           joint_velocities[i] = other.joint_velocities[i];
           joint_kp[i] = other.joint_kp[i];
           joint_kd[i] = other.joint_kd[i];
           joint_torques_ff[i] = other.joint_torques_ff[i];
       }
      }
       
   }
   Eigen::VectorXd SolveFullTorque(const Eigen::VectorXd &qa, const Eigen::VectorXd &qa_dot) const
   {
      Eigen::VectorXd full_torque = joint_torques_ff;

      // Compute the full torque using the joint positions and velocities
      full_torque -= joint_kp.cwiseProduct(qa - joint_positions);
      full_torque -= joint_kd.cwiseProduct(qa_dot - joint_velocities);

      return full_torque;
   }

   friend std::ostream &operator<<(std::ostream &os, const BipedMotorCommands &cmd)
   {
      os << "Joint Positions: " << cmd.joint_positions.transpose() << "\n"
         << "Joint Velocities: " << cmd.joint_velocities.transpose() << "\n"
         << "Joint KP: " << cmd.joint_kp.transpose() << "\n"
         << "Joint KD: " << cmd.joint_kd.transpose() << "\n"
         << "Joint Torques FF: " << cmd.joint_torques_ff.transpose();
      return os;
   }
};

#endif // BIPED_MOTOR_COMMANDS_HPP