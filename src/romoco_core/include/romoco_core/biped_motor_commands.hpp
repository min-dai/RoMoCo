#ifndef BIPED_MOTOR_COMMANDS_HPP
#define BIPED_MOTOR_COMMANDS_HPP

#include <Eigen/Dense>
#include <iostream>

namespace romoco
{
/**
 * @struct BipedMotorCommands
 * @brief Structure representing motor commands for a bipedal robot.
 * @ingroup group_controller
 * @details This structure contains vectors for joint positions, velocities,
 * stiffness (kp), damping (kd), feedforward torques, and final torques after
 * feedback. It also includes methods for resizing, zeroing, and updating these
 * vectors, as well as computing the full torque based on current joint states.
 */
struct BipedMotorCommands
{

   Eigen::VectorXd joint_positions;
   Eigen::VectorXd joint_velocities;
   Eigen::VectorXd joint_kp;
   Eigen::VectorXd joint_kd;
   Eigen::VectorXd joint_torques_ff;

   Eigen::VectorXd joint_torques; // final torques after adding feedback terms for logging
   std::vector<int> joint_indices;

   BipedMotorCommands() = default;

   explicit BipedMotorCommands(int dof)
       : joint_positions(Eigen::VectorXd::Zero(dof)),
         joint_velocities(Eigen::VectorXd::Zero(dof)),
         joint_kp(Eigen::VectorXd::Zero(dof)),
         joint_kd(Eigen::VectorXd::Zero(dof)),
         joint_torques_ff(Eigen::VectorXd::Zero(dof)),
         joint_torques(Eigen::VectorXd::Zero(dof)) {};

   void ResizeAll(int dof)
   {
      joint_positions.resize(dof);
      joint_velocities.resize(dof);
      joint_kp.resize(dof);
      joint_kd.resize(dof);
      joint_torques_ff.resize(dof);
      joint_torques.resize(dof);
   }

   void ZeroAll()
   {
      joint_positions.setZero();
      joint_velocities.setZero();
      joint_kp.setZero();
      joint_kd.setZero();
      joint_torques_ff.setZero();
      joint_torques.setZero();
   }

   void ZeroAll(int dof)
   {
      joint_positions = Eigen::VectorXd::Zero(dof);
      joint_velocities = Eigen::VectorXd::Zero(dof);
      joint_kp = Eigen::VectorXd::Zero(dof);
      joint_kd = Eigen::VectorXd::Zero(dof);
      joint_torques_ff = Eigen::VectorXd::Zero(dof);
      joint_torques = Eigen::VectorXd::Zero(dof);
   }

   void ResizeTorque(int dof)
   {
      joint_torques_ff.resize(dof);
      joint_torques.resize(dof);
   }

   BipedMotorCommands GetIndex(const std::vector<int> &indices) const
   {
      BipedMotorCommands result;
      result.joint_positions = joint_positions(indices);
      result.joint_velocities = joint_velocities(indices);
      result.joint_kp = joint_kp(indices);
      result.joint_kd = joint_kd(indices);
      result.joint_torques_ff = joint_torques_ff(indices);
      return result;
   }

   void Update(const BipedMotorCommands &other)
   {
      joint_positions = other.joint_positions;
      joint_velocities = other.joint_velocities;
      joint_kp = other.joint_kp;
      joint_kd = other.joint_kd;
      joint_torques_ff = other.joint_torques_ff;
      joint_torques = other.joint_torques;
   }

   void UpdatePartialWithIndices(const BipedMotorCommands &other, const std::vector<int> &indices)
   {
      if (indices.size() > 0){
         for (int i = 0; i < indices.size(); i++)
       {
            int target_idx = indices[i];
            
            // Add bounds checking
            if (target_idx < joint_positions.size()) {
                joint_positions(target_idx) = other.joint_positions(i);
                joint_velocities(target_idx) = other.joint_velocities(i);
                joint_kp(target_idx) = other.joint_kp(i);
                joint_kd(target_idx) = other.joint_kd(i);
                joint_torques_ff(target_idx) = other.joint_torques_ff(i);
                if (other.joint_torques.size() == other.joint_positions.size()){
                  //since joint_torques is optional, only copy if it has the same size as other
                     joint_torques(target_idx) = other.joint_torques(i);
                }
            } else {
                std::cerr << "Error: Index " << target_idx << " is out of bounds!" << std::endl;
            }
       }
      }else{
         std::cerr << "Warning: No joint indices specified for partial update. Skipping update." << std::endl;
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
         << "Joint Torques FF: " << cmd.joint_torques_ff.transpose() << "\n"
         << "Joint Torques: " << cmd.joint_torques.transpose() << "\n";

      return os;
   }
};

} // namespace romoco

#endif // BIPED_MOTOR_COMMANDS_HPP