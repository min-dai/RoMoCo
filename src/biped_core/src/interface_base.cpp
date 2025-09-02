#include "biped_core/interface_base.hpp"

void InterfaceBase::InitMotorCommands()
{
   if (pd_motor_indices_.size() + loco_motor_indices_.size() != total_motor_dof_)
   {
      throw std::runtime_error("Motor command sizes do not match total DOF");
   }
   pd_motor_commands_.ZeroAll(pd_motor_indices_.size());
   loco_motor_commands_.ZeroAll(loco_motor_indices_.size());
   final_motor_commands_.ZeroAll(total_motor_dof_);
}

void InterfaceBase::InitProprioception()
{
   if (pd_proprio_indices_.size() + loco_proprio_indices_.size() != total_proprio_dof_)
   {
      throw std::runtime_error("Joint sizes do not match total DOF");
   }
   full_proprioception_.ZeroAll(total_proprio_dof_);
   loco_proprioception_.ZeroAll(loco_proprio_indices_.size());
   pd_proprioception_.ZeroAll(pd_proprio_indices_.size());
}

std::vector<int> InterfaceBase::GetJointIndicesFromSubset(
    const std::vector<std::string> &all_encoder_names_pinocchio_order,
    const std::vector<std::string> &subset_joint_names)
{
   std::vector<int> subset_indices;
   subset_indices.reserve(subset_joint_names.size());

   // Build a lookup: joint_name -> index in pinocchio ordering
   std::unordered_map<std::string, int> name_to_index;
   for (size_t i = 0; i < all_encoder_names_pinocchio_order.size(); ++i)
   {
      name_to_index[all_encoder_names_pinocchio_order[i]] = static_cast<int>(i);
   }

   // Collect indices for the subset
   for (const auto &joint_name : subset_joint_names)
   {
      auto it = name_to_index.find(joint_name);
      if (it != name_to_index.end())
      {
         subset_indices.push_back(it->second);
      }
   }

   return subset_indices;
}

BipedProprioception InterfaceBase::Update(const BipedMotorCommands &loco_cmd)
{
   ReadAndEstimate();
   loco_motor_commands_ = loco_cmd;
   return loco_proprioception_;
}


void InterfaceBase::ReconfigurePdMotorCommands(const Eigen::VectorXd &Kp, const Eigen::VectorXd &Kd, const Eigen::VectorXd &qd)
{
   //must be called after InitMotorCommands()
   pd_motor_commands_.joint_positions = qd;
   pd_motor_commands_.joint_velocities.setZero();
   pd_motor_commands_.joint_kp = Kp;
   pd_motor_commands_.joint_kd = Kd;
   pd_motor_commands_.joint_torques_ff.setZero();
   final_motor_commands_.UpdatePartialWithIndices(pd_motor_commands_);
}