#include "biped_core/interface_base.hpp"
#include "biped_utils/yaml_parser.hpp"


void InterfaceBase::InitDofAndIndicesFromConfigFile(const std::string &config_folder)
{
   // pull locked and loco encoder names from interface config
   std::string interface_config_file = config_folder + "/interface_config.yaml";
   YAMLParser yaml_parser(interface_config_file);
   yaml_parser.Init(interface_config_file);
   pd_encoder_names_ = yaml_parser.get_string_vector("locked_encoder_names");
   loco_encoder_names_ = yaml_parser.get_string_vector("pinocchio_encoder_names");
   //verify they add up to all
   if (pd_encoder_names_.size() + loco_encoder_names_.size() != all_encoder_names_pinocchio_order_.size())
   {
      throw std::runtime_error("Interface Error: Locked and locomotion encoder names do not add up to all encoders");
   }

   // Initialize MotorCommands and Proprioception

   total_motor_dof_ = all_encoder_names_pinocchio_order_.size();
   loco_motor_indices_ = GetJointIndicesFromSubset(all_encoder_names_pinocchio_order_, loco_encoder_names_);
   pd_motor_indices_ = GetJointIndicesFromSubset(all_encoder_names_pinocchio_order_, pd_encoder_names_);

   total_proprio_dof_ = 6 + total_motor_dof_;

   // proprio indices = base + shifted motors
   loco_proprio_indices_ = {0, 1, 2, 3, 4, 5};
   loco_proprio_indices_.reserve(6 + loco_motor_indices_.size());

   // append shifted motor indices
   std::transform(loco_motor_indices_.begin(),
                  loco_motor_indices_.end(),
                  std::back_inserter(loco_proprio_indices_),
                  [](int idx)
                  { return idx + 6; });

   // offset 
   motored_loco_proprio_indices_  = loco_motor_indices_;
   for (int &x : motored_loco_proprio_indices_)
   {
      x += 6;
   }
   // simply offset by 6 for PD joints               
   pd_proprio_indices_ = pd_motor_indices_;

   for (int &x : pd_proprio_indices_)
   {
      x += 6;
   }


}
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

BipedProprioception InterfaceBase::Update()
{
   ReadAndEstimate();
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
   pd_motor_commands_.joint_torques.setZero();
   final_motor_commands_.UpdatePartialWithIndices(pd_motor_commands_);
}

void InterfaceBase::ReconfigurePdMotorCommands(std::string &config_file)
{
   //must be called after InitMotorCommands()
   YAMLParser yaml_parser(config_file);
   pd_motor_commands_.joint_positions = yaml_parser.get_VectorXd("qdes_locked_joints");
   pd_motor_commands_.joint_velocities.setZero();
   pd_motor_commands_.joint_kp = yaml_parser.get_VectorXd("Kp_locked_joints");
   pd_motor_commands_.joint_kd = yaml_parser.get_VectorXd("Kd_locked_joints");
   pd_motor_commands_.joint_torques_ff.setZero();
   pd_motor_commands_.joint_torques.setZero();
   final_motor_commands_.UpdatePartialWithIndices(pd_motor_commands_);
}

void InterfaceBase::PrintDebugInfo() const
{
    std::cout << "=== Interface Debug Info ===" << std::endl;
    std::cout << "PD proprio indices: ";
    for (const auto idx : pd_proprio_indices_) {
        std::cout << idx << " ";
    }
    std::cout << std::endl;

    std::cout << "Loco proprio indices: ";
    for (const auto idx : loco_proprio_indices_) {
        std::cout << idx << " ";
    }
    std::cout << std::endl;

    std::cout << "PD motor indices: ";
    for (const auto idx : pd_motor_indices_) {
        std::cout << idx << " ";
    }
    std::cout << std::endl;
    
    std::cout << "Loco motor indices: ";
    for (const auto idx : loco_motor_indices_) {
        std::cout << idx << " ";
    }
    std::cout << std::endl;
    std::cout << "=========================" << std::endl;
}