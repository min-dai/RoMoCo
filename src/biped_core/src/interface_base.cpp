#include "biped_core/interface_base.hpp"
#include "biped_utils/yaml_parser.hpp"
#include <filesystem> //for create_directories
InterfaceBase::~InterfaceBase()
{
   if (logFile_.is_open())
   {
      logFile_.close();
   }
}

void InterfaceBase::InitLogFile(const std::string &log_path, bool isSim)
{
   // Check if the log directory exists, if not, create it
   if (!std::filesystem::exists(log_path))
   {
      if (std::filesystem::create_directories(log_path))
      {
         std::cout << "Log directory created: " << log_path << std::endl;
      }
      else
      {
         std::cerr << "Failed to create log directory: " << log_path<< std::endl;
      }
   }
   if (isSim)
       logFilePath_ = log_path + "/logSimInterface.bin";
   else
       logFilePath_ = log_path + "/logInterface.bin";
       
   logFile_.open(logFilePath_, std::ios::out | std::ios::binary);
}

void InterfaceBase::InitDofAndIndicesFromConfigFile(const std::string &config_folder)
{
   // pull locked and loco encoder names from interface config
   std::string interface_config_file = config_folder + "/interface_config.yaml";
   YAMLParser yaml_parser(interface_config_file);
   yaml_parser.Init(interface_config_file);
   all_encoder_names_pinocchio_order_ = yaml_parser.get_string_vector("all_encoder_names");
   pd_encoder_names_ = yaml_parser.get_string_vector("locked_encoder_names");
   loco_encoder_names_ = yaml_parser.get_string_vector("pinocchio_encoder_names");


   std::vector<std::string> passive_encoder_names = yaml_parser.get_string_vector("passive_encoder_names");

   //verify they add up to all
   if (pd_encoder_names_.size() + loco_encoder_names_.size() != all_encoder_names_pinocchio_order_.size())
   {
      throw std::runtime_error("Interface Error: Locked and locomotion encoder names do not add up to all encoders");
   }

   // Initialize MotorCommands and Proprioception

   int total_encoder_dof = all_encoder_names_pinocchio_order_.size();
   total_proprio_dof_ = 6 + total_encoder_dof;
   pd_motor_dof_ = pd_encoder_names_.size();
   loco_motor_dof_ = loco_encoder_names_.size() - passive_encoder_names.size();
   total_motor_dof_ = pd_motor_dof_ + loco_motor_dof_;

   std::vector<int> loco_encoder_indices = GetJointIndicesFromSubset(all_encoder_names_pinocchio_order_, loco_encoder_names_);
   std::vector<int> pd_encoder_indices = GetJointIndicesFromSubset(all_encoder_names_pinocchio_order_, pd_encoder_names_);

   std::vector<int> passive_encoder_indices = GetJointIndicesFromSubset(all_encoder_names_pinocchio_order_, passive_encoder_names);

   // proprio indices = base + shifted encoders
   loco_proprio_indices_ = {0, 1, 2, 3, 4, 5};
   loco_proprio_indices_.reserve(6 + loco_encoder_indices.size());
   std::transform(loco_encoder_indices.begin(),
                  loco_encoder_indices.end(),
                  std::back_inserter(loco_proprio_indices_),
                  [](int idx)
                  { return idx + 6; });

   // loco_proprio_motor_indices_ are actuated joint indices in full proprio
   for (int idx : loco_encoder_indices) {
    if (std::find(passive_encoder_indices.begin(), passive_encoder_indices.end(), idx) == passive_encoder_indices.end()) {
        loco_proprio_motor_indices_.push_back(idx+6);
    }
   }

   // simply offset by 6 for PD joints               
   pd_proprio_indices_ = pd_encoder_indices;
   for (int &x : pd_proprio_indices_)
   {
      x += 6;
   }

   PrintDebugInfo();

}
void InterfaceBase::InitMotorCommands()
{
   if (total_motor_dof_ <= 0 || loco_motor_dof_ <= 0)
   {
      throw std::runtime_error("motor DOF not initialized. Call InitDofAndIndicesFromConfigFile first.");
   }

   pd_motor_commands_.ZeroAll(pd_motor_dof_);
   loco_motor_commands_.ZeroAll(loco_motor_dof_);
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

void InterfaceBase::CheckInitialization() const
{
   if (pd_proprio_indices_.empty() || loco_proprio_indices_.empty())
   {
      throw std::runtime_error("Proprio indices not initialized. Call InitDofAndIndicesFromConfigFile first.");
   }
   if (total_motor_dof_ <= 0 || total_proprio_dof_ <= 0)
   {
      throw std::runtime_error("Total DOF not initialized. Call InitDofAndIndicesFromConfigFile first.");
   }
   if (loco_proprio_motor_indices_.empty())
   {
      throw std::runtime_error("Motored loco proprio indices not initialized. Call InitDofAndIndicesFromConfigFile first.");
   }

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


void InterfaceBase::ProcessMotorCommands(const BipedMotorCommands &loco_cmd)
{
   loco_motor_commands_.Update(loco_cmd);
   final_motor_commands_.UpdatePartialWithIndices(loco_motor_commands_);
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
   for (const auto idx : pd_proprio_indices_)
   {
      std::cout << idx << " ";
   }
   std::cout << std::endl;

   std::cout << "Loco proprio indices: ";
   for (const auto idx : loco_proprio_indices_)
   {
      std::cout << idx << " ";
   }
   std::cout << std::endl;

   std::cout << "Loco proprio motor indices: ";
   for (const auto idx : loco_proprio_motor_indices_)
   {
      std::cout << idx << " ";
   }
   std::cout << std::endl;
   std::cout << "=========================" << std::endl;
}

Eigen::VectorXf InterfaceBase::CollectLog(const double t, const std::vector<VectorXd> &vectors)
{
   int logsize = 1; // Start with 1 for the time
   for (const auto &vec : vectors)
   {
      logsize += vec.size();
   }

   Eigen::VectorXf log(logsize);
   log(0) = static_cast<float>(t);
   int offset = 1; // Start after the constant
   for (const auto &vec : vectors)
   {
      log.segment(offset, vec.size()) = vec.cast<float>();
      offset += vec.size();
   }
   return log;
}