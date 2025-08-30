#include "biped_core/interface_base.hpp"

std::vector<int> InterfaceBase::GetJointIndicesFromSubset(
    const std::vector<std::string> &all_joint_names_pinocchio_order,
    const std::vector<std::string> &subset_joint_names)
{
   std::vector<int> subset_indices;
   subset_indices.reserve(subset_joint_names.size());

   // Build a lookup: joint_name -> index in pinocchio ordering
   std::unordered_map<std::string, int> name_to_index;
   for (size_t i = 0; i < all_joint_names_pinocchio_order.size(); ++i)
   {
      name_to_index[all_joint_names_pinocchio_order[i]] = static_cast<int>(i);
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
