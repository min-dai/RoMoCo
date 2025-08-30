#ifndef INTERFACE_BASE_HPP
#define INTERFACE_BASE_HPP

#include <Eigen/Dense>
#include "biped_core/robot_base_pinocchio.hpp"
#include "biped_types/biped_proprioception.hpp"
#include "biped_types/biped_motor_commands.hpp"
class InterfaceBase
{
public:
    explicit InterfaceBase() {}

    virtual ~InterfaceBase() = default;

    // Pull raw sensors and update internal estimation to update proprioception_
    virtual void ReadAndEstimate() = 0;

    virtual void sendPacket(const BipedMotorCommands &cmd) = 0;

    // use proprioception_ to compute pd_motor_commands_
    virtual void ComputePdMotorCommands() = 0;

    BipedProprioception Update(const BipedMotorCommands &loco_cmd);

protected:
    std::vector<int> GetJointIndicesFromSubset(
        const std::vector<std::string> &all_joint_names_pinocchio_order,
        const std::vector<std::string> &subset_joint_names);

    void InitMotorCommands()
    {
        pd_motor_commands_.ZeroAll(pd_joint_indices_.size());
        loco_motor_commands_.ZeroAll(loco_joint_indices_.size());
        final_motor_commands_.ZeroAll(total_dof_);
    }

    std::vector<int> pd_joint_indices_;
    std::vector<int> loco_joint_indices_;
    int total_dof_;

    BipedProprioception full_proprioception_;
    BipedProprioception loco_proprioception_;
    BipedProprioception pd_proprioception_;

    BipedMotorCommands pd_motor_commands_;
    BipedMotorCommands loco_motor_commands_;
    BipedMotorCommands final_motor_commands_;


};

#endif // INTERFACE_BASE_HPP