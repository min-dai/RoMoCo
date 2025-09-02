#ifndef INTERFACE_BASE_HPP
#define INTERFACE_BASE_HPP

#include <Eigen/Dense>
#include "biped_core/robot_base_pinocchio.hpp"
#include "biped_types/biped_proprioception.hpp"
#include "biped_types/biped_motor_commands.hpp"
#include "biped_utils/pd_controller.hpp"
class InterfaceBase
{
public:
    explicit InterfaceBase() {}

    virtual ~InterfaceBase() = default;

    BipedProprioception Update(const BipedMotorCommands &loco_cmd);

    virtual void SendPacket() = 0;
    
protected:
    virtual void Init(const std::string &config_folder) = 0;

    // Pull raw sensors and update internal estimation to update proprioception_
    virtual void ReadAndEstimate() = 0;

    



    void InitMotorCommands();

    void InitProprioception();

    std::vector<int> GetJointIndicesFromSubset(
        const std::vector<std::string> &all_encoder_names_pinocchio_order,
        const std::vector<std::string> &subset_joint_names);

    void ReconfigurePdMotorCommands(const Eigen::VectorXd &Kp, const Eigen::VectorXd &Kd, const Eigen::VectorXd &qd);


    int total_proprio_dof_;
    std::vector<int> pd_proprio_indices_;
    std::vector<int> loco_proprio_indices_;

    BipedProprioception full_proprioception_;
    BipedProprioception loco_proprioception_;
    BipedProprioception pd_proprioception_;

    int total_motor_dof_;
    std::vector<int> pd_motor_indices_;
    std::vector<int> loco_motor_indices_;

    BipedMotorCommands pd_motor_commands_;
    BipedMotorCommands loco_motor_commands_;
    BipedMotorCommands final_motor_commands_;


};

#endif // INTERFACE_BASE_HPP