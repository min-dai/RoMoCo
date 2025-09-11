#ifndef INTERFACE_BASE_HPP
#define INTERFACE_BASE_HPP

#include <Eigen/Dense>
#include "romoco_core/robot_base_pinocchio.hpp"
#include "romoco_core/biped_proprioception.hpp"
#include "romoco_core/biped_motor_commands.hpp"
#include "romoco_utils/pd_controller.hpp"

namespace romoco
{
class InterfaceBase
{
public:
    explicit InterfaceBase(bool initial_ready_state = false) : ready_for_control_(initial_ready_state) {}

    virtual ~InterfaceBase();

    // Use internal sensor_ to update internal estimation to update
    // full_proprioception_, loco_proprioception_, and pd_proprioception_
    virtual BipedProprioception ReadAndEstimate() = 0;

    virtual bool is_sim() const = 0;

    bool ready_for_control() const { return ready_for_control_; }

    int loco_motor_dof() const { return loco_motor_dof_; }

    // update loco_motor_commands, pd_motor_commands are constants
    // return loco_proprioception_
    void ProcessMotorCommands(const BipedMotorCommands &loco_cmd);

    // Send control commands to the robot
    // Internally update sensor_ (type depends on HW/SIM)
    virtual void SendPacket() = 0;

    virtual bool IsInterfaceRunning() const = 0;

    virtual void Init(const std::string &config_folder, const std::string &log_path) = 0;

    // for hardware interface, do nothing for sim interface
    virtual void DampedInitializationControl() {};

protected:
    void InitLogFile(const std::string &log_path);

    void InitDofAndIndicesFromConfigFile(const std::string &config_folder);

    void InitMotorCommands();

    void InitProprioception();

    void CheckInitialization() const;

    std::vector<int> GetJointIndicesFromSubset(
        const std::vector<std::string> &all_encoder_names_pinocchio_order,
        const std::vector<std::string> &subset_joint_names);

    void ReconfigurePdMotorCommands(const Eigen::VectorXd &Kp, const Eigen::VectorXd &Kd, const Eigen::VectorXd &qd);
    void ReconfigurePdMotorCommands(std::string &config_file);

    void PrintDebugInfo() const;

    std::vector<std::string> pd_encoder_names_;
    std::vector<std::string> loco_encoder_names_;
    std::vector<std::string> all_encoder_names_pinocchio_order_;

    int total_proprio_dof_;
    std::vector<int> pd_proprio_indices_;
    std::vector<int> loco_proprio_indices_;
    std::vector<int> loco_proprio_motor_indices_;

    // interface recieves sensor data and parse to full proprioception
    // then ros node publish loco proprioception for controller
    BipedProprioception full_proprioception_;
    BipedProprioception loco_proprioception_;
    BipedProprioception pd_proprioception_;

    int total_motor_dof_;
    int pd_motor_dof_;
    int loco_motor_dof_;

    bool is_sim_;

    // set at beginning, constant after
    BipedMotorCommands pd_motor_commands_;
    // updated every cycle
    BipedMotorCommands loco_motor_commands_;
    BipedMotorCommands final_motor_commands_;
    // pd_motor_commands = final_motor_commands.GetIndex(pd_motor_command_indices);
    // loco_motor_commands = final_motor_commands.GetIndex(loco_motor_command_indices);
    std::vector<int> pd_motor_command_indices_;
    std::vector<int> loco_motor_command_indices_;

    Eigen::VectorXf CollectLog(const double t, const std::vector<Eigen::VectorXd> &vectors);

    // for Log files
    std::fstream logFile_;
    std::string logFilePath_;

    // for hardware initialization
    bool ready_for_control_ = false;
};

} // namespace romoco

#endif // INTERFACE_BASE_HPP