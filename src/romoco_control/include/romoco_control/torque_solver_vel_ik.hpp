#ifndef TORQUE_SOLVER_VELIK_HPP
#define TORQUE_SOLVER_VELIK_HPP

#include "romoco_control/torque_solver_base.hpp"
#include "biped_utils/pd_controller.hpp"

class TorqueSolverVELIK : public TorqueSolverBase
{
public:
    TorqueSolverVELIK(const std::string &config_file, std::shared_ptr<RobotBasePinocchio> robot, std::shared_ptr<OutputBase> output);

    void Init(const std::string &config_file) override;

    BipedMotorCommands Solve() override;

private:
    PDController pd_controller_;
    Eigen::VectorXd JointKP_, JointKD_;
    Eigen::VectorXd JointKPing_, JointKDing_;
    Eigen::VectorXd output_ik_gain_;
    VectorXd qm_actual_, dqm_actual_;

    void SolveIk();

};

#endif // TORQUE_SOLVER_VELIK_HPP
