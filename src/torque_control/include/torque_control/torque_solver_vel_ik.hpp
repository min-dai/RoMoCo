#ifndef TORQUE_SOLVER_VELIK_HPP
#define TORQUE_SOLVER_VELIK_HPP

#include "torque_control/torque_solver_base.hpp"
#include "torque_control/pd_controller.hpp"

class TorqueSolverVELIK : public TorqueSolverBase
{
public:
    TorqueSolverVELIK(const std::string &config_file, std::shared_ptr<RobotBasePinocchio> robot, std::shared_ptr<OutputBase> output);

    void Init(const std::string &config_file) override;

    Eigen::VectorXd Solve() override;

private:
    PDController pd_controller_;
    Eigen::VectorXd JointKP_, JointKD_;
    Eigen::VectorXd JointKPing_, JointKDing_;
    Eigen::VectorXd ik_gain_;

    void SolveIk();
    Eigen::VectorXd SolveTorqueFeedback();
};

#endif // TORQUE_SOLVER_VELIK_HPP
