#ifndef TORQUE_SOLVER_INV_DYN_HPP
#define TORQUE_SOLVER_INV_DYN_HPP

#include "romoco_control/torque_solver_base.hpp"

namespace romoco
{
class TorqueSolverInvDyn : public TorqueSolverBase
{
public:
    TorqueSolverInvDyn(const std::string &config_file, std::shared_ptr<romoco::robot::RobotBasePinocchio> robot, std::shared_ptr<OutputBase> output);

    void Init(const std::string &config_file) override;

    BipedMotorCommands Solve() override;

private:
    double threshold_;
    Eigen::VectorXd OutputKP_, OutputKD_;

};

} // namespace romoco

#endif // TORQUE_SOLVER_INV_DYN_HPP
