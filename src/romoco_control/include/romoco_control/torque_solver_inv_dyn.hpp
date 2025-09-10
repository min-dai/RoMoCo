#ifndef TORQUE_SOLVER_INV_DYN_HPP
#define TORQUE_SOLVER_INV_DYN_HPP

#include "romoco_control/torque_solver_base.hpp"

namespace romoco
{
class TorqueSolverInvDyn : public TorqueSolverBase
{
public:
    TorqueSolverInvDyn(const std::string &config_file, std::shared_ptr<RobotBasePinocchio> robot, std::shared_ptr<OutputBase> output);

    void Init(const std::string &config_file) override;

    BipedMotorCommands Solve() override;

private:
    double threshold_;
    VectorXd OutputKP_, OutputKD_;
    VectorXd OutputKPing_, OutputKDing_;
};

} // namespace romoco

#endif // TORQUE_SOLVER_INV_DYN_HPP
