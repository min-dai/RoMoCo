#pragma once


#include "torque_control/torque_solver_base.hpp"




class TorqueSolverInvDyn : public TorqueSolverBase
{
public:
    TorqueSolverInvDyn(const std::string& config_file, std::shared_ptr<RobotBasePinocchio> robot, std::shared_ptr<OutputBase> output);

    void Init(const std::string& config_file) override;

    Eigen::VectorXd Solve() override;

    void reset();

private:
    


    double threshold_;
    VectorXd OutputKP_, OutputKD_;
    VectorXd OutputKPing_, OutputKDing_;

    


};
