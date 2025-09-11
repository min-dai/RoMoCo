#ifndef TORQUE_SOLVER_TSCQP_HPP
#define TORQUE_SOLVER_TSCQP_HPP

#include "romoco_control/torque_solver_base.hpp"
#include <clarabel.hpp>


namespace romoco
{
class TorqueSolverTSCQP : public TorqueSolverBase
{
public:
    // var = [ddq; u; F]
    TorqueSolverTSCQP(const std::string &config_file, std::shared_ptr<RobotBasePinocchio> robot, std::shared_ptr<OutputBase> output);

    void Init(const std::string &config_file) override;

    BipedMotorCommands Solve() override;

    

private:
    void ResetSize();
    bool ClarabelSolve();

    bool print_qp_ = false;
    Eigen::VectorXd OutputKP_, OutputKD_;
    Eigen::VectorXd OutputKPing_, OutputKDing_;

    int nVar_;
    Eigen::MatrixXd A_y_;
    Eigen::VectorXd b_y_;
    Eigen::MatrixXd G_, Aeq_, Aub_fric_, Aub_u_;
    Eigen::VectorXd g_, beq_, bub_fric_;

    bool if_solved_ = false;
    Eigen::VectorXd sol_, u_sol_, F_sol_, u_sol_prev_;

    // Settings for the Clarabel QP solver
    clarabel::DefaultSettings<double> settings_;
};

} // namespace romoco

#endif // TORQUE_SOLVER_TSCQP_HPP