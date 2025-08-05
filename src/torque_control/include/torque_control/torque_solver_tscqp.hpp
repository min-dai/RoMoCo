#ifndef TORQUE_SOLVER_TSCQP_HPP
#define TORQUE_SOLVER_TSCQP_HPP

#include "torque_control/torque_solver_base.hpp"
#include <clarabel.hpp>



class TorqueSolverTSCQP : public TorqueSolverBase
{
public:
    // var = [ddq; u; F]
    TorqueSolverTSCQP(const std::string &config_file, std::shared_ptr<RobotBasePinocchio> robot, std::shared_ptr<OutputBase> output);

    void Init(const std::string &config_file) override;

    Eigen::VectorXd Solve() override;

    

private:
    void ResetSize();
    bool ClarabelSolve();

    bool print_qp_ = false;
    VectorXd OutputKP_, OutputKD_;
    VectorXd OutputKPing_, OutputKDing_;

    int nVar_;
    MatrixXd A_y_;
    VectorXd b_y_;
    MatrixXd G_, Aeq_, Aub_fric_, Aub_u_;
    VectorXd g_, beq_, bub_fric_;

    bool if_solved_ = false;
    VectorXd sol_, u_sol_, F_sol_, u_sol_prev_;

    // Settings for the Clarabel QP solver
    clarabel::DefaultSettings<double> settings_;
};

#endif // TORQUE_SOLVER_TSCQP_HPP