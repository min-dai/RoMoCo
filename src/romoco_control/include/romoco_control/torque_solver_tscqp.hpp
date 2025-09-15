#ifndef TORQUE_SOLVER_TSCQP_HPP
#define TORQUE_SOLVER_TSCQP_HPP

#include "romoco_control/torque_solver_base.hpp"
#include <clarabel.hpp>
#include <optional>
namespace romoco
{
    /**
     * @class TorqueSolverTSCQP
     * @brief A torque solver that uses a time-stepping contact QP approach to compute torque commands for a biped robot.
     * @ingroup group_controller
     * This class extends the TorqueSolverBase class to implement a torque solver based on a time-stepping contact QP approach.
     * It computes the required joint torques to achieve desired accelerations while handling contact constraints and friction.
     * The TorqueSolverTSCQP class utilizes the robot model and output generation provided by the RobotBasePinocchio and OutputBase classes.
     */
    class TorqueSolverTSCQP : public TorqueSolverBase
    {
    public:
        // var = [ddq; u; F]
        TorqueSolverTSCQP(const std::string &config_file, std::shared_ptr<romoco::robot::RobotBasePinocchio> robot, std::shared_ptr<OutputBase> output);

        void Init(const std::string &config_file) override;

        BipedMotorCommands Solve() override;

    private:
        void ResetSize();
        bool ClarabelSolve(const Eigen::MatrixXd &G,
                           const Eigen::VectorXd &g,
                           const std::optional<Eigen::MatrixXd> &Aub,
                           const std::optional<Eigen::VectorXd> &bub,
                           const std::optional<Eigen::MatrixXd> &Aeq,
                           const std::optional<Eigen::VectorXd> &beq,
                           Eigen::VectorXd &sol);
        void ComputeWeightedQuadraticCostTerms(const Eigen::MatrixXd &Acost, const Eigen::VectorXd &bcost, const Eigen::VectorXd &weights, Eigen::MatrixXd &G, Eigen::VectorXd &g);


        bool print_qp_ = false;
        Eigen::VectorXd OutputKP_, OutputKD_;
        Eigen::VectorXd OutputW_;

        int nVar_;
        Eigen::MatrixXd A_y_;
        Eigen::VectorXd b_y_;
        Eigen::MatrixXd G_, Aeq_, Aub_, Aub_fric_, Aub_u_;
        Eigen::VectorXd g_, beq_, bub_, bub_fric_;

        bool if_solved_ = false;
        Eigen::VectorXd sol_, u_sol_, F_sol_, u_sol_prev_;

        // Settings for the Clarabel QP solver
        clarabel::DefaultSettings<double> settings_;
    };

} // namespace romoco

#endif // TORQUE_SOLVER_TSCQP_HPP