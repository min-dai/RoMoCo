#ifndef TORQUE_SOLVER_TSCQP_IK_HPP
#define TORQUE_SOLVER_TSCQP_IK_HPP

#include "romoco_control/torque_solver_base.hpp"
#include "romoco_utils/pd_controller.hpp"

#include <clarabel.hpp>
#include <optional>
namespace romoco
{
    /**
     * @class TorqueSolverQPIK
     * @brief A torque solver that uses a QP as feedforward for position IK.
     * @ingroup group_solver
     * This class extends the TorqueSolverBase class to implement a torque solver that utilizes a Quadratic Programming (QP) approach
     * for inverse kinematics (IK) with position control. It integrates with the Clarabel QP solver to compute optimal joint torques and forces
     * while considering various constraints and objectivess
     */
    class TorqueSolverQPIK : public TorqueSolverBase
    {
    public:
        // var = [ddq; u; F]
        TorqueSolverQPIK(const std::string &config_file, std::shared_ptr<romoco::robot::RobotBasePinocchio> robot, std::shared_ptr<OutputBase> output);

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
        void ComputeQuadraticCostTerms(const Eigen::MatrixXd &Acost, const Eigen::VectorXd &bcost, Eigen::MatrixXd &G, Eigen::VectorXd &g);
        void SolveIk();

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

        double tol_ = 1e-3;
        int max_iter_ = 100;

        Eigen::VectorXd qm_des_, dqm_des_;

        Eigen::VectorXd qm_actual_, dqm_actual_;

        Eigen::VectorXd JointKP_, JointKD_;
        PDController pd_controller_;
        Eigen::VectorXd u_sol;

        // Settings for the Clarabel QP solver
        clarabel::DefaultSettings<double> settings_;
    };

} // namespace romoco

#endif // TORQUE_SOLVER_TSCQP_IK_HPP