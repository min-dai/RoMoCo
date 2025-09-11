#ifndef TORQUE_SOLVER_VELIK_HPP
#define TORQUE_SOLVER_VELIK_HPP

#include "romoco_control/torque_solver_base.hpp"
#include "romoco_utils/pd_controller.hpp"
namespace romoco
{
    /**
     * @class TorqueSolverVELIK
     * @brief A torque solver that uses velocity-based inverse kinematics to compute torque commands for a biped robot.
     * @ingroup group_controller
     * This class extends the TorqueSolverBase class to implement a torque solver based on velocity-based inverse kinematics.
     * It computes the required joint torques to achieve desired velocities while compensating for gravity and other dynamic effects.
     * The TorqueSolverVELIK class utilizes the robot model and output generation provided by the RobotBasePinocchio and OutputBase classes.
     */
    class TorqueSolverVELIK : public TorqueSolverBase
    {
    public:
        TorqueSolverVELIK(const std::string &config_file, std::shared_ptr<romoco::robot::RobotBasePinocchio> robot, std::shared_ptr<OutputBase> output);

        void Init(const std::string &config_file) override;

        BipedMotorCommands Solve() override;

    private:
        PDController pd_controller_;
        Eigen::VectorXd JointKP_, JointKD_;

        Eigen::VectorXd output_ik_gain_;

        Eigen::VectorXd qm_des_, dqm_des_;

        Eigen::VectorXd qm_actual_, dqm_actual_;

        void SolveIk();
    };

} // namespace romoco

#endif // TORQUE_SOLVER_VELIK_HPP
