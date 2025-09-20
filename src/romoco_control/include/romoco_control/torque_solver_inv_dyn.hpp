#ifndef TORQUE_SOLVER_INV_DYN_HPP
#define TORQUE_SOLVER_INV_DYN_HPP

#include "romoco_control/torque_solver_base.hpp"

namespace romoco
{
    /**
     * @class TorqueSolverInvDyn
     * @brief A torque solver that uses inverse dynamics to compute torque commands for a biped robot.
     * @ingroup group_solver
     * This class extends the TorqueSolverBase class to implement a torque solver based on inverse dynamics.
     * It computes the required joint torques to achieve desired accelerations while compensating for gravity and other dynamic effects.
     * The TorqueSolverInvDyn class utilizes the robot model and output generation provided by the RobotBasePinocchio and OutputBase classes.
     */
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
