#ifndef TORQUE_SOLVER_BASE_HPP
#define TORQUE_SOLVER_BASE_HPP

#include <memory>
#include <string>
#include <Eigen/Dense>
#include "romoco_utils/yaml_parser.hpp"
#include "romoco_core/output_base.hpp"
#include "romoco_core/robot_base_pinocchio.hpp"
#include "romoco_core/biped_motor_commands.hpp"

namespace romoco
{
    /**
     * @class TorqueSolverBase
     * @brief An abstract base class for torque solvers, providing common functionality for different torque solving strategies.
     * @note The Solve() method must be called after updating the RobotBasePinocchio and OutputBase instances.
     * @ingroup group_controller
     * This class serves as a foundation for implementing various torque solving algorithms for biped robots.
     * It manages the robot model and output generation, and provides a method for solving the torque commands.
     * The TorqueSolverBase class is designed to be extended by specific torque solver implementations.
     */
    class TorqueSolverBase
    {
    public:
        TorqueSolverBase(std::shared_ptr<romoco::robot::RobotBasePinocchio> robot, std::shared_ptr<OutputBase> output);

        virtual ~TorqueSolverBase() = default;

        virtual void Init(const std::string &config_file) = 0;
        /**
         * @brief Solves for the torque commands based on the current robot state and desired outputs.
         * @return BipedMotorCommands A structure containing the computed torque commands for the robot's motors.
         * @note This is a pure virtual function that must be implemented by derived classes.
         */
        virtual BipedMotorCommands Solve() = 0;

    protected:
        /**
         * @brief Solves for the gravity compensation torques.
         * @return Eigen::VectorXd A vector containing the gravity compensation torques.
         */
        Eigen::VectorXd SolveGravityCompensation();

        std::shared_ptr<romoco::robot::RobotBasePinocchio> robot_;
        std::shared_ptr<OutputBase> output_;

        YAMLParser yaml_parser_;

        BipedMotorCommands motor_commands_;
    };

} // namespace romoco

#endif // TORQUE_SOLVER_BASE_HPP
