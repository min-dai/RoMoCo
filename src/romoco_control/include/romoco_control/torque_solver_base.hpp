#ifndef TORQUE_SOLVER_BASE_HPP
#define TORQUE_SOLVER_BASE_HPP

#include <memory>
#include <string>
#include <Eigen/Dense>
#include "romoco_utils/yaml_parser.hpp"
#include "romoco_core/output_base.hpp"
#include "romoco_core/robot_base_pinocchio.hpp"
#include "romoco_core/biped_motor_commands.hpp"
// IMPORTANT: make sure robot and outputs are updated before calling Solve

namespace romoco
{
class TorqueSolverBase
{
public:
    TorqueSolverBase(std::shared_ptr<RobotBasePinocchio> robot, std::shared_ptr<OutputBase> output);

    virtual ~TorqueSolverBase() = default;

    virtual void Init(const std::string &config_file) = 0;
    virtual BipedMotorCommands Solve() = 0;

protected:
    // solve for gravity compensation
    Eigen::VectorXd SolveGravityCompensation();

    std::shared_ptr<RobotBasePinocchio> robot_;
    std::shared_ptr<OutputBase> output_;

    YAMLParser yaml_parser_;

    BipedMotorCommands motor_commands_;
};

} // namespace romoco

#endif // TORQUE_SOLVER_BASE_HPP
