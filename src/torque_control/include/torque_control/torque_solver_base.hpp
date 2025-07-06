#ifndef TORQUE_SOLVER_BASE_HPP
#define TORQUE_SOLVER_BASE_HPP

#include <memory>
#include <string>
#include <Eigen/Dense>
#include "biped_utils/yaml_parser.hpp"
#include "biped_core/output_base.hpp"
#include "biped_core/robot_base_pinocchio.hpp"
// IMPORTANT: make sure robot and outputs are updated before calling Solve

class TorqueSolverBase
{
public:
    TorqueSolverBase(std::shared_ptr<RobotBasePinocchio> robot, std::shared_ptr<OutputBase> output)
        : robot_(robot), output_(output) {}

    virtual ~TorqueSolverBase() = default;

    virtual void Init(const std::string &config_file) = 0;
    virtual Eigen::VectorXd Solve() = 0;

protected:
    // solve for gravity compensation
    Eigen::VectorXd SolveGravityCompensation();

    std::shared_ptr<RobotBasePinocchio> robot_;
    std::shared_ptr<OutputBase> output_;

    YAMLParser yaml_parser_;
};

#endif // TORQUE_SOLVER_BASE_HPP
