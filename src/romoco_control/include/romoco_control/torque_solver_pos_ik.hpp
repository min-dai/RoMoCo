#pragma once


#include "romoco_control/torque_solver_base.hpp"
#include "romoco_utils/pd_controller.hpp"

class TorqueSolverPOSIK : public TorqueSolverBase
{
public:
   TorqueSolverPOSIK(const std::string &config_file, std::shared_ptr<RobotBasePinocchio> robot, std::shared_ptr<OutputBase> output);

   void Init(const std::string &config_file) override;

   BipedMotorCommands Solve() override;



private:
   double tol_ = 1e-3;
   int max_iter_ = 100;

   VectorXd qm_des_, dqm_des_;



   VectorXd qm_actual_, dqm_actual_;

   VectorXd JointKP_, JointKD_;
   VectorXd JointKPing_, JointKDing_;

   VectorXd u_sol;

   void SolveIk();


   PDController pd_controller_;
};
