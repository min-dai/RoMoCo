#pragma once


#include "torque_control/torque_solver_base.hpp"
#include "torque_control/pd_controller.hpp"

class TorqueSolverPOSIK : public TorqueSolverBase
{
public:
   TorqueSolverPOSIK(const std::string &config_file, std::shared_ptr<RobotBasePinocchio> robot, std::shared_ptr<OutputBase> output);

   void Init(const std::string &config_file) override;

   Eigen::VectorXd Solve() override;



private:
   std::vector<int> get_unactuated_indices(const std::vector<int> &actuated_idx, int nq);

   std::vector<int> unactuated_idx_;

   double tol_ = 1e-3;
   int max_iter_ = 100;



   VectorXd qm_actual_, dqm_actual_;

   VectorXd JointKP_, JointKD_;
   VectorXd JointKPing_, JointKDing_;

   VectorXd u_sol;

   void SolveIk();


   PDController pd_controller_;
};
