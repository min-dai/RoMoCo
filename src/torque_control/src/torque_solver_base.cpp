#include "torque_control/torque_solver_base.hpp"

Eigen::VectorXd TorqueSolverBase::SolveGravityCompensation()
{
   // Inverse Dynamics Control of Floating Base Systems using Orthogonal projection
   Eigen::MatrixXd Q = output_->Jh.transpose().householderQr().householderQ();

   Eigen::MatrixXd Su, Sc;
   Su.resize(robot_->nv() - output_->nh(), robot_->nv());
   Sc.resize(output_->nh(), robot_->nv());
   Su << Eigen::MatrixXd::Zero(robot_->nv() - output_->nh(), output_->nh()), Eigen::MatrixXd::Identity(robot_->nv() - output_->nh(), robot_->nv() - output_->nh());
   Sc << Eigen::MatrixXd::Identity(output_->nh(), output_->nh()), Eigen::MatrixXd::Zero(output_->nh(), robot_->nv() - output_->nh());

   Eigen::MatrixXd B = robot_->B()(Eigen::all, output_->actuated_u_idx);

   Eigen::VectorXd u_sol = (Su * Q.transpose() * B).completeOrthogonalDecomposition().solve(Su) * Q.transpose() * robot_->G();


   // std::cout << "u_ff = " << u_sol.transpose() << std::endl;

   return u_sol;
}

