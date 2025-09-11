#ifndef TORQUE_SOLVER_POS_IK_HPP
#define TORQUE_SOLVER_POS_IK_HPP

#include "romoco_control/torque_solver_base.hpp"
#include "romoco_utils/pd_controller.hpp"
namespace romoco
{
   /**
    * @class TorqueSolverPOSIK
    * @brief A torque solver that uses position-based inverse kinematics to compute torque commands for a biped robot.
    * @ingroup group_controller
    * This class extends the TorqueSolverBase class to implement a torque solver based on position-based inverse kinematics.
    * It computes the required joint torques to achieve desired positions while compensating for gravity and other dynamic effects.
    * The TorqueSolverPOSIK class utilizes the robot model and output generation provided by the RobotBasePinocchio and OutputBase classes.
    */
   class TorqueSolverPOSIK : public TorqueSolverBase
   {
   public:
      TorqueSolverPOSIK(const std::string &config_file, std::shared_ptr<romoco::robot::RobotBasePinocchio> robot, std::shared_ptr<OutputBase> output);

      void Init(const std::string &config_file) override;

      BipedMotorCommands Solve() override;

   private:
      double tol_ = 1e-3;
      int max_iter_ = 100;

      Eigen::VectorXd qm_des_, dqm_des_;

      Eigen::VectorXd qm_actual_, dqm_actual_;

      Eigen::VectorXd JointKP_, JointKD_;


      Eigen::VectorXd u_sol;

      void SolveIk();

      PDController pd_controller_;
   };

} // namespace romoco

#endif // TORQUE_SOLVER_POS_IK_HPP
