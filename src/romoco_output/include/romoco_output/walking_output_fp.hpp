
#ifndef ROMOCO_WALKING_OUTPUT_FP_HPP
#define ROMOCO_WALKING_OUTPUT_FP_HPP

#include "romoco_output/walking_output_base.hpp"
#include "romoco_utils/phase_variable.hpp"

#include "romoco_output/com_states_rel_to_stance.hpp"
#include "romoco_planner/flatfoot_fp_planner.hpp"


namespace romoco
{

/**
 * @class WalkingOutputFp
 * @brief An output class for generating desired outputs during walking using a foot placement strategy for a biped robot.
 * @ingroup group_output
 * This class extends the WalkingOutputBase class to provide specific implementations for generating desired outputs
 * during walking using a foot placement strategy. It manages the walking domain, contact status, and planner inputs/outputs.
 * The WalkingOutputFp class utilizes a FlatFootFPPlanner to compute foot placements based on the robot's state and desired commands.
 */

class WalkingOutputFp : public WalkingOutputBase
{
public:
      enum OuptutIndices
      {
            zCOM,
            stanceHipYaw,
            basePitch,
            baseRoll,
            swingStepx,
            swingStepy,
            swingStepz,
            swingHipYaw,
            SwingDeltaPitch,
            SwingDeltaRoll
      };

      WalkingOutputFp(const std::string &config_file, std::shared_ptr<romoco::robot::RobotBasePinocchio> robot);
      void UpdateOutput(const DesiredCommand &command, const double &t, const double &t_old) override;

      void updateTargetWalkingRadio(const DesiredCommand &command);
      void ComputeActual() override;
      void ComputeDesired();

      void timeBasedDomainContactStatusSwitch(double t);

      std::vector<Eigen::VectorXd> CollectLog() const override;

      Eigen::Vector2d computeFPwithROmodel();

      void Init(const std::string &config_file);

private:

      PhaseVariable phase;

      Kinematics3D com2stance_zeroyaw;
      Kinematics3D sw2stance_zeroyaw;
      Kinematics1D sw_hipyaw, st_hipyaw;
      Kinematics1D sw_deltapitch, sw_deltaroll;
      struct Config : public ConfigBase
      {
            void Init(RobotType robot_type);
            double robot_initial_yaw; // todo, if delta_yaw==0, set to track initial yaw, allow modification of it
      } config;

      bool firstStepInitialized = false;

      struct Updated
      {
            Domain curSagDomain;

            Eigen::Vector2d PhaseRange;
            double t;
            double dt;

            double target_yaw, delta_yaw;
            double delta_yaw_des, stance_toe_yaw;

            double tOAp, tUAp;
            double TOA, TUA;
            double tNstep0;

            double TSS, TDS;

            Eigen::VectorXd y0_UA;
            Eigen::VectorXd y0d_OA;

            Eigen::VectorXd bezierCOMz; // bezierCOMz = (1-bezierComVertical)*y0 + bezierComVertical*y1

            Eigen::VectorXd bezierSwingx, bezierSwingy, bezierSwingz;

            Eigen::VectorXd bezierStanceHipYaw, bezierSwingHipYaw;

            Eigen::Vector2d planned_footstep;

            bool isDSP;

            double desiredVx, desiredVy;
            double stepWidth;
            double zCOMdes;

            bool readyToTransition = false;
      } updated;

      double tau_DS;
      ComStatesRelToStance com_rel_to_below_ankle;
      ComStatesRelToStance com_rel_to_ankle;

      // todo can be moved to based with input stance & contact status
      void updateCOMstates();

      std::unique_ptr<FlatFootFPPlanner> ROplanner;
};

} // namespace romoco   
#endif // ROMOCO_WALKING_OUTPUT_FP_HPP