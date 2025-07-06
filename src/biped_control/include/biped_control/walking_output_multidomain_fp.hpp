

#include "biped_control/walking_output_base.hpp"
#include "biped_utils/PhaseVariable.hpp"

#include "biped_core/com_states_rel_to_stance.hpp"
#include "biped_planner/flatfoot_fp_planner.hpp"

using namespace Eigen;
using std::cout;
using std::endl;

class WalkingOutputMultiDomainFP : public WalkingOutputBase
{
public:
      enum OuptutIndices
      {
            zCOM,
            stanceHipYaw,
            basePitch,
            baseRoll,
            stanceDeltaPitch,
            swingStepx,
            swingStepy,
            swingStepz,
            swingHipYaw,
            swingDeltaPitch,
            swingDeltaRoll
      };

      WalkingOutputMultiDomainFP(const std::string &config_file, std::shared_ptr<RobotBasePinocchio> robot);
      void UpdateOutput(const Eigen::VectorXd &radio, const double &t, const double &t_old) override;

      void updateTargetWalkingRadio(const VectorXd &radio);
      void ComputeActual() override;
      void ComputeDesired();

      void timeBasedDomainContactStatusSwitch(double t);

      Vector2d computeFPwithROmodel();

      void Init(const std::string &config_file);

private:
      int nY;
      PhaseVariable phase;

      Kinematics3D com2stance_zeroyaw;
      Kinematics3D sw2stance_zeroyaw;
      Kinematics1D sw_hipyaw, st_hipyaw;
      Kinematics1D sw_deltapitch, sw_deltaroll;
      Kinematics1D st_deltapitch;

      struct Config : public ConfigBase
      {
            void Init(RobotType robot_type);
            double robot_initial_yaw; // todo, if delta_yaw==0, set to track initial yaw, allow modification of it
      } config;

      bool firstStepInitialized = false;

      struct Updated
      {
            Domain curSagDomain;

            Vector2d PhaseRange;
            double t;
            double dt;

            double target_yaw, delta_yaw;
            double delta_yaw_des, stance_toe_yaw;

            double tOAp, tFAp, tUAp;
            double TOA, TFA, TUA;
            double tNstep0;

            double TSS, TDS;

            VectorXd y0_UA;
            VectorXd y0d_OA;

            VectorXd bezierCOMz; // bezierCOMz = (1-bezierComVertical)*y0 + bezierComVertical*y1

            VectorXd bezierSwingx, bezierSwingy, bezierSwingz;

            VectorXd bezierStanceHipYaw, bezierSwingHipYaw;

            Vector2d planned_footstep;

            bool isDSP;

            double desiredVx, desiredVy;
            double stepWidth;
            double zCOMdes;

            bool readyToTransition = false;
      } updated;

      double tau_DS;

      // todo can be moved to based with input stance & contact status
      void updateCOMstates();

      ComStatesRelToStance com_rel_to_below_ankle;
      ComStatesRelToStance com_rel_to_toe;
      ComStatesRelToStance com_rel_to_heel;

      std::unique_ptr<FlatFootFPPlanner> ROplanner;
};
