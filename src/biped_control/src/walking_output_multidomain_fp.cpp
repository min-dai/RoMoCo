#include "biped_control/walking_output_multidomain_fp.hpp"
#include "biped_command/radio_slider_map.hpp"
#include "biped_planner/HLIPplanner.hpp"

WalkingOutputMultiDomainFP::WalkingOutputMultiDomainFP(const std::string &config_file, std::shared_ptr<RobotBasePinocchio> robot)
    : WalkingOutputBase(robot)
{
   config.yaml_parser.Init(config_file);

   if (robot_->robot_type() == RobotType::PlaneFoot)
   {
      nY = 11;
   }
   else if (robot_->robot_type() == RobotType::LineFoot)
   {
      nY = 10;
   }
   else
   {
      throw std::runtime_error("Unsupported robot type");
   }
   SetOutputSize(robot_->nv(), nY);

   // Initialize the walking output
   Init(config_file);
}

void WalkingOutputMultiDomainFP::Init(const std::string &config_file)
{
   config.InitConfigBase(config_file, robot_->robot_type());

   ROplanner = std::make_unique<HLIPPlanner>();

   // update updated struct using config assuming radio is zero
   updateTargetWalkingRadio(VectorXd::Zero(10));

   ROplanner->Init(updated.zCOMdes, updated.TSS, updated.TDS, 0., 0., updated.stepWidth);

   updated.bezierSwingz.resize(7);

   updated.bezierCOMz.resize(config.bezierComVertical.size());

   lowpass_vel_x_des_.reconfigure(config.dt_lowpass, config.velX_dt_cutoff);
   lowpass_vel_y_des_.reconfigure(config.dt_lowpass, config.velY_dt_cutoff);
   updated.tNstep0 = -1;
}

void WalkingOutputMultiDomainFP::timeBasedDomainContactStatusSwitch(double t)
{
   bool transitioned;

   do
   {
      transitioned = false; // Reset transition flag at the start of each loop iteration
      // OA to UA
      if (t >= (updated.tOAp + updated.TOA) && updated.curSagDomain == Domain::domain_OA)
      {
         cout << t << "entering UA phase" << endl;
         transitioned = true; // Mark that a transition occurred

         updated.tUAp = t;
         updated.curSagDomain = Domain::domain_UA;
         updated.isDSP = false;
         updated.readyToTransition = true;

         // update stance leg
         domain.NextStance();
         if (domain.isLeftStance())
         {
            domain.leftC = (robot_->robot_type() == RobotType::PlaneFoot) ? FootContactStatus::ToeLineContact : FootContactStatus::ToePatchContact;
            domain.rightC = FootContactStatus::InAir;
            actuated_q_idx = robot_->actuated_q_idx(AnkleMotorStatus::ActivePitch, AnkleMotorStatus::ActiveAll);
            actuated_u_idx = robot_->actuated_u_idx(AnkleMotorStatus::ActivePitch, AnkleMotorStatus::ActiveAll);
         }
         else
         {
            domain.leftC = FootContactStatus::InAir;
            domain.rightC = (robot_->robot_type() == RobotType::PlaneFoot) ? FootContactStatus::ToeLineContact : FootContactStatus::ToePatchContact;
            actuated_q_idx = robot_->actuated_q_idx(AnkleMotorStatus::ActiveAll, AnkleMotorStatus::ActivePitch);
            actuated_u_idx = robot_->actuated_u_idx(AnkleMotorStatus::ActiveAll, AnkleMotorStatus::ActivePitch);
         }
         active_y_idx = OutputBase::generate_full_y_idx(nY);

         com_rel_to_below_ankle.Reset();
         com_rel_to_toe.Reset();
         com_rel_to_heel.Reset();

         updated.tNstep0 = t;
         updated.PhaseRange << 0, updated.TSS;
         phase.reconfigure(updated.PhaseRange, 1);

         ComputeActual();
         updated.y0_UA = ya;

         cout << "updated.y0_UA" << updated.y0_UA.transpose() << endl;

         updated.stance_toe_yaw = (domain.isLeftStance()) ? robot_->GetLeftToeYaw() : robot_->GetRightToeYaw();
         updated.delta_yaw = std::clamp(updated.delta_yaw_des, -0.2, 0.2);
         cout << "updated.delta_yaw_des" << updated.delta_yaw_des << endl;
         updated.delta_yaw_des = 0; // reset for next step
         updated.target_yaw = updated.stance_toe_yaw - updated.delta_yaw;
         cout << "delta: " << updated.delta_yaw << ", stance_toe: " << updated.stance_toe_yaw << ", target: " << updated.target_yaw << endl;

         // update bezier polynomials that only need to update once
         updated.bezierSwingz << updated.y0_UA(swingStepz), updated.y0_UA(swingStepz), config.zsw_max / 3,
             config.zsw_max, config.zsw_max, config.zsw_max / 2, // config.zsw_neg ,
             config.zsw_neg;
         updated.bezierCOMz = (Eigen::VectorXd::Ones(updated.bezierCOMz.size()) - config.bezierComVertical) * updated.y0_UA(zCOM) + config.bezierComVertical * updated.zCOMdes;
         // update bezier parameters for swing and stance hip yaw
         double ToYaw = updated.delta_yaw / 2.;
         updated.bezierStanceHipYaw = (VectorXd::Ones(config.bezierSwingHorizontal.size()) - config.bezierSwingHorizontal) * updated.y0_UA(stanceHipYaw) + config.bezierSwingHorizontal * ToYaw;
         updated.bezierSwingHipYaw = (VectorXd::Ones(config.bezierSwingHorizontal.size()) - config.bezierSwingHorizontal) * updated.y0_UA(swingHipYaw) - config.bezierSwingHorizontal * ToYaw;

         // Compute Friction Constraints
         ComputeFrictionConstriants(config.fric_params);
      }

      // UA to FA
      if (t >= (updated.tUAp + updated.TUA) && updated.curSagDomain == Domain::domain_UA)
      {
         cout << t << "entering FA phase" << endl;
         transitioned = true; // Mark that a transition occurred
         updated.tFAp = t;
         updated.curSagDomain = Domain::domain_FA;
         updated.isDSP = false;
         updated.readyToTransition = true;
      }

      // UA to OA
      if (t >= (updated.tFAp + updated.TFA) && updated.curSagDomain == Domain::domain_FA)
      {
         updated.curSagDomain = Domain::domain_OA;
         updated.tOAp = updated.t;
         cout << t << "entering OA phase" << endl;
         transitioned = true;
         updated.isDSP = true;
         updated.readyToTransition = true;
         if (updated.TOA > 0)
         { // recompute friction cone only if there is a valid OA phase
            domain.leftC = (robot_->robot_type() == RobotType::PlaneFoot) ? FootContactStatus::FlatPlaneContact : FootContactStatus::FlatLineContact;
            domain.rightC = (robot_->robot_type() == RobotType::PlaneFoot) ? FootContactStatus::FlatPlaneContact : FootContactStatus::FlatLineContact;
            actuated_q_idx = robot_->actuated_q_idx(AnkleMotorStatus::ActivePitch, AnkleMotorStatus::ActivePitch);
            actuated_u_idx = robot_->actuated_u_idx(AnkleMotorStatus::ActivePitch, AnkleMotorStatus::ActivePitch);

            // assume both passive ankle
            ComputeActual();

            updated.y0d_OA = yd;
            active_y_idx = {0, 1, 2, 3};

            // Compute Friction Constraints
            ComputeFrictionConstriants(config.fric_params);
         }
      }

   } while (transitioned);
}

void WalkingOutputMultiDomainFP::updateTargetWalkingRadio(const VectorXd &radio)
{
   lowpass_vel_x_des_.update(config.velXmax * radio(Radio::LV));
   lowpass_vel_y_des_.update(config.velYmax * radio(Radio::LH));
   updated.delta_yaw_des = updated.delta_yaw_des + radio(Radio::RH) * updated.dt;

   updated.desiredVx = lowpass_vel_x_des_.getValue() + config.vx_offset;
   updated.desiredVy = lowpass_vel_y_des_.getValue() + config.vy_offset;

   double Tradio = (config.TDS + config.TSS) + 0.1 * radio(Radio::RS); // walking period
   updated.TSS = Tradio / (config.TDS + config.TSS) * config.TSS;
   updated.TDS = Tradio / (config.TDS + config.TSS) * config.TDS;

   updated.TUA = updated.TSS;
   updated.TFA = 0;
   updated.TOA = updated.TDS;

   updated.zCOMdes = config.znom + 0.05 * radio(Radio::LS); // walking height

   updated.stepWidth = config.stepWidthNominal + 0.2 * radio(Radio::S2);
   updated.stepWidth = std::clamp(updated.stepWidth, 0.15, 0.5); // range of the realizable step Width

   ROplanner->UpdateParams(updated.zCOMdes, updated.TSS, updated.TDS, updated.desiredVx, updated.desiredVy, updated.stepWidth);
}

void WalkingOutputMultiDomainFP::UpdateOutput(const Eigen::VectorXd &radio, const double &t, const double &t_old)
{
   updated.t = t;
   updated.dt = t - t_old;

   if (!firstStepInitialized)
   {
      updated.tOAp = t - updated.TOA - .001;
      updated.curSagDomain = Domain::domain_OA;
      domain.stance = StanceStatus::RightStance;

      timeBasedDomainContactStatusSwitch(t);
      ComputeActual();
      // config.znom = updated.y0_UA(zCOM);
      cout << "first step initialized" << endl;
      firstStepInitialized = true;
   }

   updateTargetWalkingRadio(radio);

   timeBasedDomainContactStatusSwitch(t);
   phase.update(t - updated.tNstep0);

   // bewteen 0-1
   tau_DS = (updated.isDSP && updated.TDS != 0) ? (phase.tau - 1.) * updated.TSS / updated.TDS : NAN;

   ComputeActual();
   updateCOMstates();

   ComputeDesired();

   // Compute Holonomic
   ComputeHolonomicConstraints();
}

void WalkingOutputMultiDomainFP::ComputeDesired()
{
   dyd.setZero();
   d2yd.setZero();

   if (updated.isDSP)
   {
      yd = updated.y0d_OA;
   }
   else
   {
      yd.setZero();

      setBezierDesiredOutputs(updated.bezierCOMz, phase.tau, phase.dtau, zCOM);
      setBezierDesiredOutputs(updated.bezierStanceHipYaw, phase.tau, phase.dtau, stanceHipYaw);
      // base pitch and roll are zero

      Vector2d StepLocal = computeFPwithROmodel();
      updated.planned_footstep = computeFPwithROmodel();
      cout << "planned_footstep: " << updated.planned_footstep.transpose() << endl;

      updated.bezierSwingx = (Eigen::VectorXd::Ones(config.bezierSwingHorizontal.size()) - config.bezierSwingHorizontal) * updated.y0_UA(swingStepx) + config.bezierSwingHorizontal * updated.planned_footstep.x();
      updated.bezierSwingy = (Eigen::VectorXd::Ones(config.bezierSwingHorizontal.size()) - config.bezierSwingHorizontal) * updated.y0_UA(swingStepy) + config.bezierSwingHorizontal * updated.planned_footstep.y();

      setBezierDesiredOutputs(updated.bezierSwingx, phase.tau, phase.dtau, swingStepx);
      setBezierDesiredOutputs(updated.bezierSwingy, phase.tau, phase.dtau, swingStepy);

      setBezierDesiredOutputs(updated.bezierSwingz, phase.tau, phase.dtau, swingStepz);
      setBezierDesiredOutputs(updated.bezierSwingHipYaw, phase.tau, phase.dtau, swingHipYaw);
      // swing delta pitch and roll are zero

      // setBezierDesiredOutputs(updated.bezierSwingz, phase.tau, phase.dtau, StanceToePitch);
      yd(stanceDeltaPitch) = 0.4;
      yd(swingDeltaPitch) = 0.4;
   }
}

Vector2d WalkingOutputMultiDomainFP::computeFPwithROmodel()
{
   // swing x and y
   Vector4d x_now;
   x_now << com_rel_to_toe.states.pCOM.x(), com_rel_to_toe.states.Lpivot.y(), com_rel_to_toe.states.pCOM.y(), com_rel_to_toe.states.Lpivot.x();
   double T2imp = (updated.PhaseRange(1) - phase.pActual);
   Vector2d stepSize = ROplanner->UpdatePlan(x_now, T2imp, domain.stance);

   double ang = updated.target_yaw - robot_->q()(BaseRotZ); // convert from target yaw frame to local frame
   MatrixXd mat = AngleAxis<double>(ang, Vector3d(0, 0, 1)).toRotationMatrix();
   Vector3d StepLocal(stepSize(0), stepSize(1), 0);
   StepLocal = mat * StepLocal;

   StepLocal.y() = (domain.isLeftStance()) ? std::clamp(StepLocal(1), -0.6, -0.1) : std::clamp(StepLocal(1), 0.1, 0.6);

   return StepLocal.head(2);
}

void WalkingOutputMultiDomainFP::ComputeActual()
{
   Matrix3d Rbase = robot_->GetBaseR_yaw();
   if (domain.isLeftStance())
   {
      // todo: I need toe kinematics
      com2stance_zeroyaw = (robot_->com_kinematics() - robot_->left_toe_kinematics()).Rot(Rbase.transpose());
      sw2stance_zeroyaw = (robot_->right_toe_kinematics() - robot_->left_toe_kinematics()).Rot(Rbase.transpose());
      sw_hipyaw = robot_->right_hip_yaw_kinematics();
      st_hipyaw = robot_->left_hip_yaw_kinematics();
      // todo: which frame???
      sw_deltapitch = robot_->GetRightFootDeltaPitch();
      sw_deltaroll = robot_->GetRightFootDeltaRoll(); // 1 or 0

      st_deltapitch = robot_->GetLeftFootDeltaPitch();
   }
   else
   {
      com2stance_zeroyaw = (robot_->com_kinematics() - robot_->right_toe_kinematics()).Rot(Rbase.transpose());
      sw2stance_zeroyaw = (robot_->left_toe_kinematics() - robot_->right_toe_kinematics()).Rot(Rbase.transpose());
      sw_hipyaw = robot_->left_hip_yaw_kinematics();
      st_hipyaw = robot_->right_hip_yaw_kinematics();
      sw_deltapitch = robot_->GetLeftFootDeltaPitch();
      sw_deltaroll = robot_->GetLeftFootDeltaRoll(); // 1 or 0
      st_deltapitch = robot_->GetRightFootDeltaPitch();
   }

   Kinematics1D delta_pitch = robot_->GetBaseDeltaPitch();
   Kinematics1D delta_roll = robot_->GetBaseDeltaRoll();

   ya << com2stance_zeroyaw.position.z(), // 1
       st_hipyaw.position,                // 1
       delta_pitch.position,              // 1
       delta_roll.position,               // 1
       st_deltapitch.position,            // 1
       sw2stance_zeroyaw.position,        // 3
       sw_hipyaw.position,                // 1
       sw_deltapitch.position,            // 1
       sw_deltaroll.position;             // 1 or 0

   dya << com2stance_zeroyaw.velocity.z(), // 1
       st_hipyaw.velocity,                 // 1
       delta_pitch.velocity,               // 1
       delta_roll.velocity,                // 1
       st_deltapitch.velocity,             // 1
       sw2stance_zeroyaw.velocity,         // 3
       sw_hipyaw.velocity,                 // 1
       sw_deltapitch.velocity,             // 1
       sw_deltaroll.velocity;              // 1 or 0

   Jya << com2stance_zeroyaw.jacobian.row(2), // 1
       st_hipyaw.jacobian,                    // 1
       delta_pitch.jacobian,                  // 1
       delta_roll.jacobian,                   // 1
       st_deltapitch.jacobian,                // 1
       sw2stance_zeroyaw.jacobian,            // 3
       sw_hipyaw.jacobian,                    // 1
       sw_deltapitch.jacobian,                // 1
       sw_deltaroll.jacobian;                 // 1 or 0

   dJyadq << com2stance_zeroyaw.dJdq.row(2), // 1
       st_hipyaw.dJdq,                       // 1
       delta_pitch.dJdq,                     // 1
       delta_roll.dJdq,                      // 1
       st_deltapitch.dJdq,                   // 1
       sw2stance_zeroyaw.dJdq,               // 3
       sw_hipyaw.dJdq,                       // 1
       sw_deltapitch.dJdq,                   // 1
       sw_deltaroll.dJdq;                    // 1 or 0
}

void WalkingOutputMultiDomainFP::updateCOMstates() // in target yaw frame
{
   Kinematics3D p_toe = (domain.isLeftStance()) ? robot_->left_toe_kinematics() : robot_->right_toe_kinematics();
   Vector3d Lcom = robot_->ComputeCentroidalAngularMomentum() / robot_->mass();

   com_rel_to_toe.compute(robot_->com_kinematics().position, robot_->com_kinematics().velocity, p_toe.position, Lcom, updated.target_yaw, updated.dt);
}