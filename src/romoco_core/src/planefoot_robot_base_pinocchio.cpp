#include "romoco_core/planefoot_robot_base_pinocchio.hpp"
namespace romoco
{
   namespace robot
   {
      PlaneFootRobotBasePinocchio::PlaneFootRobotBasePinocchio(const std::string &urdf_path, const std::vector<std::string> &locked_encoder_names, const Eigen::VectorXd &locked_joints_q)
          : RobotBasePinocchio(urdf_path, locked_encoder_names, locked_joints_q)
      {
      }
      PlaneFootRobotBasePinocchio::PlaneFootRobotBasePinocchio(const std::string &config_folder)
          : RobotBasePinocchio(config_folder)
      {
      }

      std::vector<std::reference_wrapper<RobotBasePinocchio::FrameKinematics3D>> PlaneFootRobotBasePinocchio::GetAllFrameKinematics()
      {
         return {
             left_footLF_, right_footLF_, left_footRF_, right_footRF_,
             left_footLB_, right_footLB_, left_footRB_, right_footRB_,
             left_below_ankle_, right_below_ankle_,
             left_mid_foot_, right_mid_foot_,
             left_ankle_, right_ankle_,
             left_hip_, right_hip_, base_,
             baseF_, baseB_, baseL_, baseR_};
      }

      Kinematics1D PlaneFootRobotBasePinocchio::GetLeftFootDeltaPitch()
      {
         return ((left_footLB_.kinematics.z() + left_footRB_.kinematics.z()) / 2. - (left_footLF_.kinematics.z() + left_footRF_.kinematics.z()) / 2.) / (position_params.footMF_x - position_params.footMB_x);
      }

      Kinematics1D PlaneFootRobotBasePinocchio::GetRightFootDeltaPitch()
      {
         return ((right_footLB_.kinematics.z() + right_footRB_.kinematics.z()) / 2. - (right_footLF_.kinematics.z() + right_footRF_.kinematics.z()) / 2.) / (position_params.footMF_x - position_params.footMB_x);
      }

      Kinematics1D PlaneFootRobotBasePinocchio::GetLeftFootDeltaRoll()
      {
         return ((left_footLF_.kinematics.z() + left_footLB_.kinematics.z()) / 2. - (left_footRF_.kinematics.z() + left_footRB_.kinematics.z()) / 2.) / (position_params.footLF_y - position_params.footRF_y);
      }

      Kinematics1D PlaneFootRobotBasePinocchio::GetRightFootDeltaRoll()
      {
         return ((right_footLF_.kinematics.z() + right_footLB_.kinematics.z()) / 2. - (right_footRF_.kinematics.z() + right_footRB_.kinematics.z()) / 2.) / (position_params.footLF_y - position_params.footRF_y);
      }

      Kinematics1D PlaneFootRobotBasePinocchio::GetBaseDeltaPitch()
      {
         return (baseB_.kinematics.z() - baseF_.kinematics.z()) / (position_params.baseF_x - position_params.baseB_x);
      }
      Kinematics1D PlaneFootRobotBasePinocchio::GetBaseDeltaRoll()
      {
         return (baseL_.kinematics.z() - baseR_.kinematics.z()) / (position_params.baseL_y - position_params.baseR_y);
      }

      void PlaneFootRobotBasePinocchio::GetHolonomicConstraintsSingleFoot(const FootContactStatus con, const Kinematics3D &LF, const Kinematics3D &RF, const Kinematics3D &LB, const Kinematics3D &RB, Eigen::MatrixXd &Jh, Eigen::VectorXd &dJhdq)
      {
         // Add the implementation here
         if (con == FootContactStatus::InAir)
         {
            Jh.resize(0, nv());
            dJhdq.setZero(0);
         }
         else if (con == FootContactStatus::FlatPlaneContact)
         {
            // F_C = [Fx1 Fy1 Fz1 Fx2 Fz2 Fz3]'
            Kinematics3D MB;
            MB = (LB + RB) / 2.;
            Jh.resize(6, nv());
            Jh << LF.jacobian,
                RF.jacobian.row(0),
                RF.jacobian.row(2),
                MB.jacobian.row(2);
            dJhdq.resize(6);
            dJhdq << LF.dJdq,
                RF.dJdq.row(0),
                RF.dJdq.row(2),
                MB.dJdq.row(2);
         }
         else if (con == FootContactStatus::ToeLineContact)
         {
            // F_C = [Fx1 Fy1 Fz1 Fx2 Fz2]'
            Jh.resize(5, nv());
            Jh << LF.jacobian,
                RF.jacobian.row(0),
                RF.jacobian.row(2);
            dJhdq.resize(5);
            dJhdq << LF.dJdq,
                RF.dJdq.row(0),
                RF.dJdq.row(2);
         }
         else if (con == FootContactStatus::HeelLineContact)
         {
            // F_C = [Fx1 Fy1 Fz1 Fx2 Fz2]'
            Jh.resize(5, nv());
            Jh << LB.jacobian,
                RB.jacobian.row(0),
                RB.jacobian.row(2);
            dJhdq.resize(5);
            dJhdq << LB.dJdq,
                RB.dJdq.row(0),
                RB.dJdq.row(2);
         }
         else
         {
            std::cerr << "Holonomic constraints not implemented for this foot contact status" << std::endl;
         }
      }

      void PlaneFootRobotBasePinocchio::GetContactHolonomicConstraints(const FootContactStatus leftC, const FootContactStatus rightC, Eigen::MatrixXd &Jh, Eigen::VectorXd &dJhdq, const Eigen::Matrix3d Rground)
      {
         // Holonomic constraints should corresponds to ground plane frame but rotated to local foot yaw
         Eigen::MatrixXd Jh_left, Jh_right;
         Eigen::VectorXd dJhdq_left, dJhdq_right;

         Eigen::Matrix3d Rleft_fromeul = GetLeftToeRyaw();
         Eigen::Matrix3d Rleft = Rground * Rleft_fromeul;
         GetHolonomicConstraintsSingleFoot(leftC, left_footLF_.kinematics.Rot(Rleft.transpose()), left_footRF_.kinematics.Rot(Rleft.transpose()), left_footLB_.kinematics.Rot(Rleft.transpose()), left_footRB_.kinematics.Rot(Rleft.transpose()), Jh_left, dJhdq_left);

         Eigen::Matrix3d Rright_fromeul = GetRightToeRyaw();
         Eigen::Matrix3d Rright = Rground * Rright_fromeul;
         GetHolonomicConstraintsSingleFoot(rightC, right_footLF_.kinematics.Rot(Rright.transpose()), right_footRF_.kinematics.Rot(Rright.transpose()), right_footLB_.kinematics.Rot(Rright.transpose()), right_footRB_.kinematics.Rot(Rright.transpose()), Jh_right, dJhdq_right);

         Jh.resize(Jh_left.rows() + Jh_right.rows(), nv());
         Jh << Jh_left, Jh_right;
         dJhdq.resize(Jh_left.rows() + Jh_right.rows());
         dJhdq << dJhdq_left, dJhdq_right;
      }

      void PlaneFootRobotBasePinocchio::GetFrictionConeSingleFoot(const FrictionParams fric_params, const FootContactStatus con, Eigen::MatrixXd &Acone, Eigen::VectorXd &bcone)
      {
         double mu = fric_params.frictionCoef;
         double nu = fric_params.Rot_frictionCoef;
         double l1 = fric_params.Lfront;
         double l2 = fric_params.Lback;
         double s = fric_params.W;

         // nu = 2/3.*mu*(l1-l2) / 2.;
         if (con == FootContactStatus::InAir)
         {
            Acone.resize(0, 0);
            bcone.resize(0);
         }
         else if (con == FootContactStatus::FlatPlaneContact)
         {
            int nCone = 11;
            int nM = 6;
            Eigen::MatrixXd Acone_raw = Eigen::MatrixXd::Zero(nCone, nM);
            Acone_raw << 0, 0, -1, 0, 0, 0,
                1, 0, -mu / sqrt(2.), 0, 0, 0,
                -1, 0, -mu / sqrt(2.), 0, 0, 0,
                0, 1, -mu / sqrt(2.), 0, 0, 0,
                0, -1, -mu / sqrt(2.), 0, 0, 0,
                0, 0, -s, 1, 0, 0,
                0, 0, -s, -1, 0, 0,
                0, 0, -l1, 0, 1, 0,
                0, 0, l2, 0, -1, 0,
                0, 0, -nu, 0, 0, 1,
                0, 0, -nu, 0, 0, -1;

            double w = (position_params.footLF_y - position_params.footRF_y) / 2.;

            Eigen::MatrixXd H(nM, nM);
            H << 1, 0, 0, 1, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 1, 0, 1, 1,
                0, 0, w, 0, -w, 0,
                0, 0, -position_params.footMF_x, 0, -position_params.footMF_x, -position_params.footMB_x,
                -w, position_params.footMF_x, 0, w, 0, 0;
            Acone.resize(nCone, nM);
            Acone = Acone_raw * H;

            bcone.resize(nCone);
            bcone = Eigen::VectorXd::Zero(nCone);
            bcone(0) = -fric_params.Fz_lb;
         }
         else if (con == FootContactStatus::ToeLineContact || con == FootContactStatus::HeelLineContact)
         {
            int nCone = 9;
            int nM = 5;
            Eigen::MatrixXd Acone_raw = Eigen::MatrixXd::Zero(nCone, nM);
            Acone_raw << 0, 0, -1, 0, 0,
                1, 0, -mu / sqrt(2.), 0, 0,
                -1, 0, -mu / sqrt(2.), 0, 0,
                0, 1, -mu / sqrt(2.), 0, 0,
                0, -1, -mu / sqrt(2.), 0, 0,
                0, 0, -s, 1, 0,
                0, 0, -s, -1, 0,
                0, 0, -nu, 0, 1,
                0, 0, -nu, 0, -1;

            double w = (position_params.footLF_y - position_params.footRF_y) / 2.;
            Eigen::MatrixXd H(nM, nM);
            H << 1, 0, 0, 1, 0,
                0, 1, 0, 0, 0,
                0, 0, 1, 0, 1,
                0, 0, w, 0, -w,
                -w, 0, 0, w, 0;

            Acone.resize(nCone, nM);
            Acone = Acone_raw * H;

            bcone.resize(nCone);
            bcone = Eigen::VectorXd::Zero(nCone);
            bcone(0) = -fric_params.Fz_lb;
         }
         else
         {
            std::cerr << "Friction cone not implemented" << std::endl;
         }
      }

      void PlaneFootRobotBasePinocchio::GetFrictionCone(const FrictionParams fric_params, const FootContactStatus leftC, const FootContactStatus rightC, Eigen::MatrixXd &Acone, Eigen::VectorXd &bcone)
      {
         Eigen::MatrixXd Acone_left, Acone_right;
         Eigen::VectorXd bcone_left, bcone_right;
         GetFrictionConeSingleFoot(fric_params, leftC, Acone_left, bcone_left);
         GetFrictionConeSingleFoot(fric_params, rightC, Acone_right, bcone_right);
         Acone = Eigen::MatrixXd::Zero(Acone_left.rows() + Acone_right.rows(), Acone_left.cols() + Acone_right.cols());
         Acone.topLeftCorner(Acone_left.rows(), Acone_left.cols()) = Acone_left;
         Acone.bottomRightCorner(Acone_right.rows(), Acone_right.cols()) = Acone_right;
         bcone.resize(bcone_left.rows() + bcone_right.rows());
         bcone << bcone_left, bcone_right;
      }

      std::vector<std::pair<std::reference_wrapper<RobotBasePinocchio::FrameKinematics3D>, std::string>> PlaneFootRobotBasePinocchio::GetFrameIds()
      {
         return {
             {left_footLF_, "left_foot_LF"}, {left_footRF_, "left_foot_RF"}, {left_footLB_, "left_foot_LB"}, {left_footRB_, "left_foot_RB"}, {left_below_ankle_, "left_below_ankle"}, {left_mid_foot_, "left_mid_foot"}, {left_ankle_, "left_ankle"}, {right_footLF_, "right_foot_LF"}, {right_footRF_, "right_foot_RF"}, {right_footLB_, "right_foot_LB"}, {right_footRB_, "right_foot_RB"}, {right_below_ankle_, "right_below_ankle"}, {right_mid_foot_, "right_mid_foot"}, {right_ankle_, "right_ankle"}, {left_hip_, "left_hip"}, {right_hip_, "right_hip"}, {base_, "base"}, {baseF_, "baseF"}, {baseB_, "baseB"}, {baseL_, "baseL"}, {baseR_, "baseR"}};
      }

   } // namespace robot
} // namespace romoco