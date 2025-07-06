#include "biped_core/planefoot_robot_base_pinocchio.hpp"

PlaneFootRobotBasePinocchio::PlaneFootRobotBasePinocchio(const std::string &urdf_path, const std::vector<std::string> &locked_joints_names, const VectorXd &locked_joints_q)
    : RobotBasePinocchio(urdf_path, locked_joints_names, locked_joints_q)
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

void PlaneFootRobotBasePinocchio::GetHolonomicConstraintsSingleFoot(const FootContactStatus con, const Kinematics3D &LF, const Kinematics3D &RF, const Kinematics3D &LB, const Kinematics3D &RB, MatrixXd &Jh, VectorXd &dJhdq)
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

void PlaneFootRobotBasePinocchio::GetContactHolonomicConstraints(const FootContactStatus leftC, const FootContactStatus rightC, MatrixXd &Jh, VectorXd &dJhdq, const Matrix3d Rground)
{
   // Holonomic constraints should corresponds to ground plane frame but rotated to local foot yaw
   MatrixXd Jh_left, Jh_right;
   VectorXd dJhdq_left, dJhdq_right;

   Matrix3d Rleft_fromeul = GetLeftToeRyaw();
   Matrix3d Rleft = Rground * Rleft_fromeul;
   GetHolonomicConstraintsSingleFoot(leftC, left_footLF_.kinematics.Rot(Rleft.transpose()), left_footRF_.kinematics.Rot(Rleft.transpose()), left_footLB_.kinematics.Rot(Rleft.transpose()), left_footRB_.kinematics.Rot(Rleft.transpose()), Jh_left, dJhdq_left);

   Matrix3d Rright_fromeul = GetRightToeRyaw();
   Matrix3d Rright = Rground * Rright_fromeul;
   GetHolonomicConstraintsSingleFoot(rightC, right_footLF_.kinematics.Rot(Rright.transpose()), right_footRF_.kinematics.Rot(Rright.transpose()), right_footLB_.kinematics.Rot(Rright.transpose()), right_footRB_.kinematics.Rot(Rright.transpose()), Jh_right, dJhdq_right);

   Jh.resize(Jh_left.rows() + Jh_right.rows(), nv());
   Jh << Jh_left, Jh_right;
   dJhdq.resize(Jh_left.rows() + Jh_right.rows());
   dJhdq << dJhdq_left, dJhdq_right;
}

void PlaneFootRobotBasePinocchio::GetFrictionConeSingleFoot(const FrictionParams fric_params, const FootContactStatus con, MatrixXd &Acone, VectorXd &bcone)
{
   double mu = fric_params.frictionCoef;
   double nu = fric_params.Rot_frictionCoef;
   double l1 = fric_params.Lfront;
   double l2 = fric_params.Lback;
   double s = fric_params.W;

   // nu = 2/3.*mu*(l1+l2) / 2.;
   if (con == FootContactStatus::InAir)
   {
      Acone.resize(0, 0);
      bcone.resize(0);
   }
   else if (con == FootContactStatus::FlatPlaneContact)
   {
      // int nCone = 11;
      // int nM = 6;
      // MatrixXd Acone_raw = MatrixXd::Zero(nCone, nM);
      // Acone_raw << 0, 0, -1, 0, 0, 0,
      //     1, 0, -mu / sqrt(2.), 0, 0, 0,
      //     -1, 0, -mu / sqrt(2.), 0, 0, 0,
      //     0, 1, -mu / sqrt(2.), 0, 0, 0,
      //     0, -1, -mu / sqrt(2.), 0, 0, 0,
      //     0, 0, -s, 1, 0, 0,
      //     0, 0, -s, -1, 0, 0,
      //     0, 0, -l1, 0, 1, 0,
      //     0, 0, -l2, 0, -1, 0,
      //     0, 0, -nu, 0, 0, 1,
      //     0, 0, -nu, 0, 0, -1;

      int nCone = 9;
      int nM = 6;
      MatrixXd Acone_raw = MatrixXd::Zero(nCone, nM);
      Acone_raw << 0, 0, -1, 0, 0, 0,
          1, 0, -mu / sqrt(2.), 0, 0, 0,
          -1, 0, -mu / sqrt(2.), 0, 0, 0,
          0, 1, -mu / sqrt(2.), 0, 0, 0,
          0, -1, -mu / sqrt(2.), 0, 0, 0,
          0, 0, -s, 1, 0, 0,
          0, 0, -s, -1, 0, 0,
          0, 0, -l1, 0, 1, 0,
          0, 0, -l2, 0, -1, 0;

      MatrixXd H(nM, nM);
      H << 1, 0, 0, 1, 0, 0,
          0, 1, 0, 0, 0, 0,
          0, 0, 1, 0, 1, 1,
          0, 0, s, 0, -s, 0,
          0, 0, -l1, 0, -l1, l2,
          -s, l1, 0, s, 0, 0;
      Acone.resize(nCone, nM);
      Acone = Acone_raw * H;

      bcone.resize(nCone);
      bcone = VectorXd::Zero(nCone);
      bcone(0) = -fric_params.Fz_lb;
   }
   else if (con == FootContactStatus::ToeLineContact || con == FootContactStatus::HeelLineContact)
   {
      int nCone = 9;
      int nM = 5;
      MatrixXd Acone_raw = MatrixXd::Zero(nCone, nM);
      Acone_raw << 0, 0, -1, 0, 0,
          1, 0, -mu / sqrt(2.), 0, 0,
          -1, 0, -mu / sqrt(2.), 0, 0,
          0, 1, -mu / sqrt(2.), 0, 0,
          0, -1, -mu / sqrt(2.), 0, 0,
          0, 0, -s, 1, 0,
          0, 0, -s, -1, 0,
          0, 0, -nu, 0, 1,
          0, 0, -nu, 0, -1;

      MatrixXd H(nM, nM);
      H << 1, 0, 0, 1, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 1,
          0, 0, s, 0, -s,
          -s, 0, 0, s, 0;

      Acone.resize(nCone, nM);
      Acone = Acone_raw * H;

      bcone.resize(nCone);
      bcone = VectorXd::Zero(nCone);
      bcone(0) = -fric_params.Fz_lb;
   }
   else
   {
      std::cerr << "Friction cone not implemented" << std::endl;
   }
}

void PlaneFootRobotBasePinocchio::GetFrictionCone(const FrictionParams fric_params, const FootContactStatus leftC, const FootContactStatus rightC, MatrixXd &Acone, VectorXd &bcone)
{
   MatrixXd Acone_left, Acone_right;
   VectorXd bcone_left, bcone_right;
   GetFrictionConeSingleFoot(fric_params, leftC, Acone_left, bcone_left);
   GetFrictionConeSingleFoot(fric_params, rightC, Acone_right, bcone_right);
   Acone = MatrixXd::Zero(Acone_left.rows() + Acone_right.rows(), Acone_left.cols() + Acone_right.cols());
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
