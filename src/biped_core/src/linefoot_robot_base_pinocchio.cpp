#include "biped_core/linefoot_robot_base_pinocchio.hpp"

LineFootRobotBasePinocchio::LineFootRobotBasePinocchio(const std::string &urdf_path, const std::vector<std::string> &locked_joints_names, const VectorXd &locked_joints_q)
    : RobotBasePinocchio(urdf_path, locked_joints_names, locked_joints_q)
{
}

std::vector<std::reference_wrapper<RobotBasePinocchio::FrameKinematics3D>> LineFootRobotBasePinocchio::GetAllFrameKinematics()
{
   return {
       left_footF_, right_footF_,
       left_footB_, right_footB_,
       left_below_ankle_, right_below_ankle_,
       left_mid_foot_, right_mid_foot_,
       left_ankle_, right_ankle_,
       left_hip_, right_hip_, base_,
       baseF_, baseB_, baseL_, baseR_};
}



void LineFootRobotBasePinocchio::GetHolonomicConstraintsSingleFoot(const FootContactStatus con, const Kinematics3D &F, const Kinematics3D &B,  MatrixXd &Jh, VectorXd &dJhdq) 
{
   if (con == FootContactStatus::InAir)
   {
      Jh.resize(0, nv());
      dJhdq.setZero(0);
   }
   else if (con == FootContactStatus::FlatLineContact)
   {
      // F_C = [Fx1 Fy1 Fz1 Fy2 Fz2]'
      Jh.resize(5, nv());
      Jh << F.jacobian,
          B.jacobian.row(1),
          B.jacobian.row(2);
      dJhdq.resize(5);
      dJhdq << F.dJdq,
          B.dJdq.row(1),
          B.dJdq.row(2);
   }
   else if (con == FootContactStatus::ToePatchContact)
   {
      // F_C = [Fx1 Fy1 Fz1 Fy2]'
      Jh.resize(4, nv());
      Jh << F.jacobian,
          B.jacobian.row(1);
      dJhdq.resize(4);
      dJhdq << F.dJdq,
          B.dJdq.row(1);
   }
   else if (con == FootContactStatus::HeelPatchContact)
   {
      // F_C = [Fy1 Fx2 Fy2 Fz2]'
      Jh.resize(4, nv());
      Jh << F.jacobian.row(1),
          B.jacobian;
      dJhdq.resize(4);
      dJhdq << F.dJdq.row(1),
          B.dJdq;
   }
   else
   {
      std::cerr << "Holonomic constraints not implemented for this foot contact status" << std::endl;
   }
}

void LineFootRobotBasePinocchio::GetContactHolonomicConstraints(const FootContactStatus leftC, const FootContactStatus rightC, MatrixXd &Jh, VectorXd &dJhdq, const Matrix3d Rground)  
{
   MatrixXd Jh_left, Jh_right;
   VectorXd dJhdq_left, dJhdq_right;

   Matrix3d Rleft_fromeul = GetLeftToeRyaw();
   Matrix3d Rleft = Rground*Rleft_fromeul;
   GetHolonomicConstraintsSingleFoot(leftC, left_footF_.kinematics.Rot(Rleft.transpose()), left_footB_.kinematics.Rot(Rleft.transpose()), Jh_left, dJhdq_left);

   Matrix3d Rright_fromeul = GetRightToeRyaw();
   Matrix3d Rright = Rground*Rright_fromeul;
   GetHolonomicConstraintsSingleFoot(rightC, right_footF_.kinematics.Rot(Rright.transpose()), right_footB_.kinematics.Rot(Rright.transpose()), Jh_right, dJhdq_right);

   
   Jh.resize(Jh_left.rows() + Jh_right.rows(), nv());
   Jh << Jh_left, Jh_right;
   dJhdq.resize(Jh_left.rows() + Jh_right.rows());
   dJhdq << dJhdq_left, dJhdq_right;
}

void LineFootRobotBasePinocchio::GetFrictionConeSingleFoot(const FrictionParams fric_params, const FootContactStatus con, MatrixXd &Acone, VectorXd &bcone) 
{
   double mu = fric_params.frictionCoef;
   double nu = fric_params.Rot_frictionCoef;
   
   double l1 = fric_params.Lfront;
   double l2 = fric_params.Lback;

   nu = 2/3.*mu*(l1+l2) / 2.;
   if (con==FootContactStatus::InAir)
   {
      Acone.resize(0, 0);
      bcone.resize(0);
   }
   else if (con == FootContactStatus::FlatLineContact)
   {
      int nCone = 9;
      int nM = 5;
      MatrixXd Acone_raw = MatrixXd::Zero(nCone, nM);
      Acone_raw << 0, 0, -1, 0, 0, 
          1, 0, -mu / sqrt(2.), 0, 0, 
          -1, 0, -mu / sqrt(2.), 0, 0, 
          0, 1, -mu / sqrt(2.), 0, 0, 
          0, -1, -mu / sqrt(2.), 0, 0, 
          0, 0, -l1, 1, 0,
          0, 0, -l2, -1, 0,
          0, 0, -nu, 0, 1,
          0, 0, -nu, 0, -1;

      MatrixXd H(nM,nM);
      H <<   1, 0, 0, 0, 0,
             0, 1, 0, 1, 0, 
             0, 0, 1, 0, 1, 
             0, 0, -l1 , 0, l2,
             0, l1, 0, -l2, 0;
      
      Acone.resize(nCone, nM);
      Acone = Acone_raw*H;

      bcone = VectorXd::Zero(nCone);
      bcone(0) = -fric_params.Fz_lb;
   }
   else if (con == FootContactStatus::ToePatchContact||con == FootContactStatus::HeelPatchContact)
   {
      int nCone = 7;
      int nM = 4;
      MatrixXd Acone_raw = MatrixXd::Zero(nCone, nM);
      Acone_raw << 0, 0, -1, 0, 
          1, 0, -mu / sqrt(2.), 0, 
          -1, 0, -mu / sqrt(2.), 0, 
          0, 1, -mu / sqrt(2.), 0, 
          0, -1, -mu / sqrt(2.), 0,
          0, 0, -nu,  1,
          0, 0, -nu,  -1;

      MatrixXd H(nM,nM);
      H <<   1, 0, 0, 0,
             0, 1, 0, 1, 
             0, 0, 1, 0, 
             0, l1, 0, -l2;
      
      Acone.resize(nCone, nM);
      Acone = Acone_raw*H;

      bcone = VectorXd::Zero(nCone);
      bcone(0) = -fric_params.Fz_lb;
      //todo heel might be different
   }
   else
   {
      std::cerr << "Friction cone not implemented" << std::endl;
   }
}

void LineFootRobotBasePinocchio::GetFrictionCone(const FrictionParams fric_params, const FootContactStatus leftC, const FootContactStatus rightC, MatrixXd &Acone, VectorXd &bcone) 
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

std::vector<std::pair<std::reference_wrapper<RobotBasePinocchio::FrameKinematics3D>, std::string>> LineFootRobotBasePinocchio::GetFrameIds()
{
return {
      {left_footF_, "left_foot_F"}, {left_footB_, "left_foot_B"}, {left_below_ankle_, "left_below_ankle"}, {left_mid_foot_, "left_mid_foot"},{left_ankle_, "left_ankle"},
      {right_footF_, "right_foot_F"}, {right_footB_, "right_foot_B"}, {right_below_ankle_, "right_below_ankle"}, {right_mid_foot_, "right_mid_foot"},{right_ankle_, "right_ankle"},
      {left_hip_, "left_hip"}, {right_hip_, "right_hip"}, {base_, "base"}, 
      {baseF_, "baseF"}, {baseB_, "baseB"}, {baseL_, "baseL"}, {baseR_, "baseR"}};
}