#pragma once

#include "biped_core/robot_base_pinocchio.hpp"

class PlaneFootRobotBasePinocchio : public RobotBasePinocchio
{
public:
   PlaneFootRobotBasePinocchio(const std::string &urdf_path, const std::vector<std::string> &locked_joints_names = {}, const VectorXd &locked_joints_q = VectorXd::Zero(0));

   Kinematics3D left_toe_kinematics() const override { return (left_footLF_.kinematics + left_footRF_.kinematics) / 2.; }
   Kinematics3D right_toe_kinematics() const override { return (right_footLF_.kinematics + right_footRF_.kinematics) / 2.; }
   Kinematics3D left_heel_kinematics() const override { return (left_footLB_.kinematics + left_footRB_.kinematics) / 2.; }
   Kinematics3D right_heel_kinematics() const override { return (right_footLB_.kinematics + right_footRB_.kinematics) / 2.; }

   RobotType robot_type() const override { return RobotType::PlaneFoot; }

   void GetContactHolonomicConstraints(const FootContactStatus leftC, const FootContactStatus rightC, MatrixXd &Jh, VectorXd &dJhdq, const Matrix3d Rground = MatrixXd::Identity(3, 3)) override;
   void GetFrictionCone(const FrictionParams fric_params, const FootContactStatus leftC, const FootContactStatus rightC, MatrixXd &Acone, VectorXd &bcone) override;

protected:
   // all frame kinematics needed so far
   FrameKinematics3D left_footLF_, left_footRF_, left_footLB_, left_footRB_;
   FrameKinematics3D right_footLF_, right_footRF_, right_footLB_, right_footRB_;

   std::vector<std::pair<std::reference_wrapper<RobotBasePinocchio::FrameKinematics3D>, std::string>> GetFrameIds() override;
   std::vector<std::reference_wrapper<FrameKinematics3D>> GetAllFrameKinematics() override;

private:
   void GetHolonomicConstraintsSingleFoot(const FootContactStatus con, const Kinematics3D &LF, const Kinematics3D &RF, const Kinematics3D &LB, const Kinematics3D &RB, MatrixXd &Jh, VectorXd &dJhdq);
   void GetFrictionConeSingleFoot(const FrictionParams fric_params, const FootContactStatus con, MatrixXd &Acone, VectorXd &bcone);
};