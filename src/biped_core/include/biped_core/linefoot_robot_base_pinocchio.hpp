#ifndef BIPED_CORE_LINEFOOT_ROBOT_BASE_PINOCCHIO_HPP_
#define BIPED_CORE_LINEFOOT_ROBOT_BASE_PINOCCHIO_HPP_

#include "biped_core/robot_base_pinocchio.hpp"

class LineFootRobotBasePinocchio : public RobotBasePinocchio
{
public:
   LineFootRobotBasePinocchio(const std::string &urdf_path, const std::vector<std::string> &locked_joints_names = {}, const VectorXd &locked_joints_q = VectorXd::Zero(0));
   ~LineFootRobotBasePinocchio() override = default;

   Kinematics3D left_toe_kinematics() const override { return left_footF_.kinematics; }
   Kinematics3D right_toe_kinematics() const override { return right_footF_.kinematics; }
   Kinematics3D left_heel_kinematics() const override { return left_footB_.kinematics; }
   Kinematics3D right_heel_kinematics() const override { return right_footB_.kinematics; }

   RobotType robot_type() const override { return RobotType::LineFoot; }

   void GetContactHolonomicConstraints(const FootContactStatus leftC, const FootContactStatus rightC, MatrixXd &Jh, VectorXd &dJhdq, const Matrix3d Rground) override;
   void GetFrictionCone(const FrictionParams fric_params, const FootContactStatus leftC, const FootContactStatus rightC, MatrixXd &Acone, VectorXd &bcone) override;

protected:
   // all frame kinematics needed so far
   FrameKinematics3D left_footF_, left_footB_;
   FrameKinematics3D right_footF_, right_footB_;
   // Add the model specific functions here
   std::vector<std::reference_wrapper<FrameKinematics3D>> GetAllFrameKinematics() override;

   std::vector<std::pair<std::reference_wrapper<RobotBasePinocchio::FrameKinematics3D>, std::string>> GetFrameIds() override;

private:
   void GetHolonomicConstraintsSingleFoot(const FootContactStatus con, const Kinematics3D &F, const Kinematics3D &B, MatrixXd &Jh, VectorXd &dJhdq);
   void GetFrictionConeSingleFoot(const FrictionParams fric_params, const FootContactStatus con, MatrixXd &Acone, VectorXd &bcone);
};

#endif // BIPED_CORE_LINEFOOT_ROBOT_BASE_PINOCCHIO_HPP_