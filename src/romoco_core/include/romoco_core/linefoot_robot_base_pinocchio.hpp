#ifndef BIPED_CORE_LINEFOOT_ROBOT_BASE_PINOCCHIO_HPP_
#define BIPED_CORE_LINEFOOT_ROBOT_BASE_PINOCCHIO_HPP_

#include "romoco_core/robot_base_pinocchio.hpp"

namespace romoco
{
   namespace robot
   {
      class LineFootRobotBasePinocchio : public RobotBasePinocchio
      {
      public:
         explicit LineFootRobotBasePinocchio(const std::string &urdf_path, const std::vector<std::string> &locked_encoder_names, const Eigen::VectorXd &locked_joints_q);
         explicit LineFootRobotBasePinocchio(const std::string &config_folder);
         virtual ~LineFootRobotBasePinocchio() override = default;

         Kinematics3D left_toe_kinematics() const override { return left_footF_.kinematics; }
         Kinematics3D right_toe_kinematics() const override { return right_footF_.kinematics; }
         Kinematics3D left_heel_kinematics() const override { return left_footB_.kinematics; }
         Kinematics3D right_heel_kinematics() const override { return right_footB_.kinematics; }

         RobotType robot_type() const override { return RobotType::LineFoot; }

         void GetContactHolonomicConstraints(const FootContactStatus leftC, const FootContactStatus rightC, Eigen::MatrixXd &Jh, Eigen::VectorXd &dJhdq, const Eigen::Matrix3d Rground) override;
         void GetFrictionCone(const FrictionParams fric_params, const FootContactStatus leftC, const FootContactStatus rightC, Eigen::MatrixXd &Acone, Eigen::VectorXd &bcone) override;

      protected:
         // all frame kinematics needed so far
         FrameKinematics3D left_footF_, left_footB_;
         FrameKinematics3D right_footF_, right_footB_;
         // Add the model specific functions here
         std::vector<std::reference_wrapper<FrameKinematics3D>> GetAllFrameKinematics() override;

         std::vector<std::pair<std::reference_wrapper<RobotBasePinocchio::FrameKinematics3D>, std::string>> GetFrameIds() override;

      private:
         void GetHolonomicConstraintsSingleFoot(const FootContactStatus con, const Kinematics3D &F, const Kinematics3D &B, Eigen::MatrixXd &Jh, Eigen::VectorXd &dJhdq);
         void GetFrictionConeSingleFoot(const FrictionParams fric_params, const FootContactStatus con, Eigen::MatrixXd &Acone, Eigen::VectorXd &bcone);
      };
   } // namespace robot
} // namespace romoco

#endif // BIPED_CORE_LINEFOOT_ROBOT_BASE_PINOCCHIO_HPP_