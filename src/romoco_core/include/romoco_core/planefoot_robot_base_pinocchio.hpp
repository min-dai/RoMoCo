#ifndef BIPED_CORE_PLANEFOOT_ROBOT_BASE_PINOCCHIO_HPP_
#define BIPED_CORE_PLANEFOOT_ROBOT_BASE_PINOCCHIO_HPP_

#include "romoco_core/robot_base_pinocchio.hpp"
namespace romoco
{
   namespace robot
   {
      /**
       * @class PlaneFootRobotBasePinocchio
       * @brief A Pinocchio-based robot model for biped robots with plane feet.
       * @ingroup group_robot 
       * This class extends the RobotBasePinocchio class to represent a biped robot with plane feet.
       * It provides specific implementations for kinematics, contact constraints, and friction cones
       * relevant to plane-footed robots.
       */
      class PlaneFootRobotBasePinocchio : public RobotBasePinocchio
      {
      public:
         PlaneFootRobotBasePinocchio(const std::string &urdf_path, const std::vector<std::string> &locked_encoder_names, const Eigen::VectorXd &locked_joints_q);
         PlaneFootRobotBasePinocchio(const std::string &config_folder);
         virtual ~PlaneFootRobotBasePinocchio() override = default;

         Kinematics3D left_toe_kinematics() const override { return (left_footLF_.kinematics + left_footRF_.kinematics) / 2.; }
         Kinematics3D right_toe_kinematics() const override { return (right_footLF_.kinematics + right_footRF_.kinematics) / 2.; }
         Kinematics3D left_heel_kinematics() const override { return (left_footLB_.kinematics + left_footRB_.kinematics) / 2.; }
         Kinematics3D right_heel_kinematics() const override { return (right_footLB_.kinematics + right_footRB_.kinematics) / 2.; }

         RobotType robot_type() const override { return RobotType::PlaneFoot; }

         void GetContactHolonomicConstraints(const FootContactStatus leftC, const FootContactStatus rightC, Eigen::MatrixXd &Jh, Eigen::VectorXd &dJhdq, const Eigen::Matrix3d Rground = Eigen::MatrixXd::Identity(3, 3)) override;
         void GetFrictionCone(const FrictionParams fric_params, const FootContactStatus leftC, const FootContactStatus rightC, Eigen::MatrixXd &Acone, Eigen::VectorXd &bcone) override;

      protected:
         // all frame kinematics needed so far
         FrameKinematics3D left_footLF_, left_footRF_, left_footLB_, left_footRB_;
         FrameKinematics3D right_footLF_, right_footRF_, right_footLB_, right_footRB_;

         std::vector<std::pair<std::reference_wrapper<RobotBasePinocchio::FrameKinematics3D>, std::string>> GetFrameIds() override;
         std::vector<std::reference_wrapper<FrameKinematics3D>> GetAllFrameKinematics() override;

      private:
         void GetHolonomicConstraintsSingleFoot(const FootContactStatus con, const Kinematics3D &LF, const Kinematics3D &RF, const Kinematics3D &LB, const Kinematics3D &RB, Eigen::MatrixXd &Jh, Eigen::VectorXd &dJhdq);
         void GetFrictionConeSingleFoot(const FrictionParams fric_params, const FootContactStatus con, Eigen::MatrixXd &Acone, Eigen::VectorXd &bcone);
      };
   } // namespace robot
} // namespace romoco

#endif // BIPED_CORE_PLANEFOOT_ROBOT_BASE_PINOCCHIO_HPP_