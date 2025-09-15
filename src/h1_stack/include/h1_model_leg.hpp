#ifndef H1_MODEL_LEG_HPP
#define H1_MODEL_LEG_HPP

#include "romoco_core/planefoot_robot_base_pinocchio.hpp"
namespace romoco
{
    namespace robot
    {
        /**
         * @brief Pinocchio model for the Unitree H1 robot with 18 degrees of freedom.
         * @ingroup group_h1_examples
         * @details This class defines the kinematic and dynamic model of the Unitree H1 robot
         * using the Pinocchio library. It includes methods for initializing the model,
         * computing contact classifier inputs, and retrieving kinematic information.
         * The model accounts for the robot's specific joint configuration and provides
         * interfaces for leg kinematics and actuation.
         */
        class H1ModelLeg : public PlaneFootRobotBasePinocchio
        {
        public:
            enum JointIndex
            {
                BasePosX = 0,
                BasePosY = 1,
                BasePosZ = 2,
                BaseRotZ = 3,
                BaseRotY = 4,
                BaseRotX = 5,
                LeftHipYaw = 6,
                LeftHipPitch = 7,
                LeftHipRoll = 8,
                LeftKneePitch = 9,
                LeftAnklePitch = 10,
                LeftAnkleRoll = 11,
                RightHipYaw = 12,
                RightHipPitch = 13,
                RightHipRoll = 14,
                RightKneePitch = 15,
                RightAnklePitch = 16,
                RightAnkleRoll = 17
            };

            H1ModelLeg(const std::string &urdf_path, const std::vector<std::string> &locked_encoder_names, const Eigen::VectorXd &locked_joints_q);
            H1ModelLeg(const std::string &config_folder);
            ~H1ModelLeg() override = default;

            std::vector<int> left_leg_encoder_idx() const override
            {
                return {JointIndex::LeftHipPitch, JointIndex::LeftHipRoll, JointIndex::LeftHipYaw, JointIndex::LeftKneePitch, JointIndex::LeftAnklePitch, JointIndex::LeftAnkleRoll};
            }
            std::vector<int> right_leg_encoder_idx() const override
            {
                return {JointIndex::RightHipPitch, JointIndex::RightHipRoll, JointIndex::RightHipYaw, JointIndex::RightKneePitch, JointIndex::RightAnklePitch, JointIndex::RightAnkleRoll};
            }

            void ComputeContactClassifierInput() override;

            std::vector<int> actuated_q_idx(AnkleMotorStatus left_ankle_status, AnkleMotorStatus right_ankle_status) const override;
            std::vector<int> actuated_u_idx(AnkleMotorStatus left_ankle_status, AnkleMotorStatus right_ankle_status) const override;

            void AddFrames() override;

            void InitActuation() override;

            void InitJointKinematics() override;


        protected:
            void AddFramesImpl(const std::string &base_joint_name);
            void InitActuationImpl(int nu);
        };
    } // namespace robot
} // namespace romoco
#endif // H1_MODEL_LEG_HPP