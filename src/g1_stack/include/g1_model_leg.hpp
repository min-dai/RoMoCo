#ifndef G1_MODEL_LEG_HPP
#define G1_MODEL_LEG_HPP

#include "romoco_core/planefoot_robot_base_pinocchio.hpp"
namespace romoco
{
    namespace robot
    {
        /**
         * @brief Pinocchio model for the Unitree G1 robot with 18 degrees of freedom.
         * @ingroup group_g1_examples
         * @details This class defines the kinematic and dynamic model of the Unitree G1 robot
         * using the Pinocchio library. It includes methods for initializing the model,
         * computing contact classifier inputs, and retrieving kinematic information.
         * The model accounts for the robot's specific joint configuration and provides
         * interfaces for leg kinematics and actuation.
         */
        class G1ModelLeg : public PlaneFootRobotBasePinocchio
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
                LeftHipPitch = 6,
                LeftHipRoll = 7,
                LeftHipYaw = 8,
                LeftKneePitch = 9,
                LeftAnklePitch = 10,
                LeftAnkleRoll = 11,
                RightHipPitch = 12,
                RightHipRoll = 13,
                RightHipYaw = 14,
                RightKneePitch = 15,
                RightAnklePitch = 16,
                RightAnkleRoll = 17
            };

            G1ModelLeg(const std::string &urdf_path, const std::vector<std::string> &locked_encoder_names, const Eigen::VectorXd &locked_joints_q);
            G1ModelLeg(const std::string &config_folder);
            ~G1ModelLeg() override = default;

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

            Kinematics1D GetLeftFootDeltaPitch() override;
            Kinematics1D GetRightFootDeltaPitch() override;
            Kinematics1D GetLeftFootDeltaRoll() override;
            Kinematics1D GetRightFootDeltaRoll() override;
            Kinematics1D GetBaseDeltaPitch() override;
            Kinematics1D GetBaseDeltaRoll() override;

        protected:
            void AddFramesImpl(const std::string &base_joint_name);
            void InitActuationImpl(int nu);
        };
    } // namespace robot
} // namespace romoco
#endif // G1_MODEL_LEG_HPP