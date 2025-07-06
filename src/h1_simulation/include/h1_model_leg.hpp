#pragma once

#include "biped_core/planefoot_robot_base_pinocchio.hpp"

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

    H1ModelLeg(const std::string &urdf_path, const std::vector<std::string> &locked_joints_names = {}, const VectorXd &locked_joints_q = VectorXd::Zero(0));
    ~H1ModelLeg() override = default;

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
    

private:

};