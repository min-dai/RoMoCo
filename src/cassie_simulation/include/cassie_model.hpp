#pragma once

#include "biped_core/linefoot_robot_base_pinocchio.hpp"

class CassieModel : public LineFootRobotBasePinocchio
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
        LeftHipRoll = 6,
        LeftHipYaw = 7,
        LeftHipPitch = 8,
        LeftKneePitch = 9,
        LeftTarsusPitch = 10,
        LeftFootPitch = 11,
        RightHipRoll = 12,
        RightHipYaw = 13,
        RightHipPitch = 14,
        RightKneePitch = 15,
        RightTarsusPitch = 16,
        RightFootPitch = 17,
    };
    FrameKinematics3D left_thigh_connector, right_thigh_connector;
    FrameKinematics3D left_heel_spring_end, right_heel_spring_end;

    Kinematics1D left_achilles, right_achilles;

    CassieModel(const std::string &urdf_path, const std::vector<std::string> &locked_joints_names = {});

    std::vector<int> actuated_q_idx(AnkleMotorStatus left_ankle_status, AnkleMotorStatus right_ankle_status) const override;
    std::vector<int> actuated_u_idx(AnkleMotorStatus left_ankle_status, AnkleMotorStatus right_ankle_status) const override;

    void UpdateDynamics(const Eigen::VectorXd &q, const Eigen::VectorXd &dq) override;
    void AddFrames() override;

    void InitActuation() override;

    void InitJointKinematics() override;

    void GetInternalHolonomicConstraints(MatrixXd &Jh, VectorXd &dJhdq) override;

    Kinematics1D GetLeftFootDeltaPitch() override;
    Kinematics1D GetRightFootDeltaPitch() override;
    Kinematics1D GetBaseDeltaPitch() override;
    Kinematics1D GetBaseDeltaRoll() override;

private:
    // Add the model specific functions here
    std::vector<std::reference_wrapper<FrameKinematics3D>> GetAllFrameKinematics() override;
    std::vector<std::pair<std::reference_wrapper<RobotBasePinocchio::FrameKinematics3D>, std::string>> GetFrameIds() override;
};