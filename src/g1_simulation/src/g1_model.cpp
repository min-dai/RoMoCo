#include "g1_model.hpp"

G1Model::G1Model(const std::string &urdf_path, const std::vector<std::string> &locked_joints_names, const VectorXd &locked_joints_q)
    : G1ModelLeg(urdf_path, locked_joints_names, locked_joints_q)
{
    Init();
}

std::vector<int> G1Model::actuated_q_idx(AnkleMotorStatus left_ankle_status, AnkleMotorStatus right_ankle_status) const
{
    std::vector<int> q_idx = G1ModelLeg::actuated_q_idx(left_ankle_status, right_ankle_status);
    
    
    auto add_upper_joints = [&](bool is_left)
    {
        int shoulder_pitch = is_left ? LeftShoulderPitch : RightShoulderPitch;
        int shoulder_roll = is_left ? LeftShoulderRoll : RightShoulderRoll;
        int shoulder_yaw = is_left ? LeftShoulderYaw : RightShoulderYaw;
        int elbow = is_left ? LeftElbowPitch : RightElbowPitch;

        q_idx.push_back(shoulder_pitch);
        q_idx.push_back(shoulder_roll);
        q_idx.push_back(shoulder_yaw);
        q_idx.push_back(elbow);
    };

    // add waist joints
    q_idx.push_back(WaistYaw);
    q_idx.push_back(WaistRoll);
    q_idx.push_back(WaistPitch);
    // add shoulder joints
    add_upper_joints(/*left=*/true);
    add_upper_joints(/*left=*/false);


    return q_idx;
}

std::vector<int> G1Model::actuated_u_idx(AnkleMotorStatus left_ankle_status, AnkleMotorStatus right_ankle_status) const
{
    std::vector<int> u_idx = G1ModelLeg::actuated_u_idx(left_ankle_status, right_ankle_status);
    auto add_upper_joints = [&](bool is_left)
    {
        int shoulder_pitch = is_left ? LeftShoulderPitch-6 : RightShoulderPitch-6;
        int shoulder_roll = is_left ? LeftShoulderRoll-6 : RightShoulderRoll-6;
        int shoulder_yaw = is_left ? LeftShoulderYaw-6 : RightShoulderYaw-6;
        int elbow = is_left ? LeftElbowPitch-6 : RightElbowPitch-6;

        u_idx.push_back(shoulder_pitch);
        u_idx.push_back(shoulder_roll);
        u_idx.push_back(shoulder_yaw);
        u_idx.push_back(elbow);
    };

    u_idx.push_back(WaistYaw-6);
    u_idx.push_back(WaistRoll-6);
    u_idx.push_back(WaistPitch-6);
    add_upper_joints(/*left=*/true);
    add_upper_joints(/*left=*/false);

    return u_idx;
}

void G1Model::AddFrames()
{
    AddFramesImpl("waist_pitch_joint");
    // AddFramesImpl("root_joint");

}

void G1Model::InitActuation()
{
    InitActuationImpl(12 + 11);
}

