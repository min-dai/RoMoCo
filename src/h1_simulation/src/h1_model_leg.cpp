#include "h1_model_leg.hpp"

H1ModelLeg::H1ModelLeg(const std::string &urdf_path, const std::vector<std::string> &locked_joints_names, const VectorXd &locked_joints_q)
    : PlaneFootRobotBasePinocchio(urdf_path, locked_joints_names, locked_joints_q)
{
    Init();
}

std::vector<int> H1ModelLeg::actuated_q_idx(AnkleMotorStatus left_ankle_status, AnkleMotorStatus right_ankle_status) const
{
    std::vector<int> q_idx;
    auto add_leg_joints = [&](bool is_left, AnkleMotorStatus ankle_status)
    {
        int hip_yaw = is_left ? LeftHipYaw : RightHipYaw;
        int hip_pitch = is_left ? LeftHipPitch : RightHipPitch;
        int hip_roll = is_left ? LeftHipRoll : RightHipRoll;
        int knee = is_left ? LeftKneePitch : RightKneePitch;
        int ankle_pitch = is_left ? LeftAnklePitch : RightAnklePitch;
        int ankle_roll = is_left ? LeftAnkleRoll : RightAnkleRoll;

        q_idx.push_back(hip_yaw);
        q_idx.push_back(hip_pitch);
        q_idx.push_back(hip_roll);
        q_idx.push_back(knee);

        switch (ankle_status)
        {
        case AnkleMotorStatus::PassiveAll:
            break; // do not include ankle joints
        case AnkleMotorStatus::ActivePitch:
            q_idx.push_back(ankle_pitch);
            break;
        case AnkleMotorStatus::ActiveAll:
            q_idx.push_back(ankle_pitch);
            q_idx.push_back(ankle_roll);
            break;
        }
    };

    add_leg_joints(/*left=*/true, left_ankle_status);
    add_leg_joints(/*left=*/false, right_ankle_status);

    return q_idx;
}

std::vector<int> H1ModelLeg::actuated_u_idx(AnkleMotorStatus left_ankle_status, AnkleMotorStatus right_ankle_status) const
{
    std::vector<int> u_idx;
    auto add_leg_joints = [&](bool is_left, AnkleMotorStatus ankle_status)
    {
        int hip_yaw = is_left ? 0 : 6;
        int hip_pitch = is_left ? 1 : 7;
        int hip_roll = is_left ? 2 : 8;
        int knee = is_left ? 3 : 9;
        int ankle_pitch = is_left ? 4 : 10;
        int ankle_roll = is_left ? 5 : 11;
        u_idx.push_back(hip_yaw);
        u_idx.push_back(hip_pitch);
        u_idx.push_back(hip_roll);
        u_idx.push_back(knee);
        switch (ankle_status)
        {
        case AnkleMotorStatus::PassiveAll:
            break; // do not include ankle joints
        case AnkleMotorStatus::ActivePitch:
            u_idx.push_back(ankle_pitch);
            break;
        case AnkleMotorStatus::ActiveAll:
            u_idx.push_back(ankle_pitch);
            u_idx.push_back(ankle_roll);
            break;
        }
    };
    add_leg_joints(/*left=*/true, left_ankle_status);
    add_leg_joints(/*left=*/false, right_ankle_status);
    return u_idx;
}

void H1ModelLeg::AddFrames()
{
    pinocchio::SE3 placement = pinocchio::SE3::Identity();

    pinocchio::JointIndex left_ankle_joint_id = model_.getJointId("left_ankle_roll_joint");
    pinocchio::JointIndex righ_ankle_joint_id = model_.getJointId("right_ankle_roll_joint");

    //TODO: need to figure out foot size
    std::vector<std::tuple<std::string, Eigen::Vector3d, pinocchio::JointIndex>> frame_data = {
        {"left_foot_LF", {0.12, 0.03, -0.03}, left_ankle_joint_id},
        {"right_foot_LF", {0.12, 0.03, -0.03}, righ_ankle_joint_id},
        {"left_foot_RF", {0.12, -0.03, -0.03}, left_ankle_joint_id},
        {"right_foot_RF", {0.12, -0.03, -0.03}, righ_ankle_joint_id},
        {"left_foot_LB", {-0.05, 0.025, -0.03}, left_ankle_joint_id},
        {"right_foot_LB", {-0.05, 0.025, -0.03}, righ_ankle_joint_id},
        {"left_foot_RB", {-0.05, -0.025, -0.03}, left_ankle_joint_id},
        {"right_foot_RB", {-0.05, -0.025, -0.03}, righ_ankle_joint_id},
        {"left_below_ankle", {0.0, 0.0, -0.03}, left_ankle_joint_id},
        {"right_below_ankle", {0.0, 0.0, -0.03}, righ_ankle_joint_id},
        {"left_ankle", {0.0, 0.0, 0.0}, left_ankle_joint_id},
        {"right_ankle", {0.0, 0.0, 0.0}, righ_ankle_joint_id},
        {"left_mid_foot", {0.035, 0.0, -0.03}, left_ankle_joint_id},
        {"right_mid_foot", {0.035, 0.0, -0.03}, righ_ankle_joint_id},
        {"left_hip", {0.0, 0.0, 0.0}, model_.getJointId("left_hip_roll_joint")},
        {"right_hip", {0.0, 0.0, 0.0}, model_.getJointId("right_hip_roll_joint")},
        {"base", {0.0, 0.0, 0.0}, model_.getJointId("root_joint")},
        {"baseF", {0.05, 0.0, 0.0}, model_.getJointId("root_joint")},
        {"baseB", {-0.05, 0.0, 0.0}, model_.getJointId("root_joint")},
        {"baseL", {0.0, 0.05, 0.0}, model_.getJointId("root_joint")},
        {"baseR", {0.0, -0.05, 0.0}, model_.getJointId("root_joint")}
    };

    for (const auto &frame : frame_data)
    {
        placement.translation() = std::get<1>(frame);
        model_.addFrame(pinocchio::Frame(std::get<0>(frame), std::get<2>(frame), 0, placement, pinocchio::OP_FRAME));
    }
}

void H1ModelLeg::InitActuation()
{
    // todo: find a cleaner way to do this
    nu_ = 12;

    // set actuateion matrix
    VectorXd Btmp = model_.rotorGearRatio.tail(nu_);
    B_.setZero(model_.nv, nu_);
    B_.bottomRows(nu_) = Btmp.asDiagonal();
    u_ub_ = model_.effortLimit.tail(nu_);
    u_lb_ = -model_.effortLimit.tail(nu_);

}

void H1ModelLeg::InitJointKinematics()
{
    left_hip_yaw_.joint_id = LeftHipYaw;
    right_hip_yaw_.joint_id = RightHipYaw;
}
//TODO: need to figure out foot size
Kinematics1D H1ModelLeg::GetLeftFootDeltaPitch()
{
    return ((left_footLB_.kinematics.z() + left_footRB_.kinematics.z())/2.- (left_footLF_.kinematics.z() + left_footRF_.kinematics.z())/2.)/0.17;
}

Kinematics1D H1ModelLeg::GetRightFootDeltaPitch()
{
    return ((right_footLB_.kinematics.z() + right_footRB_.kinematics.z())/2.- (right_footLF_.kinematics.z() + right_footRF_.kinematics.z())/2.)/0.17;
}

Kinematics1D H1ModelLeg::GetLeftFootDeltaRoll()
{
    return ((left_footLF_.kinematics.z()+left_footLB_.kinematics.z())/2. - (left_footRF_.kinematics.z() +left_footRB_.kinematics.z() )/2.)/0.06;
}

Kinematics1D H1ModelLeg::GetRightFootDeltaRoll()
{
    return ((right_footLF_.kinematics.z()+right_footLB_.kinematics.z())/2. - (right_footRF_.kinematics.z() +right_footRB_.kinematics.z() )/2.)/0.06;
}

Kinematics1D H1ModelLeg::GetBaseDeltaPitch()
{
    return (baseB_.kinematics.z() - baseF_.kinematics.z())/0.1;
}
Kinematics1D H1ModelLeg::GetBaseDeltaRoll()
{
    return (baseL_.kinematics.z() - baseR_.kinematics.z())/0.1;
}