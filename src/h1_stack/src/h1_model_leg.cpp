#include "h1_model_leg.hpp"

#include <iostream>
namespace romoco
{
    namespace robot
    {
H1ModelLeg::H1ModelLeg(const std::string &urdf_path, const std::vector<std::string> &locked_encoder_names, const Eigen::VectorXd &locked_joints_q)
    : PlaneFootRobotBasePinocchio(urdf_path, locked_encoder_names, locked_joints_q)
{
    Init();
    
}
H1ModelLeg::H1ModelLeg(const std::string &config_folder)
    : PlaneFootRobotBasePinocchio(config_folder)
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
    AddFramesImpl("root_joint");
}

void H1ModelLeg::InitActuation()
{
    InitActuationImpl(12);

}

void H1ModelLeg::InitJointKinematics()
{
    left_hip_yaw_.joint_id = LeftHipYaw;
    right_hip_yaw_.joint_id = RightHipYaw;
}

void H1ModelLeg::AddFramesImpl(const std::string& base_joint_name)
{
    pinocchio::SE3 placement = pinocchio::SE3::Identity();

    pinocchio::JointIndex left_ankle_joint_id = model_.getJointId("left_ankle_roll_joint");
    pinocchio::JointIndex righ_ankle_joint_id = model_.getJointId("right_ankle_roll_joint");

    double y_one_side = 0.025;
    std::vector<std::tuple<std::string, Eigen::Vector3d, pinocchio::JointIndex>> frame_data = {
        {"left_foot_LF", {0.12, y_one_side, -0.03}, left_ankle_joint_id},
        {"right_foot_LF", {0.12, y_one_side, -0.03}, righ_ankle_joint_id},
        {"left_foot_RF", {0.12, -y_one_side, -0.03}, left_ankle_joint_id},
        {"right_foot_RF", {0.12, -y_one_side, -0.03}, righ_ankle_joint_id},
        {"left_foot_LB", {-0.05, y_one_side, -0.03}, left_ankle_joint_id},
        {"right_foot_LB", {-0.05, y_one_side, -0.03}, righ_ankle_joint_id},
        {"left_foot_RB", {-0.05, -y_one_side, -0.03}, left_ankle_joint_id},
        {"right_foot_RB", {-0.05, -y_one_side, -0.03}, righ_ankle_joint_id},
        {"left_below_ankle", {0.0, 0.0, -0.03}, left_ankle_joint_id},
        {"right_below_ankle", {0.0, 0.0, -0.03}, righ_ankle_joint_id},
        {"left_ankle", {0.0, 0.0, 0.0}, left_ankle_joint_id},
        {"right_ankle", {0.0, 0.0, 0.0}, righ_ankle_joint_id},
        {"left_mid_foot", {0.035, 0.0, -0.03}, left_ankle_joint_id},
        {"right_mid_foot", {0.035, 0.0, -0.03}, righ_ankle_joint_id},
        {"left_hip", {0.0, 0.0, 0.0}, model_.getJointId("left_hip_roll_joint")},
        {"right_hip", {0.0, 0.0, 0.0}, model_.getJointId("right_hip_roll_joint")},
        {"base", {0, 0, 0},        model_.getJointId(base_joint_name)},
        {"baseF", {0.05, 0.0, 0},  model_.getJointId(base_joint_name)},
        {"baseB", {-0.05, 0.0, 0}, model_.getJointId(base_joint_name)},
        {"baseL", {0.0, 0.05, 0},  model_.getJointId(base_joint_name)},
        {"baseR", {0.0, -0.05, 0}, model_.getJointId(base_joint_name)}
    };

    position_params.footMF_x = 0.12;
    position_params.footLF_y = y_one_side;
    position_params.footRF_y = -y_one_side;
    position_params.footMB_x = -0.05;
    position_params.baseF_x = 0.05;
    position_params.baseB_x = -0.05;
    position_params.baseL_y = 0.05;
    position_params.baseR_y = -0.05;
    
    for (const auto &frame : frame_data)
    {
        placement.translation() = std::get<1>(frame);
        model_.addFrame(pinocchio::Frame(std::get<0>(frame), std::get<2>(frame), 0, placement, pinocchio::OP_FRAME));
    }
}

void H1ModelLeg::InitActuationImpl(int nu)
{
    nu_ = nu;

    // set actuateion matrix
    Eigen::VectorXd Btmp = model_.rotorGearRatio.tail(nu_);
    B_.setZero(model_.nv, nu_);
    B_.bottomRows(nu_) = Btmp.asDiagonal();
    u_ub_ = model_.effortLimit.tail(nu_);
    u_lb_ = -model_.effortLimit.tail(nu_);

    computed_torque_ = Eigen::VectorXd::Zero(nu_);
}

void H1ModelLeg::ComputeContactClassifierInput()
{
    Eigen::MatrixXd Jleft = left_below_ankle_.kinematics.jacobian;
    Eigen::MatrixXd Jright = right_below_ankle_.kinematics.jacobian;

    contact_classifier_input_.Jleft_active = Jleft(Eigen::all, {LeftHipPitch, LeftHipRoll, LeftHipYaw, LeftKneePitch, LeftAnklePitch, LeftAnkleRoll});
    contact_classifier_input_.Jright_active = Jright(Eigen::all, {RightHipPitch, RightHipRoll, RightHipYaw, RightKneePitch, RightAnklePitch, RightAnkleRoll});

    if (computed_torque_.size() != nu_)
    {
        throw std::runtime_error("robot class stored computed_torque_ size does not match nu_");
    }
    contact_classifier_input_.torque_left = computed_torque_({0, 1, 2, 3, 4, 5});
    contact_classifier_input_.torque_right = computed_torque_({6, 7, 8, 9, 10, 11});
}

    } // namespace robot
} // namespace romoco


