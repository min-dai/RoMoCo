#include "cassie_model.hpp"
using Eigen::Vector3d;

CassieModel::CassieModel(const std::string &urdf_path, const std::vector<std::string> &locked_joints_names)
    : LineFootRobotBasePinocchio(urdf_path, locked_joints_names)
{
    Init();
}

std::vector<int> CassieModel::actuated_q_idx(AnkleMotorStatus left_ankle_status, AnkleMotorStatus right_ankle_status) const
{
    std::vector<int> q_idx;
    auto add_leg_joints = [&](bool is_left, AnkleMotorStatus ankle_status)
    {
        int hip_roll = is_left ? LeftHipRoll : RightHipRoll;
        int hip_yaw = is_left ? LeftHipYaw : RightHipYaw;
        int hip_pitch = is_left ? LeftHipPitch : RightHipPitch;
        int knee = is_left ? LeftKneePitch : RightKneePitch;
        int ankle_pitch = is_left ? LeftFootPitch : RightFootPitch;

        q_idx.push_back(hip_roll);
        q_idx.push_back(hip_yaw);
        q_idx.push_back(hip_pitch);
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
            break;
        }
    };
    add_leg_joints(/*left=*/true, left_ankle_status);
    add_leg_joints(/*left=*/false, right_ankle_status);
    return q_idx;
}

std::vector<int> CassieModel::actuated_u_idx(AnkleMotorStatus left_ankle_status, AnkleMotorStatus right_ankle_status) const
{
    std::vector<int> u_idx;
    auto add_leg_joints = [&](bool is_left, AnkleMotorStatus ankle_status)
    {
        int hip_roll = is_left ? 0 : 5;
        int hip_yaw = is_left ? 1 : 6;
        int hip_pitch = is_left ? 2 : 7;
        int knee = is_left ? 3 : 8;
        int ankle_pitch = is_left ? 4 : 9;


        u_idx.push_back(hip_roll);
        u_idx.push_back(hip_yaw);
        u_idx.push_back(hip_pitch);
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
            break;
        }
    };
    add_leg_joints(/*left=*/true, left_ankle_status);
    add_leg_joints(/*left=*/false, right_ankle_status);
    return u_idx;
}

void CassieModel::AddFrames()
{
    pinocchio::SE3 placement = pinocchio::SE3::Identity();

    
    std::vector<std::pair<std::string, pinocchio::JointIndex>> frame_data = {
        {"left_hip", model_.getJointId("LeftHipYaw")},
        {"right_hip", model_.getJointId("RightHipYaw")},
        {"base", model_.getJointId("root_joint")}};
        

    for (const auto &frame : frame_data)
    {
        model_.addFrame(pinocchio::Frame(std::get<0>(frame), std::get<1>(frame), 0, placement, pinocchio::OP_FRAME));
    }

    Matrix3d Rfoot, Rtoworld;
    Rfoot << cos(50/180.*M_PI), -sin(50/180.*M_PI), 0,
    sin(50/180.*M_PI), cos(50/180.*M_PI), 0,
    0, 0, 1;
    Rtoworld << 0, 0, -1,
                1, 0, 0,
                0, -1, 0;

    placement.rotation() << Rfoot * Rtoworld;

    pinocchio::JointIndex left_ankle_joint_id = model_.getJointId("LeftToePitch");
    pinocchio::JointIndex righ_ankle_joint_id = model_.getJointId("RightToePitch");

    std::string base_joint_name = "root_joint";

    std::vector<std::tuple<std::string, Eigen::Vector3d, pinocchio::JointIndex>> foot_frame_data = {
        {"left_foot_F", {-0.0399655505597651,0.107943294226436,0}, left_ankle_joint_id},
        {"left_foot_B", {0.0826015603392714,0.00509727667658939,0}, left_ankle_joint_id},
        {"left_below_ankle", {0.0366388937521327,0.0436645332577817,0}, left_ankle_joint_id},
        {"left_mid_foot", {0.0213180048897532,0.0565202854515125,0}, left_ankle_joint_id},
        {"left_ankle", {0, 0, 0}, left_ankle_joint_id},
        {"right_foot_F", {-0.0399655505597651,0.107943294226436,0}, righ_ankle_joint_id},
        {"right_foot_B", {0.0826015603392714,0.00509727667658939,0}, righ_ankle_joint_id},
        {"right_below_ankle", {0.0366388937521327,0.0436645332577817,0}, righ_ankle_joint_id},
        {"right_mid_foot", {0.0213180048897532,0.0565202854515125,0}, righ_ankle_joint_id},
        {"right_ankle", {0, 0, 0}, righ_ankle_joint_id},
        {"base", {0, 0, 0},        model_.getJointId(base_joint_name)},
        {"baseF", {0.05, 0.0,  0},  model_.getJointId(base_joint_name)},
        {"baseB", {-0.05, 0.0, 0}, model_.getJointId(base_joint_name)},
        {"baseL", {0.0, 0.05,  0},  model_.getJointId(base_joint_name)},
        {"baseR", {0.0, -0.05, 0}, model_.getJointId(base_joint_name)}
    };
    for (const auto &frame : foot_frame_data)
    {
        placement.translation() = std::get<1>(frame);
        model_.addFrame(pinocchio::Frame(std::get<0>(frame), std::get<2>(frame), 0, placement, pinocchio::OP_FRAME));
    }
    placement = pinocchio::SE3::Identity();
    placement.translation() = Eigen::Vector3d(0.0, 0.0, -0.045);
    model_.addFrame(pinocchio::Frame("left_thigh_connector", model_.getJointId("LeftHipPitch"), 0, placement, pinocchio::OP_FRAME));
    placement.translation() = Eigen::Vector3d(0.0, 0.0, 0.045);
    model_.addFrame(pinocchio::Frame("right_thigh_connector", model_.getJointId("RightHipPitch"), 0, placement, pinocchio::OP_FRAME));

    Vector3d p_tarsus2Heel;
    p_tarsus2Heel << -12.69e-3, -30.59e-3, 0.92e-3;

    MatrixXd R_tarsus2Heel(3, 3);
    R_tarsus2Heel << -0.9121266047, -0.4098716742, 0.005501613619,
                      0.4082402433, -0.9095420663, -0.07793031078,
                      0.03694537597, -0.06883632969, 0.9969436288;

    Vector3d rodJoint2heelJoint;
    rodJoint2heelJoint << 0.11877, -0.01, 0;
    Vector3d pHeelRodJoint = p_tarsus2Heel + R_tarsus2Heel * rodJoint2heelJoint;
    placement.translation() = pHeelRodJoint;
    model_.addFrame(pinocchio::Frame("left_heel_spring_end", model_.getJointId("LeftTarsusPitch"), 0, placement, pinocchio::OP_FRAME));
    placement.translation() << pHeelRodJoint(0), pHeelRodJoint(1), -pHeelRodJoint(2);
    model_.addFrame(pinocchio::Frame("right_heel_spring_end", model_.getJointId("RightTarsusPitch"), 0, placement, pinocchio::OP_FRAME));



    
}

void CassieModel::InitActuation()
{
    nu_ = 10;

    // set actuateion matrix
    B_.setZero(nv(), nu_);
    B_(LeftHipRoll, 0) = 1.;
    B_(LeftHipYaw, 1) = 1.;
    B_(LeftHipPitch, 2) = 1.;
    B_(LeftKneePitch, 3) = 1.;
    B_(LeftFootPitch, 4) = 1.;
    B_(RightHipRoll, 5) = 1.;
    B_(RightHipYaw, 6) = 1.;
    B_(RightHipPitch, 7) = 1.;
    B_(RightKneePitch, 8) = 1.;
    B_(RightFootPitch, 9) = 1.;
    u_ub_.resize(nu_);
    u_lb_.resize(nu_);
    u_ub_ << 90, 60, 90, 200, 40, 90, 60, 90, 200, 40;
    u_lb_ = -u_ub_;
}

void CassieModel::InitJointKinematics()
{
    left_hip_yaw_.joint_id = LeftHipYaw;
    right_hip_yaw_.joint_id = RightHipYaw;
}

void CassieModel::GetInternalHolonomicConstraints(MatrixXd &Jh, VectorXd &dJhdq)
{
    Kinematics3D left = left_thigh_connector.kinematics-left_heel_spring_end.kinematics;
    Kinematics3D right = right_thigh_connector.kinematics-right_heel_spring_end.kinematics;
    left_achilles = (left).dot(left) ;
    right_achilles = (right).dot(right);
    Jh.resize(2, nv());
    Jh << left_achilles.jacobian,
        right_achilles.jacobian;
    dJhdq.resize(2);
    dJhdq << left_achilles.dJdq,
        right_achilles.dJdq;


}

void CassieModel::UpdateDynamics(const Eigen::VectorXd &q, const Eigen::VectorXd &dq)
{
    RobotBasePinocchio::UpdateDynamics(q,dq);
    
    // Add reflected rotor inertias
    data_.M(LeftHipRoll, LeftHipRoll) += 6.62e-05 * 25 * 25;
    data_.M(RightHipRoll, RightHipRoll) += 6.62e-05 * 25 * 25;
    data_.M(LeftHipYaw, LeftHipYaw) += 6.62e-05 * 25 * 25;
    data_.M(RightHipYaw, RightHipYaw) += 6.62e-05 * 25 * 25;
    data_.M(LeftHipPitch, LeftHipPitch) += 0.000365 * 16 * 16;
    data_.M(RightHipPitch, RightHipPitch) += 0.000365 * 16 * 16;
    data_.M(LeftKneePitch, LeftKneePitch) += 0.000365 * 16 * 16;
    data_.M(RightKneePitch, RightKneePitch) += 0.000365 * 16 * 16;
    data_.M(LeftFootPitch, LeftFootPitch) += 4.9e-06 * 50 * 50;
    data_.M(RightFootPitch, RightFootPitch) += 4.9e-06 * 50 * 50;

}

std::vector<std::reference_wrapper<RobotBasePinocchio::FrameKinematics3D>> CassieModel::GetAllFrameKinematics()
{
    auto base_kinematics = LineFootRobotBasePinocchio::GetAllFrameKinematics(); // implicit this
    base_kinematics.push_back(left_thigh_connector);
    base_kinematics.push_back(right_thigh_connector);
    base_kinematics.push_back(left_heel_spring_end);
    base_kinematics.push_back(right_heel_spring_end);
    return base_kinematics;
}

std::vector<std::pair<std::reference_wrapper<RobotBasePinocchio::FrameKinematics3D>, std::string>> CassieModel::GetFrameIds()
{
    auto base_ids = LineFootRobotBasePinocchio::GetFrameIds(); // implicit this
    base_ids.insert(base_ids.end(), {{left_thigh_connector, "left_thigh_connector"}, {right_thigh_connector, "right_thigh_connector"}, {left_heel_spring_end, "left_heel_spring_end"}, {right_heel_spring_end, "right_heel_spring_end"}});
    return base_ids;
}

Kinematics1D CassieModel::GetLeftFootDeltaPitch()
{
    return (left_footB_.kinematics.z() - left_footF_.kinematics.z())/0.14;
}

Kinematics1D CassieModel::GetRightFootDeltaPitch()
{
    return (right_footB_.kinematics.z() - right_footF_.kinematics.z())/0.14;
}
Kinematics1D CassieModel::GetBaseDeltaPitch()
{
    // std::cout << "base pitch: " << baseF.kinematics.position << std::endl;
    return (baseB_.kinematics.z() - baseF_.kinematics.z())/0.1;
}
Kinematics1D CassieModel::GetBaseDeltaRoll()
{
    return (baseL_.kinematics.z() - baseR_.kinematics.z())/0.1;
}
