#include "romoco_output/standing_output.hpp"

namespace romoco
{
StandingOutput::StandingOutput(const std::string &config_file, std::shared_ptr<romoco::robot::RobotBasePinocchio> robot)
    : OutputBase(robot)
{
    config.yaml_parser.Init(config_file);
    config.Init(robot_->robot_type());

    nY_ = 6;
    if (robot_->robot_type() == RobotType::PlaneFoot)
    {
        contact.leftC = FootContactStatus::FlatPlaneContact;
        contact.rightC = FootContactStatus::FlatPlaneContact;
    }
    else if (robot_->robot_type() == RobotType::LineFoot)
    {
        contact.leftC = FootContactStatus::FlatLineContact;
        contact.rightC = FootContactStatus::FlatLineContact;
    }
    else
    {
        throw std::runtime_error("Unsupported robot type");
    }

    SetOutputSize(robot_->nq(), nY_);
    actuated_q_idx_ = robot_->actuated_q_idx(AnkleMotorStatus::ActiveAll, AnkleMotorStatus::ActiveAll);
    actuated_u_idx_ = robot_->actuated_u_idx(AnkleMotorStatus::ActiveAll, AnkleMotorStatus::ActiveAll);
    active_y_idx_ = OutputBase::generate_full_y_idx(nY_);

    ComputeFrictionConstraints();

    lowpassyd.Reconfigure(config.dt_lowpass, config.yd_lowpass_dt_cutoff, nY_);
}

void StandingOutput::Config::Init(RobotType robot_type)
{
    dt_lowpass = yaml_parser.get_double("pose_command/dt_lowpass");
    stand2step_y_offset = yaml_parser.get_double("pose_command/stand2step_y_offset");
    x_offset = yaml_parser.get_double("pose_command/x_offset");
    x_range = yaml_parser.get_double("pose_command/x_range");
    y_range = yaml_parser.get_double("pose_command/y_range");
    z_lb = yaml_parser.get_double("pose_command/z_lb");
    z_ub = yaml_parser.get_double("pose_command/z_ub");
    pitch_range = yaml_parser.get_double("pose_command/pitch_range");
    roll_range = yaml_parser.get_double("pose_command/roll_range");
    yaw_range = yaml_parser.get_double("pose_command/yaw_range");

    yd_lowpass_dt_cutoff = yaml_parser.get_VectorXd("pose_command/yd_lowpass_dt_cutoff");

    fric_params.frictionCoef = yaml_parser.get_double("friction/frictionCoef");
    fric_params.Lfront = yaml_parser.get_double("friction/Lfront");
    fric_params.Lback = yaml_parser.get_double("friction/Lback");
    if (robot_type == RobotType::PlaneFoot)
    {
        fric_params.W = yaml_parser.get_double("friction/W");
    }
    fric_params.Fz_lb = yaml_parser.get_double("friction/Fz_lb");
    fric_params.Rot_frictionCoef = yaml_parser.get_double("friction/Rot_frictionCoef");
}

void StandingOutput::UpdateOutput(const DesiredCommand &command, const double &t, const double &t_old)
{
    // Add the implementation here
    updated.t = t;
    updated.readyToTransition = false;

    updated.queueTransition = (command.mode == Mode::Walking) ? true : false;

    // Compute Actual
    ComputeActual();

    if (!updated.isInitialized)
    {
        updated.isInitialized = true;
        lowpassyd.Update(ya_);

        yd_ = ya_;
        updated.initial_height = ya_(zCOM);
    }

    // Compute Desired
    ComputeDesired(command);

    if (updated.queueTransition && (abs(ya_(yCOM)) >= abs(config.stand2step_y_offset - 0.01)))
    {
        updated.readyToTransition = true;
    }

    ComputeHolonomicConstraints();
}

// Todo: COM should be in Zero-Yaw local_world_aligned frame!!!
void StandingOutput::ComputeActual()
{
    // calculate COM kinematics ralative to support base in Base-Yaw frame
    // TEST TODO, COM change to base
    com2supportbase = (robot_->com_kinematics() - (robot_->left_mid_foot_kinematics() + robot_->right_mid_foot_kinematics()) / 2).Rot(robot_->GetBaseR_yaw().transpose());

    // get approximate yaw difference between support base and pelvis using cross product
    Kinematics3D vhip = robot_->left_hip_kinematics() - robot_->right_hip_kinematics();
    // vhip = vhip / vhip.position.norm();
    Kinematics3D vfoot = robot_->left_mid_foot_kinematics() - robot_->right_mid_foot_kinematics();
    // vfoot = vfoot / vfoot.position.norm();
    deltaYaw_cross = vfoot.cross(vhip);

    Kinematics1D delta_pitch = robot_->GetBaseDeltaPitch();
    Kinematics1D delta_roll = robot_->GetBaseDeltaRoll();

    ya_ << com2supportbase.position,     // 3
        delta_roll.position,            // 1
        delta_pitch.position,           // 1
        deltaYaw_cross.position.z();    // 1
    dya_ << com2supportbase.velocity,    // 3
        delta_roll.velocity,            // 1
        delta_pitch.velocity,           // 1
        deltaYaw_cross.velocity.z();    // 1
    Jya_ << com2supportbase.jacobian,    // 3
        delta_roll.jacobian,            // 1
        delta_pitch.jacobian,           // 1
        deltaYaw_cross.jacobian.row(2); // 1
    dJyadq_ << com2supportbase.dJdq,     // 3
        delta_roll.dJdq,                // 1
        delta_pitch.dJdq,               // 1
        deltaYaw_cross.dJdq.z();        // 1
}

void StandingOutput::ComputeDesired(const DesiredCommand &command)
{
    double x_d = (updated.queueTransition) ? config.x_offset : config.x_offset + command.values(Channel::X) * config.x_range;
    double y_d = (updated.queueTransition) ? config.stand2step_y_offset : command.values(Channel::Y) * config.y_range;
    // double z_d = 0.5 * command.values(Channel::Z) * (config.z_ub - config.z_lb) + 0.5 * (config.z_ub + config.z_lb);
    double z_d = updated.initial_height + 0.1*command.values(Channel::Z);
    double yaw_d = (updated.queueTransition) ? 0 : command.values(Channel::Yaw) * config.yaw_range;
    double pitch_d = (updated.queueTransition) ? 0 : command.values(Channel::Pitch) * config.pitch_range;
    double roll_d = (updated.queueTransition) ? 0 : command.values(Channel::Roll) * config.roll_range;

    yd_ << x_d, y_d, z_d, roll_d, pitch_d, yaw_d;
    yd_ = lowpassyd.Update(yd_);
    dyd_.setZero();
    d2yd_.setZero();
}

void StandingOutput::ComputeHolonomicConstraints()
{
    Eigen::MatrixXd Jh_internal, Jh_contact;
    Eigen::VectorXd dJhdq_internal, dJhdq_contact;
    robot_->GetInternalHolonomicConstraints(Jh_internal, dJhdq_internal);
    robot_->GetContactHolonomicConstraints(contact.leftC, contact.rightC, Jh_contact, dJhdq_contact);

    Jh_.resize(Jh_internal.rows() + Jh_contact.rows(), robot_->nv());
    Jh_ << Jh_internal, Jh_contact;
    dJhdq_.resize(dJhdq_internal.size() + dJhdq_contact.size());
    dJhdq_ << dJhdq_internal, dJhdq_contact;
}

void StandingOutput::ComputeFrictionConstraints()
{
    robot_->GetFrictionCone(config.fric_params, contact.leftC, contact.rightC, Afric_, bfric_ub_);
}
} // namespace romoco