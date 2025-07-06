#include "biped_control/standing_output.hpp"

StandingOutput::StandingOutput(const std::string &config_file, std::shared_ptr<RobotBasePinocchio> robot)
    : OutputBase(robot)
{
    config.yaml_parser.Init(config_file);
    config.Init(robot_->robot_type());

    nY = 6;
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

    SetOutputSize(robot_->nq(), nY);
    actuated_q_idx = robot_->actuated_q_idx(AnkleMotorStatus::ActiveAll, AnkleMotorStatus::ActiveAll);
    actuated_u_idx = robot_->actuated_u_idx(AnkleMotorStatus::ActiveAll, AnkleMotorStatus::ActiveAll);
    active_y_idx = OutputBase::generate_full_y_idx(nY);

    ComputeFrictionConstriants();

    lowpassyd.reconfigure(config.dt_lowpass, config.yd_lowpass_dt_cutoff, nY);
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

void StandingOutput::UpdateOutput(const Eigen::VectorXd &radio, const double &t, const double &t_old)
{
    // Add the implementation here
    updated.t = t;
    updated.readyToTransition = false;

    updated.queueTransition = (radio(Radio::SB) == Radio::Walking) ? true : false;

    // Compute Actual
    ComputeActual();

    if (!updated.isInitialized)
    {
        updated.isInitialized = true;
        lowpassyd.update(ya);

        yd = ya;
        updated.initial_height = ya(zCOM);
    }

    // Compute Desired
    ComputeDesired(radio);

    if (updated.queueTransition && (abs(ya(yCOM)) >= abs(config.stand2step_y_offset - 0.005)))
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

    ya << com2supportbase.position,     // 3
        delta_roll.position,            // 1
        delta_pitch.position,           // 1
        deltaYaw_cross.position.z();    // 1
    dya << com2supportbase.velocity,    // 3
        delta_roll.velocity,            // 1
        delta_pitch.velocity,           // 1
        deltaYaw_cross.velocity.z();    // 1
    Jya << com2supportbase.jacobian,    // 3
        delta_roll.jacobian,            // 1
        delta_pitch.jacobian,           // 1
        deltaYaw_cross.jacobian.row(2); // 1
    dJyadq << com2supportbase.dJdq,     // 3
        delta_roll.dJdq,                // 1
        delta_pitch.dJdq,               // 1
        deltaYaw_cross.dJdq.z();        // 1
}

void StandingOutput::ComputeDesired(const Eigen::VectorXd &radio)
{
    double x_d = (updated.queueTransition) ? config.x_offset : config.x_offset + radio(Radio::LV) * config.x_range;
    double y_d = (updated.queueTransition) ? config.stand2step_y_offset : radio(Radio::LH) * config.y_range;
    double z_d = 0.5 * radio(Radio::LS) * (config.z_ub - config.z_lb) + 0.5 * (config.z_ub + config.z_lb);
    // double z_d = updated.initial_height + 0.1*radio(Radio::LS);
    double yaw_d = (updated.queueTransition) ? 0 : radio(Radio::RS) * config.yaw_range;
    double pitch_d = (updated.queueTransition) ? 0 : radio(Radio::RH) * config.pitch_range;
    double roll_d = (updated.queueTransition) ? 0 : radio(Radio::RV) * config.roll_range;

    yd << x_d, y_d, z_d, yaw_d, pitch_d, roll_d;
    yd = lowpassyd.update(yd);
    dyd.setZero();
    d2yd.setZero();
}

void StandingOutput::ComputeHolonomicConstraints()
{
    MatrixXd Jh_internal, Jh_contact;
    VectorXd dJhdq_internal, dJhdq_contact;
    robot_->GetInternalHolonomicConstraints(Jh_internal, dJhdq_internal);
    //     Matrix3d Rground;
    //     Rground << 1, 0, 0,
    //    0, 0.995004165278026, -0.0998334166468282,
    //    0, 0.0998334166468282, 0.995004165278026;
    robot_->GetContactHolonomicConstraints(contact.leftC, contact.rightC, Jh_contact, dJhdq_contact);

    Jh.resize(Jh_internal.rows() + Jh_contact.rows(), robot_->nv());
    Jh << Jh_internal, Jh_contact;
    dJhdq.resize(dJhdq_internal.size() + dJhdq_contact.size());
    dJhdq << dJhdq_internal, dJhdq_contact;
}

void StandingOutput::ComputeFrictionConstriants()
{
    robot_->GetFrictionCone(config.fric_params, contact.leftC, contact.rightC, Afric, bfric_ub);
}
