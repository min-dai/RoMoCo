#include "biped_control/inair_output.hpp"

InAirOutput::InAirOutput(const std::string &config_file, std::shared_ptr<RobotBasePinocchio> robot)
    : OutputBase(robot)
{
    config.yaml_parser.Init(config_file);
    config.Init(robot_->robot_type());

    if (robot_->robot_type() == RobotType::PlaneFoot)
    {
        nY = 12;
    }
    else if (robot_->robot_type() == RobotType::LineFoot)
    {
        nY = 10;
    }

    SetOutputSize(robot_->nq(), nY);
    actuated_q_idx = robot_->actuated_q_idx(AnkleMotorStatus::ActiveAll, AnkleMotorStatus::ActiveAll);
    actuated_u_idx = robot_->actuated_u_idx(AnkleMotorStatus::ActiveAll, AnkleMotorStatus::ActiveAll);
    active_y_idx = OutputBase::generate_full_y_idx(nY);

    lowpassyd.reconfigure(config.dt_lowpass, config.yd_lowpass_dt_cutoff, nY);
}

void InAirOutput::Config::Init(RobotType robot_type)
{
    dt_lowpass = yaml_parser.get_double("pose_command/dt_lowpass");
    swingZ_lb = yaml_parser.get_double("pose_command/swingZ_lb");
    swingZ_ub = yaml_parser.get_double("pose_command/swingZ_ub");
    swingX_backward = yaml_parser.get_double("pose_command/swingX_backward");
    swingX_forward = yaml_parser.get_double("pose_command/swingX_forward");
    swingY_inner = yaml_parser.get_double("pose_command/swingY_inner");
    swingY_outer = yaml_parser.get_double("pose_command/swingY_outer");
    yaw_lb = yaml_parser.get_double("pose_command/yaw_lb");
    yaw_ub = yaml_parser.get_double("pose_command/yaw_ub");
    pitch_lb = yaml_parser.get_double("pose_command/pitch_lb");
    pitch_ub = yaml_parser.get_double("pose_command/pitch_ub");

    if (robot_type == RobotType::PlaneFoot)
    {
        roll_lb = yaml_parser.get_double("pose_command/roll_lb");
        roll_ub = yaml_parser.get_double("pose_command/roll_ub");
    }

    yd_lowpass_dt_cutoff = yaml_parser.get_VectorXd("pose_command/yd_lowpass_dt_cutoff");
}

void InAirOutput::UpdateOutput(const Eigen::VectorXd &reference, const double &t, const double &t_old)
{
    // Add the implementation here
    updated.t = t;

    // make sure q and qdot base are zeroed
    robot_->UpdateKinematicsZeroBase();

    // Compute Actual
    ComputeActual();

    if (!updated.isInitialized)
    {
        updated.isInitialized = true;
        lowpassyd.update(ya);

        yd = ya;
    }

    // Compute Desired
    ComputeDesired(reference);

    // holonomic constriants changes if there're internal constraints
    ComputeHolonomicConstraints();
}

void InAirOutput::ComputeActual()
{
    // assume kinematics is updated in robot_
    left_foot2base = robot_->left_below_ankle_kinematics() - robot_->base_kinematics();

    right_foot2base = robot_->right_below_ankle_kinematics() - robot_->base_kinematics();

    ya << left_foot2base.position,                 // 3
        robot_->left_hip_yaw_kinematics().position,  // 1
        robot_->GetLeftFootDeltaPitch().position,  // 1
        robot_->GetLeftFootDeltaRoll().position,   // 1 or 0
        right_foot2base.position,                  // 3
        robot_->right_hip_yaw_kinematics().position, // 1
        robot_->GetRightFootDeltaPitch().position, // 1
        robot_->GetRightFootDeltaRoll().position;  // 1 or 0

    dya << left_foot2base.velocity,                // 3
        robot_->left_hip_yaw_kinematics().velocity,  // 1
        robot_->GetLeftFootDeltaPitch().velocity,  // 1
        robot_->GetLeftFootDeltaRoll().velocity,   // 1 or 0
        right_foot2base.velocity,                  // 3
        robot_->right_hip_yaw_kinematics().velocity, // 1
        robot_->GetRightFootDeltaPitch().velocity, // 1
        robot_->GetRightFootDeltaRoll().velocity;  // 1 or 0

    Jya << left_foot2base.jacobian,                // 3
        robot_->left_hip_yaw_kinematics().jacobian,  // 1
        robot_->GetLeftFootDeltaPitch().jacobian,  // 1
        robot_->GetLeftFootDeltaRoll().jacobian,   // 1 or 0
        right_foot2base.jacobian,                  // 3
        robot_->right_hip_yaw_kinematics().jacobian, // 1
        robot_->GetRightFootDeltaPitch().jacobian, // 1
        robot_->GetRightFootDeltaRoll().jacobian;  // 1 or 0

    dJyadq << left_foot2base.dJdq,             // 3
        robot_->left_hip_yaw_kinematics().dJdq,  // 1
        robot_->GetLeftFootDeltaPitch().dJdq,  // 1
        robot_->GetLeftFootDeltaRoll().dJdq,   // 1 or 0
        right_foot2base.dJdq,                  // 3
        robot_->right_hip_yaw_kinematics().dJdq, // 1
        robot_->GetRightFootDeltaPitch().dJdq, // 1
        robot_->GetRightFootDeltaRoll().dJdq;  // 1 or 0
}

void InAirOutput::ComputeDesired(const Eigen::VectorXd &reference)
{

    double xcom_d = 0.5 * reference(Radio::LV) * (config.swingX_forward - config.swingX_backward) + 0.5 * (config.swingX_forward + config.swingX_backward);
    double ycom_d = 0.5 * reference(Radio::LH) * (config.swingY_outer - config.swingY_inner) + 0.5 * (config.swingY_inner + config.swingY_outer);
    double zcom_d = 0.5 * reference(Radio::LS) * (config.swingZ_ub - config.swingZ_lb) + 0.5 * (config.swingZ_ub + config.swingZ_lb);
    double pitch_d = 0.5 * reference(Radio::RV) * (config.pitch_ub - config.pitch_lb) + 0.5 * (config.pitch_ub + config.pitch_lb);
    double yaw_d = 0.5 * reference(Radio::RS) * (config.yaw_ub - config.yaw_lb) + 0.5 * (config.yaw_ub + config.yaw_lb);

    if (robot_->robot_type() == RobotType::PlaneFoot)
    {
        double roll_d = 0.5 * reference(Radio::RH) * (config.roll_ub - config.roll_lb) + 0.5 * (config.roll_ub + config.roll_lb);

        yd << xcom_d, ycom_d, zcom_d, -yaw_d, pitch_d, -roll_d,
            xcom_d, -ycom_d, zcom_d, yaw_d, pitch_d, roll_d;
    }
    else if (robot_->robot_type() == RobotType::LineFoot)
    {
        yd << xcom_d, ycom_d, zcom_d, -yaw_d, pitch_d,
            xcom_d, -ycom_d, zcom_d, yaw_d, pitch_d;
    }

    yd = lowpassyd.update(yd);

    dyd.setZero();

    d2yd.setZero();
}

void InAirOutput::ComputeHolonomicConstraints()
{
    MatrixXd Jh_internal, Jh_base;
    VectorXd dJhdq_internal, dJhdq_base;
    robot_->GetInternalHolonomicConstraints(Jh_internal, dJhdq_internal);

    Jh_base = MatrixXd::Zero(6, robot_->nv());
    Jh_base.block(0, 0, 6, 6) = MatrixXd::Identity(6, 6);
    dJhdq_base = VectorXd::Zero(6);

    Jh.resize(Jh_internal.rows() + Jh_base.rows(), robot_->nv());
    Jh << Jh_internal,
        Jh_base;
    dJhdq.resize(dJhdq_internal.rows() + dJhdq_base.rows());
    dJhdq << dJhdq_internal,
        dJhdq_base;
}