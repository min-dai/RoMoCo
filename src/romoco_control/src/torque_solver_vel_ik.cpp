#include "romoco_control/torque_solver_vel_ik.hpp"

namespace romoco
{
TorqueSolverVELIK::TorqueSolverVELIK(const std::string &config_file, std::shared_ptr<romoco::robot::RobotBasePinocchio> robot, std::shared_ptr<OutputBase> output)
    : TorqueSolverBase(robot, output)
{
    Init(config_file);
}

void TorqueSolverVELIK::Init(const std::string &config_file)
{
    yaml_parser_.Init(config_file);
    // Load the configuration parameters
    // Implement the initialization of the solver

    Eigen::VectorXd JointKPtmp = yaml_parser_.get_VectorXd("velik/JointKP");
    Eigen::VectorXd JointKDtmp = yaml_parser_.get_VectorXd("velik/JointKD");
    output_ik_gain_ = yaml_parser_.get_VectorXd("velik/output_ik_gain");


    // check if the size of JointKP and JointKD is the same as the number of actuated joints, or half number of actuated joints
    //  if half size of actuators then it is the PD gains for each leg
    if (JointKPtmp.size() == robot_->nu())
    {
        JointKP_ = JointKPtmp;
        JointKD_ = JointKDtmp;
    }
    else if (JointKPtmp.size() == robot_->nu() / 2)
    {
        // stack the gains for each leg
        JointKP_ = Eigen::VectorXd::Zero(robot_->nu());
        JointKD_ = Eigen::VectorXd::Zero(robot_->nu());
        JointKP_ << JointKPtmp, JointKPtmp;
        JointKD_ << JointKDtmp, JointKDtmp;
    }
    else
    {
        std::cerr << "Invalid size of JointKP_ and JointKD_" << std::endl;
    }

    motor_commands_.ResizeAll(robot_->nu());
}

BipedMotorCommands TorqueSolverVELIK::Solve()
{
    SolveIk();
    
    Eigen::VectorXd KP = output_->SelectActuatedMotors(JointKP_);
    Eigen::VectorXd KD = output_->SelectActuatedMotors(JointKD_);

    pd_controller_.Reconfigure(KP, KD);

    qm_actual_ = output_->SelectActuatedStates(robot_->q());
    dqm_actual_ = output_->SelectActuatedStates(robot_->dq());

    Eigen::VectorXd u_ff = SolveGravityCompensation();

    Eigen::VectorXd u_fb = pd_controller_.Compute(qm_des_, dqm_des_, qm_actual_, dqm_actual_);


    motor_commands_.joint_torques_ff = output_->MapToFullMotors(u_ff);
    motor_commands_.joint_positions = output_->MapToFullMotors(qm_des_);
    motor_commands_.joint_velocities = output_->MapToFullMotors(dqm_des_);
    motor_commands_.joint_kp = output_->MapToFullMotors(KP);
    motor_commands_.joint_kd = output_->MapToFullMotors(KD);
    motor_commands_.joint_torques = output_->MapToFullMotors(u_fb + u_ff);

    return motor_commands_;
}

void TorqueSolverVELIK::SolveIk()
{
    qm_des_ = output_->SelectActuatedStates(robot_->q());

    Eigen::MatrixXd Nhol = Eigen::MatrixXd::Identity(robot_->nv(), robot_->nv()) - output_->Jh().completeOrthogonalDecomposition().solve(output_->Jh());

    Eigen::VectorXd ik_gain_active = output_->SelectActiveOutputs(output_ik_gain_);

    Eigen::VectorXd delta_q_output = (output_->Jya() * Nhol).completeOrthogonalDecomposition().solve(ik_gain_active.cwiseProduct(output_->yd() - output_->ya()));
    Eigen::VectorXd dq_output = (output_->Jya() * Nhol).completeOrthogonalDecomposition().solve(ik_gain_active.cwiseProduct(output_->dyd()));
    qm_des_ += output_->SelectActuatedStates(delta_q_output);
    dqm_des_ = output_->SelectActuatedStates(dq_output);
}

} // namespace romoco
