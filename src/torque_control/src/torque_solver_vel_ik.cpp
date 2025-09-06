#include "torque_control/torque_solver_vel_ik.hpp"

TorqueSolverVELIK::TorqueSolverVELIK(const std::string &config_file, std::shared_ptr<RobotBasePinocchio> robot, std::shared_ptr<OutputBase> output)
    : TorqueSolverBase(robot, output)
{
    Init(config_file);
}

void TorqueSolverVELIK::Init(const std::string &config_file)
{
    yaml_parser_.Init(config_file);
    // Load the configuration parameters
    // Implement the initialization of the solver

    VectorXd JointKPtmp = yaml_parser_.get_VectorXd("velik/JointKP");
    VectorXd JointKDtmp = yaml_parser_.get_VectorXd("velik/JointKD");
    output_ik_gain_ = yaml_parser_.get_VectorXd("velik/output_ik_gain"); 


    // check if the size of JointKP_ and JointKD_ is the same as the number of actuated joints, or half number of actuated joints
    //  if same size as actuated_u_idx then it is the PD gains for each joint
    //  if half size of actuated_u_idx then it is the PD gains for each leg
    if (JointKPtmp.size() == robot_->nu())
    {
        JointKP_ = JointKPtmp;
        JointKD_ = JointKDtmp;
    }
    else if (JointKPtmp.size() == robot_->nu() / 2)
    {
        // stack the gains for each leg
        JointKP_ = VectorXd::Zero(robot_->nu());
        JointKD_ = VectorXd::Zero(robot_->nu());
        JointKP_ << JointKPtmp, JointKPtmp;
        JointKD_ << JointKDtmp, JointKDtmp;
    }
    else
    {
        std::cerr << "Invalid size of JointKP_ and JointKD_" << std::endl;
    }

    // OutputKPing
    JointKPing_ = JointKP_;
    JointKDing_ = JointKD_;


    motor_commands_.ResizeAll(robot_->nu());
}

BipedMotorCommands TorqueSolverVELIK::Solve()
{
    SolveIk();
    
    qm_actual_ = robot_->q()(output_->actuated_q_idx);
    dqm_actual_ = robot_->dq()(output_->actuated_q_idx);

    VectorXd u_ff = SolveGravityCompensation();

    pd_controller_.Reconfigure(JointKPing_(output_->actuated_u_idx), JointKDing_(output_->actuated_u_idx));

    VectorXd u_fb = pd_controller_.Compute(output_->qDes_actuated, output_->dqDes_actuated, qm_actual_, dqm_actual_);

    Eigen::VectorXd u_full = MapU2FullIdx(u_ff+u_fb, output_->actuated_u_idx, robot_->nu());


    motor_commands_.joint_torques_ff = MapU2FullIdx(u_ff(output_->actuated_u_idx), output_->actuated_u_idx, robot_->nu());
    motor_commands_.joint_positions = MapU2FullIdx(output_->qDes_actuated, output_->actuated_u_idx, robot_->nu());
    motor_commands_.joint_velocities = MapU2FullIdx(output_->dqDes_actuated, output_->actuated_u_idx, robot_->nu());
    motor_commands_.joint_kp = MapU2FullIdx(JointKPing_(output_->actuated_u_idx), output_->actuated_u_idx, robot_->nu());
    motor_commands_.joint_kd = MapU2FullIdx(JointKDing_(output_->actuated_u_idx), output_->actuated_u_idx, robot_->nu());
    motor_commands_.joint_torques = u_full;

    std::cout << "Motor commands: " << std::endl;
    std::cout << motor_commands_<< std::endl;


    return motor_commands_;
}

void TorqueSolverVELIK::SolveIk()
{


    output_->qDes_actuated = robot_->q()(output_->actuated_q_idx);
    output_->dqDes_actuated = robot_->dq()(output_->actuated_q_idx);

    MatrixXd Nhol = MatrixXd::Zero(robot_->nv(), robot_->nv());
    MatrixXd Jc = output_->Jh;

    Nhol = MatrixXd::Identity(robot_->nv(), robot_->nv()) - Jc.completeOrthogonalDecomposition().solve(Jc);

    VectorXd delta_q_output = VectorXd::Zero(robot_->nv());
    VectorXd dq_output = VectorXd::Zero(robot_->nv());

    MatrixXd Jy = output_->Jya(output_->active_y_idx, Eigen::all);

    delta_q_output = (Jy * Nhol).completeOrthogonalDecomposition().solve(output_ik_gain_(output_->active_y_idx).cwiseProduct(output_->yd(output_->active_y_idx) - output_->ya(output_->active_y_idx)));
    dq_output = (Jy * Nhol).completeOrthogonalDecomposition().solve(output_ik_gain_(output_->active_y_idx).cwiseProduct(output_->dyd(output_->active_y_idx)));

    output_->qDes_actuated += delta_q_output(output_->actuated_q_idx);
    output_->dqDes_actuated = dq_output(output_->actuated_q_idx);


}



