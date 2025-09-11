#include <unordered_set>
// todo: remove?
#include "romoco_control/torque_solver_pos_ik.hpp"

namespace romoco
{
TorqueSolverPOSIK::TorqueSolverPOSIK(const std::string &config_file, std::shared_ptr<romoco::robot::RobotBasePinocchio> robot, std::shared_ptr<OutputBase> output)
    : TorqueSolverBase(robot, output)
{
    Init(config_file);
}

void TorqueSolverPOSIK::Init(const std::string &config_file)
{
    yaml_parser_.Init(config_file);
    // Load the configuration parameters
    // Implement the initialization of the solver

    Eigen::VectorXd JointKPtmp = yaml_parser_.get_VectorXd("posik/JointKP");
    Eigen::VectorXd JointKDtmp = yaml_parser_.get_VectorXd("posik/JointKD");

    max_iter_ = yaml_parser_.get_int("posik/max_iter");
    tol_ = yaml_parser_.get_double("posik/tol");

    // check if the size of JointKP and JointKD is the same as the number of actuated joints, or half number of actuated joints
    //  if half size of actuators then it is the PD gains for each leg
    if (JointKPtmp.size() == robot_->nu() / 2)
    {
        // stack the gains for each leg
        JointKP_ = Eigen::VectorXd::Zero(robot_->nu());
        JointKD_ = Eigen::VectorXd::Zero(robot_->nu());
        JointKP_ << JointKPtmp, JointKPtmp;
        JointKD_ << JointKDtmp, JointKDtmp;
    }
    else if (JointKPtmp.size() == robot_->nu())
    {
        JointKP_ = JointKPtmp;
        JointKD_ = JointKDtmp;
    }else{
        std::cerr << "Invalid size of JointKP_ and JointKD_" << std::endl;
    }


    motor_commands_.ResizeAll(robot_->nu());

}

BipedMotorCommands TorqueSolverPOSIK::Solve()
{

    SolveIk();

    Eigen::VectorXd KP = output_->SelectActuatedMotors(JointKP_);
    Eigen::VectorXd KD = output_->SelectActuatedMotors(JointKD_);

    pd_controller_.Reconfigure(KP, KD);

    Eigen::VectorXd u_fb = pd_controller_.Compute(qm_des_, dqm_des_, qm_actual_, dqm_actual_);

    Eigen::VectorXd u_ff = SolveGravityCompensation();

    motor_commands_.joint_torques_ff = output_->MapToFullMotors(u_ff);
    motor_commands_.joint_positions = output_->MapToFullMotors(qm_des_);
    motor_commands_.joint_velocities = output_->MapToFullMotors(dqm_des_);
    motor_commands_.joint_kp = output_->MapToFullMotors(KP);
    motor_commands_.joint_kd = output_->MapToFullMotors(KD);
    motor_commands_.joint_torques = output_->MapToFullMotors(u_fb + u_ff);

    return motor_commands_;
}

void TorqueSolverPOSIK::SolveIk()
{

    Eigen::VectorXd q_now = robot_->q();

    qm_actual_ = output_->SelectActuatedStates(q_now);
    dqm_actual_ = output_->SelectActuatedStates(robot_->dq());

    // Init q0
    qm_des_ = qm_actual_;

    // forward kinematics IK
    Eigen::VectorXd f = output_->ya();
    Eigen::MatrixXd J = output_->Jya();

    Eigen::VectorXd qfull = q_now;

    // Null-space projection for holonomic constraints
    Eigen::MatrixXd Nhol = Eigen::MatrixXd::Identity(robot_->nv(), robot_->nv()) - output_->Jh().completeOrthogonalDecomposition().solve(output_->Jh());

    int iter = 0;
    do
    {
        iter++;
        Eigen::VectorXd deltaq = (J * Nhol).completeOrthogonalDecomposition().solve(output_->yd() - f);
        qfull += deltaq;
        output_->ForwardPosIK(qfull, f, J);
    } while ((output_->yd() - f).norm() > tol_ && iter < max_iter_);

    if (iter >= max_iter_)
    {
        std::cerr << "[IK Warning] Did not converge after " << max_iter_ << " iterations. " << std::endl;
        return;
    }
    Eigen::VectorXd dqdes_full = (J * Nhol).completeOrthogonalDecomposition().solve(output_->dyd());
    qm_des_ = output_->SelectActuatedStates(qfull);
    dqm_des_ = output_->SelectActuatedStates(dqdes_full);

    // for logging
    output_->ForwardPosIK(q_now, f, J);
}

} // namespace romoco