#include "torque_control/torque_solver_vel_ik.hpp"
using namespace Eigen;
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

    ik_gain_ = yaml_parser_.get_VectorXd("velik/ik_gain");

    // OutputKPing
    JointKPing_ = JointKP_;
    JointKDing_ = JointKD_;

    pd_controller_.reconfigure(JointKP_, JointKD_);
}

Eigen::VectorXd TorqueSolverVELIK::Solve()
{
    SolveIk();
    VectorXd u_ff = SolveGravityCompensation();
    Eigen::VectorXd u_fb = SolveTorqueFeedback();

    Eigen::VectorXd u_full = MapU2FullIdx(u_ff+u_fb, output_->actuated_u_idx, robot_->nu());

    return u_full;
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

    delta_q_output = (output_->Jya(output_->active_y_idx, all) * Nhol).completeOrthogonalDecomposition().solve(ik_gain_(output_->active_y_idx).cwiseProduct(output_->yd(output_->active_y_idx) - output_->ya(output_->active_y_idx)));
    dq_output = (output_->Jya(output_->active_y_idx, all) * Nhol).completeOrthogonalDecomposition().solve(ik_gain_(output_->active_y_idx).cwiseProduct(output_->dyd(output_->active_y_idx)));

    output_->qDes_actuated += delta_q_output(output_->actuated_q_idx);
    output_->dqDes_actuated = dq_output(output_->actuated_q_idx);
    std::cout << "qDes_actuated = " << output_->qDes_actuated.transpose() << std::endl;
    std::cout << "dqDes_actuated = " << output_->dqDes_actuated.transpose() << std::endl;


}



Eigen::VectorXd TorqueSolverVELIK::SolveTorqueFeedback()
{
    VectorXd u_fb = VectorXd::Zero(output_->nu());
    pd_controller_.reconfigure(JointKPing_(output_->actuated_u_idx), JointKDing_(output_->actuated_u_idx));

    u_fb = pd_controller_.compute(output_->qDes_actuated, output_->dqDes_actuated, robot_->q()(output_->actuated_q_idx), robot_->dq()(output_->actuated_q_idx));
    std::cout << "u_fb = " << u_fb.transpose() << std::endl;
    return u_fb;
}
