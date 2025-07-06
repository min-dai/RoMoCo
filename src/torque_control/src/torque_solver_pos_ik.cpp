#include <unordered_set>
#include "torque_control/torque_solver_pos_ik.hpp"


TorqueSolverPOSIK::TorqueSolverPOSIK(const std::string &config_file, std::shared_ptr<RobotBasePinocchio> robot, std::shared_ptr<OutputBase> output)
    : TorqueSolverBase(robot, output)
{
    Init(config_file);
}

void TorqueSolverPOSIK::Init(const std::string &config_file)
{
    yaml_parser_.Init(config_file);
    // Load the configuration parameters
    // Implement the initialization of the solver

    VectorXd JointKPtmp = yaml_parser_.get_VectorXd("posik/JointKP");
    VectorXd JointKDtmp = yaml_parser_.get_VectorXd("posik/JointKD");

    max_iter_ = yaml_parser_.get_int("posik/max_iter");
    tol_ = yaml_parser_.get_double("posik/tol");

    // check if the size of JointKP and JointKD is the same as the number of actuated joints, or half number of actuated joints
    //  if half size of actuated_u_idx then it is the PD gains for each leg
    if (JointKPtmp.size() == robot_->nu() / 2)
    {
        // stack the gains for each leg
        JointKP_ = VectorXd::Zero(robot_->nu());
        JointKD_ = VectorXd::Zero(robot_->nu());
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

    // OutputKPing
    JointKPing_ = JointKP_;
    JointKDing_ = JointKD_;


    
}

Eigen::VectorXd TorqueSolverPOSIK::Solve()
{
    SolveIk();

    pd_controller_.reconfigure(JointKPing_(output_->actuated_u_idx), JointKDing_(output_->actuated_u_idx));

    VectorXd u_fb = pd_controller_.compute(output_->qDes_actuated, output_->dqDes_actuated, qm_actual_, dqm_actual_);
    std::cout << "u_fb = " << u_fb.transpose() << std::endl;

    VectorXd u_ff = SolveGravityCompensation();

    Eigen::VectorXd u_full = MapU2FullIdx(u_fb+u_ff, output_->actuated_u_idx, robot_->nu());
    return u_full;
}

void TorqueSolverPOSIK::SolveIk()
{

    VectorXd q_now = robot_->q();

    qm_actual_ = robot_->q()(output_->actuated_q_idx);
    dqm_actual_ = robot_->dq()(output_->actuated_q_idx);

    // Init q0
    output_->qDes_actuated = qm_actual_;

    // forward kinematics IK
    VectorXd f = output_->ya;
    MatrixXd J = output_->Jya;

    VectorXd qfull = q_now;

    // Null-space projection for holonomic constraints
    MatrixXd Nhol = MatrixXd::Zero(robot_->nv(), robot_->nv());
    MatrixXd Jc = output_->Jh;

    Nhol = MatrixXd::Identity(robot_->nv(), robot_->nv()) - Jc.completeOrthogonalDecomposition().solve(Jc);

    int iter = 0;
    do
    {
        iter++;
        VectorXd deltaq = (J(output_->active_y_idx, Eigen::all) * Nhol).completeOrthogonalDecomposition().solve(output_->yd(output_->active_y_idx) - f(output_->active_y_idx));

        qfull += deltaq;
        output_->ForwardPosIk(qfull, f, J);
    } while ((output_->yd(output_->active_y_idx) - f(output_->active_y_idx)).norm() > tol_ && iter < max_iter_);

    if (iter >= max_iter_)
    {
        std::cerr << "[IK Warning] Did not converge after " << max_iter_ << " iterations. " << std::endl;
    }

    unactuated_idx_ = get_unactuated_indices(output_->actuated_q_idx, robot_->nv());

    VectorXd dqdes_all = (J(output_->active_y_idx, Eigen::all) * Nhol).completeOrthogonalDecomposition().solve(output_->dyd(output_->active_y_idx));
    output_->qDes_actuated = qfull(output_->actuated_q_idx);
    output_->dqDes_actuated = dqdes_all(output_->actuated_q_idx);

    // for logging
    output_->ForwardPosIk(q_now, f, J);
}

std::vector<int> TorqueSolverPOSIK::get_unactuated_indices(const std::vector<int> &actuated_q_idx, int q_size)
{
    std::unordered_set<int> actuated_set(actuated_q_idx.begin(), actuated_q_idx.end());
    std::vector<int> unactuated_idx;

    for (int i = 0; i < q_size; ++i)
    {
        if (actuated_set.find(i) == actuated_set.end())
        {
            unactuated_idx.push_back(i);
        }
    }

    return unactuated_idx;
}