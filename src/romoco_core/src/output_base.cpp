#include "romoco_core/output_base.hpp"

OutputBase::OutputBase(std::shared_ptr<RobotBasePinocchio> robot)
    : robot_(robot)
{
}

Eigen::VectorXd OutputBase::MapToFullMotors(const Eigen::VectorXd &u)
{
    return MapU2FullIdx(u, actuated_u_idx_, robot_->nu());
}

Eigen::VectorXd OutputBase::SelectActuatedStates(const Eigen::VectorXd &q_in)
{
    if (q_in.size() != robot_->nq()) {
        throw std::invalid_argument("Input vector size does not match robot joint size");
    }
    return q_in(actuated_q_idx_);
}
Eigen::VectorXd OutputBase::SelectActuatedMotors(const Eigen::VectorXd &u_in)
{
    if (u_in.size() != robot_->nu()) {
        throw std::invalid_argument("Input vector size does not match robot actuator size");
    }
    return u_in(actuated_u_idx_);
}
Eigen::VectorXd OutputBase::SelectActiveOutputs(const Eigen::VectorXd &y_in)
{
    if (y_in.size() != nY_) {
        throw std::invalid_argument("Input vector size is smaller than number of outputs");
    }
    return y_in(active_y_idx_);
}


void OutputBase::SetOutputSize(int nq, int ny)
{
    ya_ = Eigen::VectorXd::Zero(ny);
    dya_ = Eigen::VectorXd::Zero(ny);
    Jya_ = Eigen::MatrixXd::Zero(ny, nq);
    dJyadq_ = Eigen::VectorXd::Zero(ny);

    yd_ = Eigen::VectorXd::Zero(ny);
    dyd_ = Eigen::VectorXd::Zero(ny);
    d2yd_ = Eigen::VectorXd::Zero(ny);
}



void OutputBase::ForwardPosIK(const VectorXd &qk, VectorXd &fk, MatrixXd &Jk)
{
    robot_->UpdateKinematics(qk, VectorXd::Zero(robot_->nv()));
    ComputeActual();
    fk = ya_(active_y_idx_);
    Jk = Jya_(active_y_idx_, Eigen::all);
}

void OutputBase::SetBezierDesiredOutputs(const VectorXd &alpha, const double &tau, const double &dtau, const int &OutputIdx)
{
    yd_(OutputIdx) = bezier_tools::bezier(alpha, tau);
    dyd_(OutputIdx) = bezier_tools::dtimeBezier(alpha, tau, dtau);
    d2yd_(OutputIdx) = bezier_tools::dtime2Bezier(alpha, tau, dtau);
}

Eigen::VectorXd OutputBase::MapU2FullIdx(const Eigen::VectorXd &u, const std::vector<int> &actuated_u_idx, const int &full_size){
    Eigen::VectorXd u_full = Eigen::VectorXd::Zero(full_size);
    for (int i = 0; i < actuated_u_idx.size(); i++){
        u_full(actuated_u_idx[i]) = u(i);
    }
    return u_full;
}