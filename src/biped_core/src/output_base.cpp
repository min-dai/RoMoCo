#include "biped_core/output_base.hpp"

OutputBase::OutputBase(std::shared_ptr<RobotBasePinocchio> robot)
    : robot_(robot)
{
}

void OutputBase::ForwardPosIk(const VectorXd &qk, VectorXd &fk, MatrixXd &Jk)
{
    robot_->UpdateKinematics(qk, VectorXd::Zero(robot_->nv()));
    ComputeActual();
    fk = ya;
    Jk = Jya;
}

void OutputBase::setBezierDesiredOutputs(const VectorXd &alpha, const double &tau, const double &dtau, const int &OutputIdx)
{
    yd(OutputIdx) = bezier(alpha, tau);
    dyd(OutputIdx) = dtimeBezier(alpha, tau, dtau);
    d2yd(OutputIdx) = dtime2Bezier(alpha, tau, dtau);
}

void OutputBase::setZeroOutputs(const int &OutputIdx)
{
    yd(OutputIdx) = 0.0;
    dyd(OutputIdx) = 0.0;
    d2yd(OutputIdx) = 0.0;
}