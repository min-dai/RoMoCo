#include "torque_control/torque_solver_inv_dyn.hpp"
#include "biped_utils/Pinverse.hpp"

using std::cout;
using std::endl;
using namespace Eigen;

TorqueSolverInvDyn::TorqueSolverInvDyn(const std::string &config_file, std::shared_ptr<RobotBasePinocchio> robot, std::shared_ptr<OutputBase> output)
    : TorqueSolverBase(robot, output)
{
    Init(config_file);
}

void TorqueSolverInvDyn::Init(const std::string &config_file)
{
    yaml_parser_.Init(config_file);
    // Load the configuration parameters
    // Implement the initialization of the solver
    threshold_ = yaml_parser_.get_double("id/threshold");
    OutputKP_ = yaml_parser_.get_VectorXd("id/OutputKP");
    OutputKD_ = yaml_parser_.get_VectorXd("id/OutputKD");

    // OutputKPing_
    OutputKPing_ = OutputKP_;
    OutputKDing_ = OutputKD_;
}

Eigen::VectorXd TorqueSolverInvDyn::Solve()
{
    VectorXd u_sol;
    if (robot_->nv() == (output_->ny() + output_->nh()))
    {
        // Inverse Dynamics Control of Floating Base Systems using Orthogonal projection
        Eigen::MatrixXd Jc = output_->Jh;
        Eigen::MatrixXd Q = Jc.transpose().householderQr().householderQ();

        Eigen::MatrixXd Su, Sc;
        Su.resize(robot_->nv() - output_->nh(), robot_->nv());
        Sc.resize(output_->nh(), robot_->nv());
        Su << Eigen::MatrixXd::Zero(robot_->nv() - output_->nh(), output_->nh()), Eigen::MatrixXd::Identity(robot_->nv() - output_->nh(), robot_->nv() - output_->nh());
        Sc << Eigen::MatrixXd::Identity(output_->nh(), output_->nh()), Eigen::MatrixXd::Zero(output_->nh(), robot_->nv() - output_->nh());

        Eigen::MatrixXd B = robot_->B()(Eigen::all, output_->actuated_u_idx);

        Eigen::MatrixXd Jfullrank(output_->nh() + output_->ny(), robot_->nv());

        Jfullrank << output_->Jh,
            output_->Jya(output_->active_y_idx, all);
        Eigen::VectorXd ddy_star = output_->d2yd - OutputKPing_.cwiseProduct(output_->ya - output_->yd) - OutputKDing_.cwiseProduct(output_->dya - output_->dyd);

        Eigen::VectorXd bfullrank(output_->nh() + output_->ny());
        bfullrank << -output_->dJhdq,
            -output_->dJyadq(output_->active_y_idx) + ddy_star(output_->active_y_idx);

        Eigen::MatrixXd invJ = PseudoInverse(Jfullrank, threshold_);

        VectorXd ddq_target = invJ * bfullrank;

        // cout << "ddq_target = [ " << ddq_target.transpose() << "];" << std::endl;

        u_sol = (Su * Q.transpose() * B).completeOrthogonalDecomposition().solve(Su) * Q.transpose() * (robot_->D() * ddq_target + robot_->H());

        cout << "u_sol = [ " << u_sol.transpose() << "];" << std::endl;

        MatrixXd Nhol = MatrixXd::Identity(robot_->nv(), robot_->nv()) - Jc.completeOrthogonalDecomposition().solve(Jc);
    }
    else if (output_->nu() == output_->ny())
    {
        // Contact Consistent Control Framework for Humanoid Robots
        // Important: this implementation only works for number of output = number of actuated motors

        MatrixXd Jc = output_->Jh;
        MatrixXd B = robot_->B()(all, output_->actuated_u_idx);
        VectorXd ddy_star = output_->d2yd - OutputKPing_.cwiseProduct(output_->ya - output_->yd) - OutputKDing_.cwiseProduct(output_->dya - output_->dyd);

        MatrixXd invD = PseudoInverse(robot_->D(), threshold_);

        MatrixXd Dc = PseudoInverse(Jc * invD * Jc.transpose(), threshold_);
        MatrixXd Nc = MatrixXd::Identity(robot_->nv(), robot_->nv()) - Jc.transpose() * Dc * Jc * invD;

        MatrixXd Jya = output_->Jya;

        MatrixXd Do = PseudoInverse(Jya * invD * Nc * Jya.transpose(), threshold_);
        MatrixXd Jobar_tran = Do * Jya * invD * Nc;
        MatrixXd Co = Do * Jya * invD * (Nc * robot_->H() + Jc.transpose() * Dc * output_->dJhdq) - Do * output_->dJyadq;

        u_sol = PseudoInverse(Jobar_tran * B, threshold_) * (Do * ddy_star + Co);

        cout << "u_sol = [ " << u_sol.transpose() << "];" << std::endl;

        // VectorXd Cc = Dc * Jc * invD * robot_->H() - Dc * output_->dJhdq;

        // VectorXd ddq_computed = invD * (Nc * B * u_sol - robot_->H() + Jc.transpose() * Cc);
    }

    Eigen::VectorXd u_full = MapU2FullIdx(u_sol, output_->actuated_u_idx, robot_->nu());

    std::cout << "u_id =[ " << u_full.transpose() << "];" << std::endl;

    // Implement the solver
    return u_full;
}