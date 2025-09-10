#include "romoco_control/torque_solver_inv_dyn.hpp"
#include "romoco_utils/pseudo_inverse.hpp"

using std::cout;
using std::endl;


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

    motor_commands_.ZeroAll(robot_->nu());
}

BipedMotorCommands TorqueSolverInvDyn::Solve()
{
    VectorXd u_sol;

    Eigen::MatrixXd B = output_->B();
    Eigen::VectorXd KP = output_->SelectActiveOutputs(OutputKPing_);
    Eigen::VectorXd KD = output_->SelectActiveOutputs(OutputKDing_);

    Eigen::MatrixXd Jc = output_->Jh();
    Eigen::VectorXd ddy_star = output_->d2yd() - KP.cwiseProduct(output_->ya() - output_->yd()) - KD.cwiseProduct(output_->dya() - output_->dyd());


    if (robot_->nv() == (output_->ny() + output_->nh()))
    {
        // Inverse Dynamics Control of Floating Base Systems using Orthogonal projection
        
        Eigen::MatrixXd Q = Jc.transpose().householderQr().householderQ();

        Eigen::MatrixXd Su, Sc;
        Su.resize(robot_->nv() - output_->nh(), robot_->nv());
        Sc.resize(output_->nh(), robot_->nv());
        Su << Eigen::MatrixXd::Zero(robot_->nv() - output_->nh(), output_->nh()), Eigen::MatrixXd::Identity(robot_->nv() - output_->nh(), robot_->nv() - output_->nh());
        Sc << Eigen::MatrixXd::Identity(output_->nh(), output_->nh()), Eigen::MatrixXd::Zero(output_->nh(), robot_->nv() - output_->nh());

        
        Eigen::MatrixXd Jfullrank(output_->nh() + output_->ny(), robot_->nv());

        Jfullrank << Jc,
            output_->Jya();

        Eigen::VectorXd bfullrank(output_->nh() + output_->ny());
        bfullrank << -output_->dJhdq(),
            -output_->dJyadq() + ddy_star;

        Eigen::MatrixXd invJ = PseudoInverse(Jfullrank, threshold_);

        VectorXd ddq_target = invJ * bfullrank;

        u_sol = (Su * Q.transpose() * B).completeOrthogonalDecomposition().solve(Su) * Q.transpose() * (robot_->D() * ddq_target + robot_->H());
    }
    else if (output_->nu() == output_->ny())
    {
        // Contact Consistent Control Framework for Humanoid Robots
        // Important: this implementation only works for number of output = number of actuated motors

        Eigen::MatrixXd invD = PseudoInverse(robot_->D(), threshold_);

        Eigen::MatrixXd Dc = PseudoInverse(Jc * invD * Jc.transpose(), threshold_);
        Eigen::MatrixXd Nc = Eigen::MatrixXd::Identity(robot_->nv(), robot_->nv()) - Jc.transpose() * Dc * Jc * invD;

        Eigen::MatrixXd Jya = output_->Jya();

        Eigen::MatrixXd Do = PseudoInverse(Jya * invD * Nc * Jya.transpose(), threshold_);
        Eigen::MatrixXd Jobar_tran = Do * Jya * invD * Nc;
        Eigen::MatrixXd Co = Do * Jya * invD * (Nc * robot_->H() + Jc.transpose() * Dc * output_->dJhdq()) - Do * output_->dJyadq();

        u_sol = PseudoInverse(Jobar_tran * B, threshold_) * (Do * ddy_star + Co);


        // Eigen::VectorXd Cc = Dc * Jc * invD * robot_->H() - Dc * output_->dJhdq;

        // Eigen::VectorXd ddq_computed = invD * (Nc * B * u_sol - robot_->H() + Jc.transpose() * Cc);
    }

    Eigen::VectorXd u_full = output_->MapToFullMotors(u_sol);

    std::cout << "u_id =[ " << u_full.transpose() << "];" << std::endl;

    motor_commands_.joint_torques_ff = u_full;
    motor_commands_.joint_torques = u_full;

    // Implement the solver
    return motor_commands_;
}