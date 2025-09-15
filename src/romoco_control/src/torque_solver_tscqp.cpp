#include "romoco_control/torque_solver_tscqp.hpp"

namespace romoco
{

using std::cout;
using std::endl;
using namespace clarabel;

TorqueSolverTSCQP::TorqueSolverTSCQP(const std::string &config_file, std::shared_ptr<romoco::robot::RobotBasePinocchio> robot, std::shared_ptr<OutputBase> output)
    : TorqueSolverBase(robot, output)
{
    Init(config_file);
}

void TorqueSolverTSCQP::Init(const std::string &config_file)
{
    yaml_parser_.Init(config_file);
    // Load the configuration parameters
    // Implement the initialization of the solver
    print_qp_ = yaml_parser_.get_bool("qp/ifprintQP");
    OutputKP_ = yaml_parser_.get_VectorXd("qp/OutputKP");
    OutputKD_ = yaml_parser_.get_VectorXd("qp/OutputKD");
    OutputW_ = yaml_parser_.get_VectorXd("qp/OutputW");

    ResetSize();

    // Settings
    settings_ = DefaultSettings<double>::default_settings();
    settings_.verbose = false;

    motor_commands_.ResizeTorque(robot_->nv());

}

void TorqueSolverTSCQP::ResetSize()
{
    nVar_ = robot_->nv() + output_->nu() + output_->nh();


    G_ = Eigen::MatrixXd::Zero(nVar_, nVar_);
    g_ = Eigen::VectorXd::Zero(nVar_);


    Aeq_ = Eigen::MatrixXd::Zero(robot_->nv()+output_->nh(), nVar_);
    beq_ = Eigen::VectorXd::Zero(robot_->nv()+output_->nh());

    

    Aub_u_ = Eigen::MatrixXd::Zero(output_->nu(), nVar_);
    Aub_u_.block(0, robot_->nv(), output_->nu(), output_->nu()) << Eigen::MatrixXd::Identity(output_->nu(), output_->nu());

    Aub_fric_ = Eigen::MatrixXd::Zero(output_->nfric(), nVar_);
    bub_fric_.resize(output_->nfric());
    if (output_->nfric() > 0){
        Aub_fric_.rightCols(output_->nFc())<< output_->Afric();
        
        bub_fric_ << output_->bfric_ub();
    }

    u_sol_ = Eigen::VectorXd::Zero(output_->nu());
    F_sol_ = Eigen::VectorXd::Zero(output_->nh());
    u_sol_prev_ = Eigen::VectorXd::Zero(output_->nu());

    motor_commands_.ZeroAll(robot_->nu());
}

BipedMotorCommands TorqueSolverTSCQP::Solve()
{
    //x = [ddq^T; u^T; F_internal^T; F_external]^T
    if (nVar_ != robot_->nv() + output_->nu() + output_->nh()){
        ResetSize();
    }
    Eigen::VectorXd KP = output_->SelectActiveOutputs(OutputKP_);
    Eigen::VectorXd KD = output_->SelectActiveOutputs(OutputKD_);
    
    //cost: ||Jya*ddq + dJyadq - ddy*||^2 
    A_y_ = Eigen::MatrixXd::Zero(output_->ny(), nVar_);
    b_y_ = Eigen::VectorXd::Zero(output_->ny());
    A_y_.block(0, 0, output_->ny(), robot_->nv()) << output_->Jya();
    b_y_ << output_->dJyadq()
         - output_->d2yd()
         + KP.cwiseProduct(output_->ya() - output_->yd())
         + KD.cwiseProduct(output_->dya() - output_->dyd());


    ComputeWeightedQuadraticCostTerms(A_y_, b_y_, OutputW_, G_, g_);

    //equality constraints
    // [D  -B -Jh^T] [ddq] = [-H]
    // [Jh  0     0] [u  ]   [-dJhdq]
    //               [F  ]   
    Aeq_.topRows(robot_->nv()) << robot_->D(), -output_->B(), -output_->Jh().transpose();
    Aeq_.block(Aeq_.rows()-output_->nh(), 0, output_->nh(), robot_->nv()) << output_->Jh();
    beq_.topRows(robot_->nv()) << -robot_->H();
    beq_.bottomRows(output_->nh()) << -output_->dJhdq();

    //inequality constraints
    // [0  0  0  Aub_fric_] [ddq] <= [bub_fric_]
    //                     [u  ]  
    //                     [F_internal]  
    //                     [F_contact ]
    Aub_.resize(bub_fric_.size() + output_->nu() + output_->nu(), nVar_);
    Aub_ << Aub_fric_, // output nfric
            Aub_u_,    // output nu
            -Aub_u_;   // output nu
    bub_.resize(bub_fric_.size() + output_->nu() + output_->nu());
    bub_ << bub_fric_, // output nfric
            output_->SelectActuatedMotors(robot_->u_ub()), // output nu
            -output_->SelectActuatedMotors(robot_->u_lb()); // output nu


 

    // if_solved_ = ClarabelSolve();
    if_solved_ = ClarabelSolve(G_, g_, Aub_, bub_, Aeq_, beq_, u_sol_); 
    Eigen::VectorXd u_full = output_->MapToFullMotors(u_sol_);
    motor_commands_.joint_torques_ff = u_full;
    motor_commands_.joint_torques = u_full;


    // Implement the solver
    return motor_commands_;
}



bool TorqueSolverTSCQP::ClarabelSolve(
    const Eigen::MatrixXd &G,
    const Eigen::VectorXd &g,
    const std::optional<Eigen::MatrixXd> &Aub,
    const std::optional<Eigen::VectorXd> &bub,
    const std::optional<Eigen::MatrixXd> &Aeq,
    const std::optional<Eigen::VectorXd> &beq,
    Eigen::VectorXd &sol)
{

    Eigen::MatrixXd A_combined;
    Eigen::VectorXd b_combined;
    std::vector<clarabel::SupportedConeT<double>> cones;

    int neq = 0;
    int nub = 0;

    Eigen::MatrixXd Aeq_, Aub_;
    Eigen::VectorXd beq_, bub_;

    // Add equality constraints if provided
    if (Aeq.has_value() && beq.has_value())
    {
        Aeq_ = *Aeq;
        beq_ = *beq;
        neq = Aeq_.rows();
        if (Aeq_.rows() != beq_.size())
        {
            std::cerr << "Error: Aeq and beq size mismatch!" << std::endl;
            return false;
        }
        cones.emplace_back(clarabel::ZeroConeT<double>(neq));
    }

    // Add inequality constraints if provided
    if (Aub.has_value() && bub.has_value())
    {
        Aub_ = *Aub;
        bub_ = *bub;
        nub = Aub_.rows();
        if (Aub_.rows() != bub_.size())
        {
            std::cerr << "Error: Aub and bub size mismatch!" << std::endl;
            return false;
        }
        cones.emplace_back(clarabel::NonnegativeConeT<double>(nub));
    }

    // Combine equality and inequality constraints
    A_combined.resize(neq + nub, G.cols());
    b_combined.resize(neq + nub);
    if (neq > 0)
    {
        A_combined.topRows(neq) = Aeq_;
        b_combined.head(neq) = beq_;
    }
    if (nub > 0)
    {
        A_combined.bottomRows(nub) = Aub_;
        b_combined.tail(nub) = bub_;
    }

    Eigen::SparseMatrix<double> P(G.sparseView());
    P.makeCompressed();

    Eigen::SparseMatrix<double> ASparse = A_combined.sparseView();
    ASparse.makeCompressed();

    //convert to nontemporary variables
    Eigen::VectorXd p = g;
    Eigen::VectorXd b = b_combined;

    DefaultSolver<double> solver(P, p, ASparse, b, cones, settings_);

    solver.solve();
    DefaultSolution<double> solution = solver.solution();

    if (solution.status == clarabel::SolverStatus::Solved)
    {
        if_solved_ = 1;
        sol_ = solution.x;
        u_sol_ << sol_.segment(robot_->nv(), output_->nu());
        F_sol_ << sol_.segment(robot_->nv()+ output_->nu(), output_->nh());

        u_sol_prev_ << u_sol_;

        if (print_qp_)
        {

            cout << "sol----------------------------------------" << endl;
            std::cout << "ya = " << output_->ya().transpose() << std::endl;
            // std::cout << "dya = " << output_->dya().transpose() << std::endl;
            // std::cout << "Jya = " << output_->Jya().transpose() << std::endl;
            // std::cout << "dJyadq = " << output_->dJyadq().transpose() << std::endl;
            std::cout << "yd = " << output_->yd().transpose() << std::endl;
            // std::cout << "dyd = " << output_->dyd().transpose() << std::endl;
            // std::cout << "d2yd = " << output_->d2yd().transpose() << std::endl;


            
            // cout << "% Gurobi solution: " << endl;
            cout << "sol =[ " << sol_.transpose()<< "]';" << endl;
            cout << "u_clarabel =[ " <<u_sol_.transpose() << "]';" << endl;
            // cout << "F_clarabel =[ " <<F_sol_.transpose() << "]';" << endl;
  


            cout << "Aineq*x = " << (Aub_fric_ * sol_).transpose() << endl;
            cout << "bineq = " << bub_fric_.transpose() << endl;

            // cout << "Aeq_*x = " << (Aeq_*sol_).transpose() << endl;
            // cout << "beq_ = " << beq_.transpose() << endl;

            

            cout << "----------------------------------------" << endl;
        }
    }
    else
    { 
        if_solved_ = 0;

        std::cout << "THE QP in locomotion FAILED!" << std::endl;

        // check rank of [Jy; Jh]
        Eigen::MatrixXd JyJh = Eigen::MatrixXd::Zero(output_->ny() + output_->nh(), robot_->nv());
        JyJh << output_->Jya(), output_->Jh();
        Eigen::FullPivLU<Eigen::MatrixXd> lu(JyJh);
        int rank = lu.rank();
        std::cout << "JyJh rank = " << rank << ", rows = " << JyJh.rows() << std::endl;
        std::cout << "ya = " << output_->ya().transpose() << std::endl;
        std::cout << "yd = " << output_->yd().transpose() << std::endl;


        // TODO: safe action
        u_sol_ = u_sol_prev_;
        u_sol_.setZero();
        u_sol_prev_.setZero();
    }

    return if_solved_;
}

void TorqueSolverTSCQP::ComputeWeightedQuadraticCostTerms(
    const Eigen::MatrixXd &Acost,
    const Eigen::VectorXd &bcost,
    const Eigen::VectorXd &w,
    Eigen::MatrixXd &G,
    Eigen::VectorXd &g)
{
    // Convert weights to a diagonal matrix
    Eigen::MatrixXd W = output_->SelectActiveOutputs(w).asDiagonal();
    G.resize(Acost.cols(), Acost.cols());
    g.resize(Acost.cols());

    G = Acost.transpose() * W * Acost;
    g = Acost.transpose() * W * bcost;
}

} // namespace romoco