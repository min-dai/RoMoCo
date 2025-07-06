#include "torque_control/torque_solver_tscqp.hpp"


using std::cout;
using std::endl;
using namespace clarabel;
using namespace Eigen;

TorqueSolverTSCQP::TorqueSolverTSCQP(const std::string &config_file, std::shared_ptr<RobotBasePinocchio> robot, std::shared_ptr<OutputBase> output)
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

    // OutputKPing
    OutputKPing_ = OutputKP_;
    OutputKDing_ = OutputKD_;

    ResetSize();

    // Settings
    settings_ = DefaultSettings<double>::default_settings();
    settings_.verbose = false;

}

void TorqueSolverTSCQP::ResetSize()
{
    nVar_ = robot_->nv() + output_->nu() + output_->nh();

    
    G_ = MatrixXd::Zero(nVar_, nVar_);
    g_ = VectorXd::Zero(nVar_);


    Aeq_ = MatrixXd::Zero(robot_->nv()+output_->nh(), nVar_);
    beq_ = VectorXd::Zero(robot_->nv()+output_->nh());

    

    Aub_u_ = MatrixXd::Zero(output_->nu(), nVar_);
    Aub_u_.block(0, robot_->nv(), output_->nu(), output_->nu()) << MatrixXd::Identity(output_->nu(), output_->nu());

    Aub_fric_ = MatrixXd::Zero(output_->nfric(), nVar_);
    bub_fric_.resize(output_->nfric());
    if (output_->nfric() > 0){
        Aub_fric_.rightCols(output_->nFc())<< output_->Afric;
        
        bub_fric_ << output_->bfric_ub;
    }

    u_sol_ = VectorXd::Zero(output_->nu());
    F_sol_ = VectorXd::Zero(output_->nh());
    u_sol_prev_ = VectorXd::Zero(output_->nu());
}

Eigen::VectorXd TorqueSolverTSCQP::Solve()
{
    //x = [ddq^T; u^T; F_internal^T; F_external]^T
    if (nVar_ != robot_->nv() + output_->nu() + output_->nh()){
        ResetSize();
    }
    
    //cost: ||Jya*ddq + dJyadq - ddy*||^2 
    A_y_ = MatrixXd::Zero(output_->ny(), nVar_);
    b_y_ = VectorXd::Zero(output_->ny());
    A_y_.block(0, 0, output_->ny(), robot_->nv())  << output_->Jya(output_->active_y_idx, all);
    b_y_ << output_->dJyadq(output_->active_y_idx) 
         - output_->d2yd(output_->active_y_idx) 
         + OutputKPing_(output_->active_y_idx).cwiseProduct(output_->ya(output_->active_y_idx) - output_->yd(output_->active_y_idx)) 
         + OutputKDing_(output_->active_y_idx).cwiseProduct(output_->dya(output_->active_y_idx) - output_->dyd(output_->active_y_idx));
    
    G_ << A_y_.transpose() * A_y_;
    g_ << A_y_.transpose() * b_y_;


    G_ += 1e-6 * Eigen::MatrixXd::Identity(G_.rows(), G_.cols());

    


    //equality constraints
    // [D  -B -Jh^T] [ddq] = [-H]
    // [Jh  0     0] [u  ]   [-dJhdq]
    //               [F  ]   
    Aeq_.topRows(robot_->nv()) << robot_->D(), -robot_->B()(all, output_->actuated_u_idx), -output_->Jh.transpose();
    Aeq_.block(Aeq_.rows()-output_->nh(), 0, output_->nh(), robot_->nv()) << output_->Jh;                                      
    beq_.topRows(robot_->nv()) << -robot_->H();
    beq_.bottomRows(output_->nh()) << -output_->dJhdq;



    //inequality constraints
    // [0  0  0  Aub_fric_] [ddq] <= [bub_fric_]
    //                     [u  ]  
    //                     [F_internal]  
    //                     [F_contact ]
 

    if_solved_ = ClarabelSolve();
    Eigen::VectorXd u_full = MapU2FullIdx(u_sol_, output_->actuated_u_idx, robot_->nu());
    cout << "u_qp =[ " <<u_full.transpose() << "];" << endl;




    // Implement the solver
    return u_full;
}

bool TorqueSolverTSCQP::ClarabelSolve()
{
    Eigen::SparseMatrix<double> P(G_.sparseView());
    P.makeCompressed();


    MatrixXd Alarge(beq_.size() + bub_fric_.size() + output_->nu() * 2, nVar_);
    Alarge << Aeq_, //output nv+nh
        Aub_fric_,  //output nfric
        Aub_u_,     //output nu
        -Aub_u_;    //output nu

    SparseMatrix<double> AlargeSparse = Alarge.sparseView();
    AlargeSparse.makeCompressed();

    VectorXd bLarge(beq_.size() + bub_fric_.size() + output_->nu() * 2);
    bLarge << beq_,
        bub_fric_,
        robot_->u_ub()(output_->actuated_u_idx),
        -robot_->u_lb()(output_->actuated_u_idx);



    std::vector<SupportedConeT<double>> cones{
        ZeroConeT<double>(beq_.size()),
        NonnegativeConeT<double>(bub_fric_.size() + output_->nu() * 2),
    };



 
    

    DefaultSolver<double> solver(P, g_, AlargeSparse, bLarge, cones, settings_);
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
            std::cout << "ya = " << output_->ya.transpose() << std::endl;
            // std::cout << "dya = " << output_->dya.transpose() << std::endl;
            // std::cout << "Jya = " << output_->Jya << std::endl;
            // std::cout << "dJyadq = " << output_->dJyadq.transpose() << std::endl;
            std::cout << "yd = " << output_->yd.transpose() << std::endl;
            // std::cout << "dyd = " << output_->dyd.transpose() << std::endl;
            // std::cout << "d2yd = " << output_->d2yd.transpose() << std::endl;


            
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
        MatrixXd JyJh = MatrixXd::Zero(output_->ny() + output_->nh(), robot_->nv());
        JyJh << output_->Jya(output_->active_y_idx,all), output_->Jh;
        Eigen::FullPivLU<MatrixXd> lu(JyJh);
        int rank = lu.rank();
        std::cout << "JyJh rank = " << rank << ", rows = " << JyJh.rows() << std::endl;
        std::cout << "ya = " << output_->ya.transpose() << std::endl;
        std::cout << "yd = " << output_->yd.transpose() << std::endl;


        // TODO: safe action
        u_sol_ = u_sol_prev_;
        u_sol_.setZero();
        u_sol_prev_.setZero();
    }

    return if_solved_;
}
