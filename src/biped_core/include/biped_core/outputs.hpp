#pragma once


#include <Eigen/Dense>
#include <vector>



// a shared struct among inair, stand, and walking output
// everything related to output that needs to pass into QP
struct Outputs{
    Eigen::VectorXd ya;
    Eigen::VectorXd dya;
    Eigen::MatrixXd Jya;
    Eigen::VectorXd dJyadq;

    Eigen::VectorXd yd;
    Eigen::VectorXd dyd;
    Eigen::VectorXd d2yd;


    Eigen::MatrixXd Jh;
    Eigen::VectorXd dJhdq;


    Eigen::MatrixXd Afric;
    Eigen::VectorXd bfric_ub;

    Eigen::VectorXd qDes_actuated;
    Eigen::VectorXd dqDes_actuated;




    //actuated indices in actuator list
    //mostly just full list {0,1,2,...,nu}
    //may deactivate stance ankle torque for HLIP walking
    std::vector<int> actuated_u_idx;
    std::vector<int> actuated_q_idx;
    std::vector<int> active_y_idx;

    void SetOutputSize(int nq, int ny)
    {
        ya = Eigen::VectorXd::Zero(ny);
        dya = Eigen::VectorXd::Zero(ny);
        Jya = Eigen::MatrixXd::Zero(ny, nq);
        dJyadq = Eigen::VectorXd::Zero(ny);

        yd = Eigen::VectorXd::Zero(ny);
        dyd = Eigen::VectorXd::Zero(ny);
        d2yd = Eigen::VectorXd::Zero(ny);
    };

    int nh(){return Jh.rows();}
    int nFc(){return Afric.cols();} //number of forces corresponds to Contact Force
    int nu(){return actuated_u_idx.size();}
    int ny(){return active_y_idx.size();}
    int nfric(){return bfric_ub.size();} //rows of friction cone constraints

};

inline Eigen::VectorXd MapU2FullIdx(const Eigen::VectorXd &u, const std::vector<int> &actuated_u_idx, const int &full_size){
    Eigen::VectorXd u_full = Eigen::VectorXd::Zero(full_size);
    for (int i = 0; i < actuated_u_idx.size(); i++){
        u_full(actuated_u_idx[i]) = u(i);
    }
    return u_full;
}




