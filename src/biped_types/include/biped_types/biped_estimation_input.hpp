#ifndef BIPED_ESTIMATION_INPUT_HPP
#define BIPED_ESTIMATION_INPUT_HPP

#include <Eigen/Dense>

struct BipedEstimationKinematicsInput
{
    Eigen::VectorXd p_left_foot;
    Eigen::VectorXd p_right_foot;
    //for each leg, expected size 3 x n_joints_per_leg
    Eigen::MatrixXd J_left_foot;
    Eigen::MatrixXd J_right_foot;

    double left_contact_prob = 0.0;
    double right_contact_prob = 0.0;
};

#endif // BIPED_ESTIMATION_INPUT_HPP
