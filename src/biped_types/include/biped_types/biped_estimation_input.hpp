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

    friend std::ostream &operator<<(std::ostream &os, const BipedEstimationKinematicsInput &input)
    {
        os << "BipedEstimationKinematicsInput: \n";
        os << "p_left_foot: " << input.p_left_foot.transpose() << "\n";
        os << "p_right_foot: " << input.p_right_foot.transpose() << "\n";
        os << "J_left_foot: \n"
           << input.J_left_foot << "\n";
        os << "J_right_foot: \n"
           << input.J_right_foot << "\n";
        os << "left_contact_prob: " << input.left_contact_prob << "\n";
        os << "right_contact_prob: " << input.right_contact_prob << "\n";
        return os;
    }
};

#endif // BIPED_ESTIMATION_INPUT_HPP
