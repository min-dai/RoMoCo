#ifndef ALGEBRAIC_RICCATI_HPP
#define ALGEBRAIC_RICCATI_HPP

#include <Eigen/Dense>

Eigen::MatrixXd solve_dare(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd Q, Eigen::MatrixXd R, double tol);

Eigen::VectorXd solve_dlqr_gain(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd Q, Eigen::MatrixXd R, double tol);

#endif // ALGEBRAIC_RICCATI_HPP
