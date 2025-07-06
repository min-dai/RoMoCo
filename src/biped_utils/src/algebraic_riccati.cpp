
#include "biped_utils/algebraic_riccati.hpp"


Eigen::MatrixXd solve_dare(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd Q, Eigen::MatrixXd R, double tol)
{
    Eigen::MatrixXd G = B * R.completeOrthogonalDecomposition().solve(B.transpose());
    Eigen::MatrixXd H = Q;
    Eigen::MatrixXd Hold = Eigen::MatrixXd::Zero(Q.rows(), Q.cols());
    Eigen::MatrixXd invW = Eigen::MatrixXd::Zero(Q.rows(), Q.cols());
    Eigen::MatrixXd V1 = Eigen::MatrixXd::Zero(Q.rows(), Q.cols());
    Eigen::MatrixXd V2 = Eigen::MatrixXd::Zero(Q.rows(), Q.cols());
    while ((H - Hold).norm() > tol * H.norm())
    {
        Hold = H;
        invW = (Eigen::MatrixXd::Identity(H.rows(), H.cols()) + G * H).inverse();
        V1 = invW * A;
        V2 = G * invW;
        G = G + A * V2 * A.transpose();
        H = H + V1.transpose() * H * A;
        A = A * V1;
    }
    return H;
};
Eigen::VectorXd solve_dlqr_gain(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd Q, Eigen::MatrixXd R, double tol)
{
    Eigen::MatrixXd X = solve_dare(A, B, Q, R, tol);
    Eigen::MatrixXd K = (B.transpose() * X * B + R).completeOrthogonalDecomposition().solve(B.transpose() * X * A);
    Eigen::VectorXd Kvec = K.row(0);
    return Kvec;
};