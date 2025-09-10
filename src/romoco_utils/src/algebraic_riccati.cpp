
#include "romoco_utils/algebraic_riccati.hpp"

namespace romoco{
Eigen::MatrixXd SolveDare(const Eigen::Ref<const Eigen::MatrixXd>& A, const Eigen::Ref<const Eigen::MatrixXd>& B, const Eigen::Ref<const Eigen::MatrixXd>& Q, const Eigen::Ref<const Eigen::MatrixXd>& R, double tol)
{
      Eigen::MatrixXd A_m = A;


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
        V1 = invW * A_m;
        V2 = G * invW;
        G = G + A_m * V2 * A_m.transpose();
        H = H + V1.transpose() * H * A_m;
        A_m = A_m * V1;
    }
    return H;
};
Eigen::VectorXd SolveDlqrGain(const Eigen::Ref<const Eigen::MatrixXd>& A, const Eigen::Ref<const Eigen::MatrixXd>& B, const Eigen::Ref<const Eigen::MatrixXd>& Q, const Eigen::Ref<const Eigen::MatrixXd>& R, double tol)
{
    Eigen::MatrixXd X = SolveDare(A, B, Q, R, tol);
    Eigen::MatrixXd K = (B.transpose() * X * B + R).completeOrthogonalDecomposition().solve(B.transpose() * X * A);
    Eigen::VectorXd Kvec = K.row(0);
    return Kvec;
};
} // namespace romoco