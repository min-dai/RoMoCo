#include <biped_utils/Pinverse.hpp>
#include <Eigen/QR>

Eigen::MatrixXd PseudoInverse(const Eigen::MatrixXd &matrix,
                              const double &threshold)
{
  Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(matrix.rows(),
                                                              matrix.cols());
  cod.setThreshold(threshold);
  cod.compute(matrix);
  return cod.pseudoInverse();
}

Eigen::MatrixXd WeightedPseudoInverse(const Eigen::MatrixXd &J,
                                      const Eigen::MatrixXd &W,
                                      const double &threshold)
{
  // Compute the inverse of W
  Eigen::MatrixXd W_inv = W.inverse();

  // Compute the intermediate lambda matrix
  Eigen::MatrixXd lambda = J * W_inv * J.transpose();

  // Compute the pseudo-inverse of lambda
  Eigen::MatrixXd lambda_inv = PseudoInverse(lambda, threshold);

  // Compute the weighted pseudo-inverse of J
  return W_inv * J.transpose() * lambda_inv;
}
