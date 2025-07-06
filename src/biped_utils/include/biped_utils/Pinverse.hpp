#ifndef PINVERSE_HPP
#define PINVERSE_HPP

#include <Eigen/Dense>


Eigen::MatrixXd PseudoInverse(const Eigen::MatrixXd &matrix,
                              const double &threshold = 1e-6);

Eigen::MatrixXd WeightedPseudoInverse(const Eigen::MatrixXd &J,
                                      const Eigen::MatrixXd &W,
                                      const double &threshold = 1e-6);

#endif // PINVERSE_HPP
