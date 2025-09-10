#ifndef PSEUDO_PINVERSE_HPP
#define PSEUDO_PINVERSE_HPP

#include <Eigen/Dense>

namespace romoco
{
   /**
    * @brief Compute the Moore-Penrose pseudo-inverse of a matrix.
    * @ingroup group_utils
    * @param matrix Input matrix to be pseudo-inverted.
    * @param threshold Singular values smaller than this relative to the largest singular value are set to zero.
    * @return Pseudo-inverse of the input matrix.
    * @throws std::invalid_argument if the input matrix is empty.
    */
   Eigen::MatrixXd PseudoInverse(const Eigen::MatrixXd &matrix,
                                 const double &threshold = 1e-6);

   Eigen::MatrixXd WeightedPseudoInverse(const Eigen::MatrixXd &J,
                                         const Eigen::MatrixXd &W,
                                         const double &threshold = 1e-6);
} // namespace romoco
#endif // PSEUDO_PINVERSE_HPP
