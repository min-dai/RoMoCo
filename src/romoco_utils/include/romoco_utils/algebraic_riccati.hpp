#ifndef ALGEBRAIC_RICCATI_HPP
#define ALGEBRAIC_RICCATI_HPP

#include <Eigen/Dense>

namespace romoco{
/**
 * @brief Solve the discrete algebraic Riccati equation (DARE).
 * @ingroup group_utils
 *
 * System:
 *   x_{k+1} = A x_k + B u_k
 * Cost:
 *   J = sum_{k=0..∞} (x_k^T Q x_k + u_k^T R u_k)
 *
 * Computes the stabilizing symmetric solution P of
 *   P = Aᵀ P A - Aᵀ P B (R + Bᵀ P B)⁻¹ Bᵀ P A + Q
 *
 * @param A (n×n) state matrix.
 * @param B (n×m) input matrix.
 * @param Q (n×n) state cost (PSD).
 * @param R (m×m) input cost (PD).
 * @param tol convergence tolerance (default 1e-9).
 * @return P (n×n) symmetric solution.
 * @throws std::invalid_argument on size mismatch or invalid costs.
 * @throws std::runtime_error if the fixed-point iteration fails to converge.
 */
Eigen::MatrixXd SolveDare(const Eigen::Ref<const Eigen::MatrixXd>& A, const Eigen::Ref<const Eigen::MatrixXd>& B, const Eigen::Ref<const Eigen::MatrixXd>& Q, const Eigen::Ref<const Eigen::MatrixXd>& R, double tol);
/**
 * @brief Compute the discrete-time LQR gain K for (A,B,Q,R).
 * @ingroup group_utils
 *
 * Uses SolveDare() to compute P, then:
 *   K = (R + Bᵀ P B)⁻¹ Bᵀ P A
 *
 * @param A (n×n) state matrix.
 * @param B (n×m) input matrix.
 * @param Q (n×n) state cost (PSD).
 * @param R (m×m) input cost (PD).
 * @param tol convergence tolerance passed to SolveDare (default 1e-9).
 * @return K (m×n) optimal feedback gain.
 */
Eigen::VectorXd SolveDlqrGain(const Eigen::Ref<const Eigen::MatrixXd>& A, const Eigen::Ref<const Eigen::MatrixXd>& B, const Eigen::Ref<const Eigen::MatrixXd>& Q, const Eigen::Ref<const Eigen::MatrixXd>& R, double tol);
} // namespace romoco
#endif // ALGEBRAIC_RICCATI_HPP
