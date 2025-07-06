#include <gtest/gtest.h>
#include "biped_utils/algebraic_riccati.hpp"

TEST(AlgebraicRiccatiTest, SolveDAREReturnsPositiveDefinite) {
    Eigen::MatrixXd A(1,1), B(1,1), Q(1,1), R(1,1);
    A << 1.0;
    B << 1.0;
    Q << 1.0;
    R << 1.0;

    double tol = 1e-6;
    Eigen::MatrixXd P = solve_dare(A, B, Q, R, tol);

    EXPECT_GT(P(0,0), 0.0);  // P must be positive
}

TEST(AlgebraicRiccatiTest, SolveDLQRGainIsCorrectForScalar) {
    Eigen::MatrixXd A(1,1), B(1,1), Q(1,1), R(1,1);
    A << 1.0;
    B << 1.0;
    Q << 1.0;
    R << 1.0;

    double tol = 1e-6;
    Eigen::VectorXd K = solve_dlqr_gain(A, B, Q, R, tol);

    EXPECT_NEAR(K(0), 0.618, 1e-2);  // known result for this scalar case
}
