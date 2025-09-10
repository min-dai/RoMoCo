#ifndef BIPED_ESTIMATION_INPUT_HPP
#define BIPED_ESTIMATION_INPUT_HPP

#include <Eigen/Dense>

namespace romoco
{
/**
 * @brief Input structure for estimation modules that require kinematic data.
 *
 * This struct encapsulates per-leg kinematic quantities needed for state
 * estimation, such as foot positions, Jacobians, and contact probabilities.
 *
 * Typical usage:
 * - Provides measured or estimated foot positions and Jacobians.
 * - Supplies probabilistic contact indicators for each foot.
 * - Serves as input to estimation or filtering pipelines.
 */
struct BipedEstimationKinematicsInput
{
    /**
     * @brief Left foot Cartesian position (expected size 3x1).
     */
    Eigen::VectorXd p_left_foot;

    /**
     * @brief Right foot Cartesian position (expected size 3x1).
     */
    Eigen::VectorXd p_right_foot;

    /**
     * @brief Left foot Jacobian (expected size 3 x n_encoder_joints_per_leg).
     */
    Eigen::MatrixXd J_left_foot;

    /**
     * @brief Right foot Jacobian (expected size 3 x n_encoder_joints_per_leg).
     */
    Eigen::MatrixXd J_right_foot;

    /**
     * @brief Probability that the left foot is in contact with the ground.
     * Range: [0.0, 1.0]
     */
    double left_contact_prob = 0.0;

    /**
     * @brief Probability that the right foot is in contact with the ground.
     * Range: [0.0, 1.0]
     */
    double right_contact_prob = 0.0;

    /**
     * @brief Stream output operator for debugging/logging.
     */
    friend std::ostream &operator<<(std::ostream &os,
                                    const BipedEstimationKinematicsInput &input)
    {
        os << "BipedEstimationKinematicsInput: \n";
        os << "p_left_foot: " << input.p_left_foot.transpose() << "\n";
        os << "p_right_foot: " << input.p_right_foot.transpose() << "\n";
        os << "J_left_foot: \n"
           << input.J_left_foot << "\n";
        os << "J_right_foot: \n"
           << input.J_right_foot << "\n";
        os << "left_contact_prob: " << input.left_contact_prob << ", right_contact_prob: " << input.right_contact_prob << "\n";
        return os;
    }
};
} // namespace romoco

#endif // BIPED_ESTIMATION_INPUT_HPP
