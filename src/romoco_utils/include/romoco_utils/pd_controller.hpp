#ifndef PD_CONTROLLER_HPP
#define PD_CONTROLLER_HPP

#include <Eigen/Dense>
namespace romoco
{
/**
 * @brief A Proportional-Derivative (PD) controller for joint-level control.
 * @ingroup group_utils
 * This class provides methods for configuring and computing PD control outputs.
 * It supports two usage modes:
 *  - Compute using externally provided desired states.
 *  - Compute using internally stored desired states.
 *
 * The controller throws exceptions if used before initialization or if input
 * vectors have mismatched sizes.
 */
class PDController {
 public:
  /**
   * @brief Default constructor.
   *
   * Creates an uninitialized controller. Call Reconfigure() before Compute().
   */
  PDController();

  /**
   * @brief Construct a PD controller with gains.
   *
   * @param Kp Proportional gains (per joint).
   * @param Kd Derivative gains (per joint).
   */
  PDController(const Eigen::VectorXd& Kp, const Eigen::VectorXd& Kd);

  /**
   * @brief Construct a PD controller with gains and desired states.
   *
   * @param Kp Proportional gains (per joint).
   * @param Kd Derivative gains (per joint).
   * @param q_desired Desired joint positions.
   * @param dq_desired Desired joint velocities.
   */
  PDController(const Eigen::VectorXd& Kp, const Eigen::VectorXd& Kd,
               const Eigen::VectorXd& q_desired,
               const Eigen::VectorXd& dq_desired);

  /// Default destructor.
  ~PDController() = default;

  /**
   * @brief Reconfigure the controller with new gains.
   *
   * @param Kp New proportional gains.
   * @param Kd New derivative gains.
   */
  void Reconfigure(const Eigen::VectorXd& Kp, const Eigen::VectorXd& Kd);

  /**
   * @brief Reconfigure the controller with new gains and desired states.
   *
   * @param Kp New proportional gains.
   * @param Kd New derivative gains.
   * @param q_desired New desired joint positions.
   * @param dq_desired New desired joint velocities.
   */
  void Reconfigure(const Eigen::VectorXd& Kp, const Eigen::VectorXd& Kd,
                   const Eigen::VectorXd& q_desired,
                   const Eigen::VectorXd& dq_desired);

  /**
   * @brief Compute control output using provided desired and actual states.
   *
   * @param q_desired Desired joint positions.
   * @param dq_desired Desired joint velocities.
   * @param q_actual Current joint positions.
   * @param dq_actual Current joint velocities.
   * @return Joint torques (or generalized forces) computed by PD law.
   *
   * @throws std::runtime_error If the controller has not been initialized.
   * @throws std::invalid_argument If vector sizes do not match.
   */
  Eigen::VectorXd Compute(const Eigen::VectorXd& q_desired,
                          const Eigen::VectorXd& dq_desired,
                          const Eigen::VectorXd& q_actual,
                          const Eigen::VectorXd& dq_actual) const;

  /**
   * @brief Compute control output using internally stored desired states.
   *
   * Desired states must have been set previously through Reconfigure().
   *
   * @param q_actual Current joint positions.
   * @param dq_actual Current joint velocities.
   * @return Joint torques (or generalized forces) computed by PD law.
   *
   * @throws std::runtime_error If the controller has not been initialized.
   * @throws std::invalid_argument If vector sizes do not match.
   */
  Eigen::VectorXd Compute(const Eigen::VectorXd& q_actual,
                          const Eigen::VectorXd& dq_actual) const;

 private:
  bool is_initialized_;            ///< True if controller is configured.
  Eigen::VectorXd Kp_;             ///< Proportional gains.
  Eigen::VectorXd Kd_;             ///< Derivative gains.
  Eigen::VectorXd q_desired_;      ///< Desired joint positions.
  Eigen::VectorXd dq_desired_;     ///< Desired joint velocities.
};
}  // namespace romoco
#endif  // PD_CONTROLLER_HPP
