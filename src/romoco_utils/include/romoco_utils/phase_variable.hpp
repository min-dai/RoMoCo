#ifndef PHASE_VARIABLE_HPP
#define PHASE_VARIABLE_HPP

#include <Eigen/Dense>

/**
 * @class PhaseVariable
 * @brief Represents a phase variable operating in the normalized range [0, 1].
 *
 * The phase variable is typically used in gait or trajectory generation contexts,
 * where it evolves with time. This class tracks elapsed time, computes the phase,
 * and provides utilities for reconfiguring and querying the range of operation.
 */
class PhaseVariable {
 public:
  /**
   * @brief Normalized phase variable in [0, 1].
   *
   * Computed from the elapsed time relative to the configured phase range.
   */
  double tau;

  /**
   * @brief Derivative of the phase variable with respect to time.
   */
  double dtau;

  /**
   * @brief Default constructor.
   *
   * Initializes the phase variable and internal state.
   */
  PhaseVariable();

  /**
   * @brief Update the phase variable based on the current time.
   * @param time Current time (s).
   */
  void Update(double time);

  /**
   * @brief Reconfigure the phase range.
   * @param phase_range Two-element vector: [start_time, end_time] (s).
   */
  void Reconfigure(Eigen::Vector2d& phase_range);

  /**
   * @brief Get the total elapsed time since the phase range began.
   * @return Time passed (s).
   */
  double time_passed() { return time_passed_; }

  /**
   * @brief Get the remaining time until the end of the phase range.
   * @return Time left (s).
   */
  double time_left() { return phase_range_(1) - time_passed_; }

  /**
   * @brief Get the configured phase range.
   * @return Two-element vector: [start_time, end_time] (s).
   */
  Eigen::Vector2d phase_range() { return phase_range_; };

 private:
  /**
   * @brief Current phase range [start_time, end_time] (s).
   */
  Eigen::Vector2d phase_range_;

  /**
   * @brief Elapsed time since the start of the phase range (s).
   */
  double time_passed_;

  /**
   * @brief Internal helper to update the elapsed time.
   * @param time Current time (s).
   */
  void UpdateTime(double time);

  /**
   * @brief Internal helper to update the phase variable and its derivative.
   */
  void UpdatePhase();
};

#endif  // PHASE_VARIABLE_HPP