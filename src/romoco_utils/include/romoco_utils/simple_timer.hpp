#ifndef SIMPLE_TIMER_H_
#define SIMPLE_TIMER_H_

#include <chrono>

namespace romoco
/**
 * @class SimpleTimer
 * @ingroup group_utils
 * @brief Utility class for measuring elapsed time and time intervals.
 *
 * This class provides a simple interface for tracking time using a steady clock.
 * It allows you to reset the timer, update time stamps, and query elapsed time
 * since the timer was started or the time between updates.
 *
 * Usage:
 * - Call Reset() to initialize or restart the timer.
 * - Call Tick() to update the current time and shift the previous time stamp.
 * - Use t0(), t_now(), and t_old() to access time stamps in seconds.
 * - Use ElapsedSinceStart() to get the time elapsed since the timer was started.
 * - Use DeltaTime() to get the time difference between the last two Tick() calls.
 */
{
class SimpleTimer {
 public:
  SimpleTimer() { Reset(); }

  // Reset all time stamps to now.
  void Reset() {
    auto now = Clock::now();
    t0_ = ToSeconds(now);
    t_old_ = t0_;
    t_now_ = t0_;
  }

  // Update t_now and shift t_old.
  void Tick() {
    t_old_ = t_now_;
    t_now_ = ToSeconds(Clock::now());
  }

  // Accessors (seconds).
  double t0() const { return t0_; }
  double t_now() const { return t_now_; }
  double t_old() const { return t_old_; }

  // Seconds since start.
  double ElapsedSinceStart() const { return t_now_ - t0_; }

  // Seconds between last Tick() calls.
  double DeltaTime() const { return t_now_ - t_old_; }

 private:
  using Clock = std::chrono::steady_clock;

  static double ToSeconds(const Clock::time_point& tp) {
    return std::chrono::duration<double>(tp.time_since_epoch()).count();
  }

  double t0_;
  double t_now_;
  double t_old_;
};
}  // namespace romoco
#endif  // SIMPLE_TIMER_H_
