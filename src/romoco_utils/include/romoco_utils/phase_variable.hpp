/*
 * @brief A class for the implementation of a phase variable, which operates between [0,1].
 * @author Jenna Reher (jreher@caltech.edu)
 */

#ifndef PHASE_VARIABLE_HPP
#define PHASE_VARIABLE_HPP

#include <Eigen/Dense>

class PhaseVariable
{
public:
    double tau;
    double dtau;

    PhaseVariable();
    void Update(double time);
    void Reconfigure(Eigen::Vector2d &phase_range);

    double time_passed() { return time_passed_; }
    double time_left() { return phase_range_(1) - time_passed_; }
    Eigen::Vector2d phase_range() { return phase_range_; };

private:

    Eigen::Vector2d phase_range_;
    double time_passed_;
    void UpdateTime(double time);
    void UpdatePhase();
};

#endif // PHASE_VARIABLE_HPP
