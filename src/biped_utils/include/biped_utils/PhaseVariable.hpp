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
    double pActual;
    double dpActual;

    Eigen::Vector2d phaseRange;

    PhaseVariable();
    void update(double time);
    void reconfigure(Eigen::Vector2d &phaseRange, double timeScale);
    Eigen::Vector2d getPhaseRange();

private:
    double timeScale;

    void calcP(double time);
    void calcTau();
    void calcDTau();
};

#endif // PHASE_VARIABLE_HPP
