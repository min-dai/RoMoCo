
#include <biped_utils/PhaseVariable.hpp>

PhaseVariable::PhaseVariable()
{
    this->tau = 0.0;
    this->dtau = 0.0;
    this->pActual = 0.0;
    this->dpActual = 0.0;
    this->phaseRange << 0.0, 1.0;
}

void PhaseVariable::reconfigure(Eigen::Vector2d &phaseRange, double timeScale = 1.0)
{
    this->phaseRange << phaseRange;
    this->timeScale = timeScale;
}

void PhaseVariable::update(double time)
{
    this->calcP(time);
    this->calcTau();
    this->calcDTau();
}

void PhaseVariable::calcP(double time)
{
    this->pActual = time * this->timeScale;
    this->dpActual = this->timeScale;
}

void PhaseVariable::calcTau()
{
    this->tau = (this->pActual - this->phaseRange(0)) / (this->phaseRange(1) - this->phaseRange(0));
}

void PhaseVariable::calcDTau()
{
    this->dtau = this->dpActual / (this->phaseRange(1) - this->phaseRange(0));
}

Eigen::Vector2d PhaseVariable::getPhaseRange()
{
    return this->phaseRange;
}
