
#include <romoco_utils/phase_variable.hpp>

PhaseVariable::PhaseVariable()
{
    this->tau = 0.0;
    this->dtau = 0.0;
    this->time_passed_ = 0.0;
    this->phase_range_ << 0.0, 1.0;
}

void PhaseVariable::Reconfigure(Eigen::Vector2d &phase_range)
{
    this->phase_range_ << phase_range;
}

void PhaseVariable::Update(double time)
{
    this->UpdateTime(time);
    this->UpdatePhase();
}

void PhaseVariable::UpdateTime(double time)
{
    this->time_passed_ = this->phase_range_(0) + time;
}

void PhaseVariable::UpdatePhase()
{
    this->tau = (this->time_passed_ - this->phase_range_(0)) / (this->phase_range_(1) - this->phase_range_(0));
    this->dtau = 1.0 / (this->phase_range_(1) - this->phase_range_(0));
}
