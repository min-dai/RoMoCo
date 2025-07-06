#include <biped_utils/hyperbolic.hpp>

double coth(double v)
{
    return 1 / tanh(v);
}

double sech(double v)
{
    return 1 / cosh(v);
}
