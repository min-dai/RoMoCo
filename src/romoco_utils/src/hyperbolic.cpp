#include <romoco_utils/hyperbolic.hpp>
#include <cmath>
namespace romoco
{
double coth(double v)
{
    return 1 / tanh(v);
}

double sech(double v)
{
    return 1 / cosh(v);
}

} // namespace romoco