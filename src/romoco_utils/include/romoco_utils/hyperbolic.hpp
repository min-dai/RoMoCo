#ifndef HYPERBOLIC_HPP
#define HYPERBOLIC_HPP
/**
 * @namespace romoco
 * @brief Contains utility functions for hyperbolic mathematical operations.
 *
 * This namespace provides functions for computing hyperbolic cotangent and hyperbolic secant.
 */
namespace romoco

{
   /**
    * @brief Computes the hyperbolic cotangent of a given value.
    * @param v The input value.
    * @return The hyperbolic cotangent of v.
    */
   double coth(double v);
   /**
    * @brief Computes the hyperbolic secant of a given value.
    * @param v The input value.
    * @return The hyperbolic secant of v.
    */
   double sech(double v);
} // namespace romoco

#endif // HYPERBOLIC_HPP