#ifndef BIPED_CORE_FRICTION_PARAMS_HPP
#define BIPED_CORE_FRICTION_PARAMS_HPP

namespace romoco
{
struct FrictionParams
{
   double frictionCoef;
   double Rot_frictionCoef;
   double Lfront, Lback;
   double W;
   double Fz_lb;
};

} // namespace romoco

#endif // BIPED_CORE_FRICTION_PARAMS_HPP
