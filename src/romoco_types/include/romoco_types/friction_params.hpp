#ifndef BIPED_CORE_FRICTION_PARAMS_HPP
#define BIPED_CORE_FRICTION_PARAMS_HPP

struct FrictionParams
{
   double frictionCoef;
   double Rot_frictionCoef;
   double Lfront, Lback;
   double W;
   double Fz_lb;
};

#endif // BIPED_CORE_FRICTION_PARAMS_HPP
