#ifndef PLANNER_TYPES_HPP
#define PLANNER_TYPES_HPP

#include <Eigen/Dense>
#include "biped_core/biped_constants.hpp"

struct PlannerInput
{
   Eigen::Vector4d x_now; // xnow = [pCOMx, Ly, pCOMy, -Lx]
   // Eigen::Vector4d x_SSm; // predicted state at the end of SS phase
   double T2imp;
   //stance
   StanceStatus stanceleg;
   void UpdateInputLIP(const Eigen::Vector4d &x_now, const StanceStatus &stanceleg, double T2imp)
   {
      this->x_now = x_now;
      this->stanceleg = stanceleg;
      this->T2imp = T2imp;
   }
};

struct PlannerOutput
{
   Eigen::Vector2d footstep; // desired footstep position
};

struct PlannerParams
{
   double z0;        // Initial height of the center of mass
   double Ts;        // Duration of the stance phase
   double Td;        // Duration of the swing phase
   double velx;      // Desired velocity in the x direction
   double vely;      // Desired velocity in the y direction
   double stepwidth; // Desired step width

   double l; //foot curvature length for multi-domain planner

   void UpdateParamsLIP(double z0, double Ts, double Td, double velx, double vely, double stepwidth)
   {
      this->z0 = z0;
      this->Ts = Ts;
      this->Td = Td;
      this->velx = velx;
      this->vely = vely;
      this->stepwidth = stepwidth;
   }
   void UpdateParamsMLIP(double z0, double Ts, double Td, double velx, double vely, double stepwidth, double l)
   {
      UpdateParamsLIP(z0, Ts, Td, velx, vely, stepwidth);
      this->l = l;
   }

};

#endif // PLANNER_TYPES_HPP