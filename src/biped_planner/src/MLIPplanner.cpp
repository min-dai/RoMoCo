#include "biped_planner/MLIPPlanner.hpp"

MLIPPlanner::MLIPPlanner()
{
}

MLIPPlanner::MLIPPlanner(double z0, double Ts, double Td, double velx, double vely, double stepwidth)
{
   Init(z0, Ts, Td, velx, vely, stepwidth);
}

void MLIPPlanner::Init(double z0, double Ts, double Td, double velx, double vely, double stepwidth)
{
   MLIP_sag.Init(z0, Ts, Td, 1, velx, 0.);
   MLIP_lat.Init(z0, Ts, Td, 2, vely, stepwidth);
}

Vector2d MLIPPlanner::UpdatePlan(Vector4d x_now, double T2imp, StanceStatus stanceleg)
{
   

   Vector2d plan;

   Vector2d x_sag = MLIP_sag.get_LIPsol(T2imp, x_now.head(2));
   Vector2d x_lat = MLIP_lat.get_LIPsol(T2imp, x_now.tail(2));
   // std::cout << "x_now: " << x_now.transpose() << std::endl;
   // std::cout << "x_sag: " << x_sag.transpose() << std::endl;
   // std::cout << "x_lat: " << x_lat.transpose() << std::endl;

   Vector2d xsw = MLIP_lat.getDesiredStepSizeDeadbeat(x_lat(0), x_lat(1), stanceleg);
   Vector2d ysw = MLIP_lat.getDesiredStepSizeDeadbeat(x_lat(0), x_lat(1), stanceleg);
   
   plan << xsw(0), ysw(0);
   return plan;
}

