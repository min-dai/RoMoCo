
#pragma once

#include "biped_planner/ro_planner_base.hpp"
#include "biped_planner/MLIP.hpp"

class MLIPPlanner : public ROPlannerBase
{
public:
   MLIPPlanner();
   MLIPPlanner(double z0, double Ts, double Td, double velx, double vely, double stepwidth);
   ~MLIPPlanner() override = default;
   void Init(double z0, double Ts, double Td, double velx, double vely, double stepwidth) override;

   void UpdateParams(double z0, double Ts, double Td, double velx, double vely, double stepwidth) override
   {
      params.update(z0, Ts, Td, velx, vely, stepwidth);
      MLIP_sag.updateMLIP(params.z0, params.Ts, params.Td);
      MLIP_lat.updateMLIP(params.z0, params.Ts, params.Td);
      MLIP_sag.updateDesiredWalking(params.velx, 0.);
      MLIP_lat.updateDesiredWalking(params.vely, params.stepwidth);
   }
   Vector2d UpdatePlan(Vector4d x_now, double T2imp, StanceStatus stanceleg) override;

private:
   PlannerInput planner_states;

   MLIP MLIP_sag;
   MLIP MLIP_lat;
};