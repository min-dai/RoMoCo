#include "romoco_planner/alip_planner.hpp"

namespace romoco
{

ALIPPlanner::ALIPPlanner()
{
}

ALIPPlanner::ALIPPlanner(const PlannerParams &params)
{
   Init(params);
}

void ALIPPlanner::Init(const PlannerParams &params)
{
   ALIP_sag.Init(params.z0, params.Ts, 1, params.velx, 0.);
   ALIP_lat.Init(params.z0, params.Ts, 2, params.vely, params.stepwidth);
}

void ALIPPlanner::UpdateParams(const PlannerParams &params)
{
   ALIP_sag.UpdateALIP(params.z0, params.Ts);
   ALIP_lat.UpdateALIP(params.z0, params.Ts);
   ALIP_sag.UpdateDesiredWalking(params.velx, 0.);
   ALIP_lat.UpdateDesiredWalking(params.vely, params.stepwidth);
}

PlannerOutput ALIPPlanner::UpdatePlan(const PlannerInput& input)
{
   PlannerOutput output;

   Eigen::Vector2d x_sag = ALIP_sag.SolveLIP(input.T2imp, input.x_now.head(2));
   Eigen::Vector2d x_lat = ALIP_lat.SolveLIP(input.T2imp, input.x_now.tail(2));

   double xsw = ALIP_sag.SolveDesiredStepSizeP1(x_sag);
   double ysw = ALIP_lat.SolveDesiredStepSizeP2(x_lat, input.stanceleg);

   output.footstep << xsw, ysw;

   return output;
}

} // namespace romoco