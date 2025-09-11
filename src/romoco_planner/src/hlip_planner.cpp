#include "romoco_planner/hlip_planner.hpp"

namespace romoco
{

HLIPPlanner::HLIPPlanner()
{
}

HLIPPlanner::HLIPPlanner(const PlannerParams &params)
{
   Init(params);
}

void HLIPPlanner::Init(const PlannerParams &params)
{
   HLIP_sag.Init(params.z0, params.Ts, params.Td, 1, params.velx, 0.);
   HLIP_lat.Init(params.z0, params.Ts, params.Td, 2, params.vely, params.stepwidth);
}

void HLIPPlanner::UpdateParams(const PlannerParams &params)
{
   HLIP_sag.UpdateHLIP(params.z0, params.Ts, params.Td);
   HLIP_lat.UpdateHLIP(params.z0, params.Ts, params.Td);
   HLIP_sag.UpdateDesiredWalking(params.velx, 0.);
   HLIP_lat.UpdateDesiredWalking(params.vely, params.stepwidth);
}

PlannerOutput HLIPPlanner::UpdatePlan(const PlannerInput& input)
{
   PlannerOutput output;

   Eigen::Vector2d x_sag = HLIP_sag.SolveLIP(input.T2imp, input.x_now.head(2));
   Eigen::Vector2d x_lat = HLIP_lat.SolveLIP(input.T2imp, input.x_now.tail(2));

   double xsw = HLIP_sag.SolveDesiredStepSize(x_sag, input.stanceleg);
   double ysw = HLIP_lat.SolveDesiredStepSize(x_lat, input.stanceleg);

   output.footstep << xsw, ysw;

   return output;
}

} // namespace romoco