#include "biped_planner/HLIPplanner.hpp"

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
   HLIP_sag.updateHLIP(params.z0, params.Ts, params.Td);
   HLIP_lat.updateHLIP(params.z0, params.Ts, params.Td);
   HLIP_sag.updateDesiredWalking(params.velx, 0.);
   HLIP_lat.updateDesiredWalking(params.vely, params.stepwidth);
}

PlannerOutput HLIPPlanner::UpdatePlan(const PlannerInput& input)
{
   PlannerOutput output;

   Vector2d x_sag = HLIP_sag.get_LIPsol(input.T2imp, input.x_now.head(2));
   Vector2d x_lat = HLIP_lat.get_LIPsol(input.T2imp, input.x_now.tail(2));

   Vector2d xsw = HLIP_sag.getDesiredStepSizeDeadbeat(x_sag(0), x_sag(1), input.stanceleg);
   Vector2d ysw = HLIP_lat.getDesiredStepSizeDeadbeat(x_lat(0), x_lat(1), input.stanceleg);

   output.footstep << xsw(0), ysw(0);

   return output;
}