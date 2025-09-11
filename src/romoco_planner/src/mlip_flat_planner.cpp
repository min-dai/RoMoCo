#include "romoco_planner/mlip_flat_planner.hpp"


namespace romoco
{
MLIPFlatPlanner::MLIPFlatPlanner()
{
}

MLIPFlatPlanner::MLIPFlatPlanner(const PlannerParams &params)
{
   Init(params);
}
void MLIPFlatPlanner::Init(const PlannerParams &params)
{
   double footlength = 0.;
   MLIP_sag.Init(params.z0, params.Td, 0., params.Ts, 1, params.velx, footlength, 0.);
   MLIP_lat.Init(params.z0, params.Td, 0., params.Ts, 2, params.vely, footlength, params.stepwidth);
}
void MLIPFlatPlanner::UpdateParams(const PlannerParams &params)
{
   MLIP_sag.updateMLIP(params.z0, params.Td, 0., params.Ts);
   MLIP_lat.updateMLIP(params.z0, params.Td, 0., params.Ts);
   MLIP_sag.UpdateDesiredWalking(params.velx, 0.);
   MLIP_lat.UpdateDesiredWalking(params.vely, params.stepwidth);
}

PlannerOutput MLIPFlatPlanner::UpdatePlan(const PlannerInput& input)
{
   PlannerOutput output;

   Eigen::Vector2d x_sag = MLIP_sag.SolveLIP(input.T2imp, input.x_now.head(2));
   Eigen::Vector2d x_lat = MLIP_lat.SolveLIP(input.T2imp, input.x_now.tail(2));

   Eigen::Vector2d zero2d = Eigen::Vector2d::Zero();

   double xsw = MLIP_sag.getStepSize_P1_fixedmode(zero2d, zero2d, x_sag, 0);
   double ysw = MLIP_lat.getStepSize_P2(x_lat, input.stanceleg);

   output.footstep << xsw, ysw;
   return output;
}

} // namespace romoco