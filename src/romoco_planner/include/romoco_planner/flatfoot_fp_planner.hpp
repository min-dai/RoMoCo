#pragma once
#include <Eigen/Dense>

#include "romoco_planner/planner_types.hpp"

namespace romoco
{

class FlatFootFPPlanner
{
public:
   FlatFootFPPlanner() {};
   virtual ~FlatFootFPPlanner() = default;

   virtual void Init(const PlannerParams &params) = 0;

   virtual void UpdateParams(const PlannerParams &params) = 0;

   virtual PlannerOutput UpdatePlan(const PlannerInput &input) = 0;
};

} // namespace romoco