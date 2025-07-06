#pragma once
#include <Eigen/Dense>
#include "biped_core/biped_constants.hpp"
#include "biped_planner/planner_types.hpp"
class FlatFootFPPlanner
{
public:
   FlatFootFPPlanner() {};
   virtual ~FlatFootFPPlanner() = default;

   virtual void Init(const PlannerParams &params) = 0;

   virtual void UpdateParams(const PlannerParams &params) = 0;

   virtual PlannerOutput UpdatePlan(const PlannerInput &input) = 0;
};