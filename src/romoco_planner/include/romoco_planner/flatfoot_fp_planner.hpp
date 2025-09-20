#pragma once
#include <Eigen/Dense>

#include "romoco_planner/planner_types.hpp"

namespace romoco
{
   /**
    * @class FlatFootFPPlanner
    * @brief A base class for flat-footed foot placement planners.
    * @ingroup group_ro_planner
    *
    * This abstract class defines the interface for flat-footed foot placement planners.
    * It provides methods for initialization, parameter updates, and plan updates.
    * Derived classes must implement these methods to provide specific planning algorithms.
    */

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