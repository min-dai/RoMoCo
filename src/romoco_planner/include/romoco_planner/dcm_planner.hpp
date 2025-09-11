#ifndef BIPED_PLANNER_DCMPLANNER_HPP
#define BIPED_PLANNER_DCMPLANNER_HPP

#include "romoco_planner/hlip_planner.hpp"

namespace romoco
{
class DCMPlanner : public HLIPPlanner
{
public:
   DCMPlanner() = default;
   DCMPlanner(const PlannerParams &params) : HLIPPlanner(params) {} // Constructor that initializes the HLIPPlanner with given parameters
   ~DCMPlanner() override = default;

   void Init(const PlannerParams &params) override
   {
      HLIPPlanner::Init(params);
      HLIP_sag.set_use_dcm(true);
      HLIP_lat.set_use_dcm(true);
   }
};
} // namespace romoco

#endif // BIPED_PLANNER_DCMPLANNER_HPP