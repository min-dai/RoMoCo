#ifndef BIPED_PLANNER_HLIPPLANNER_HPP
#define BIPED_PLANNER_HLIPPLANNER_HPP

#include "biped_planner/flatfoot_fp_planner.hpp"
#include "biped_planner/HLIP.hpp"
#include "biped_planner/planner_types.hpp"

class HLIPPlanner : public FlatFootFPPlanner
{
public:
   HLIPPlanner();
   // Constructor that initializes the HLIPPlanner with given parameters
   HLIPPlanner(const PlannerParams& params);
   ~HLIPPlanner() override = default;
   // Initialize the HLIPPlanner with parameters
   void Init(const PlannerParams& params) override;

   void UpdateParams(const PlannerParams& params) override;
   
   PlannerOutput UpdatePlan(const PlannerInput& input) override;

protected:

   HLIP HLIP_sag;
   HLIP HLIP_lat;
};

#endif // BIPED_PLANNER_HLIPPLANNER_HPP