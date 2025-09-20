#ifndef BIPED_PLANNER_ALIPPLANNER_HPP
#define BIPED_PLANNER_ALIPPLANNER_HPP

#include "romoco_planner/flatfoot_fp_planner.hpp"
#include "romoco_planner/alip.hpp"
#include "romoco_planner/planner_types.hpp"

namespace romoco
{
   /**
    * @class ALIPPlanner
    * @brief A class for planning using the Augmented Linear Inverted Pendulum (ALIP) model.
    * @ingroup group_ro_planner
    */
   class ALIPPlanner : public FlatFootFPPlanner
   {
   public:
      ALIPPlanner();
      // Constructor that initializes the ALIPPlanner with given parameters
      ALIPPlanner(const PlannerParams &params);
      ~ALIPPlanner() override = default;
      // Initialize the ALIPPlanner with parameters
      void Init(const PlannerParams &params) override;

      void UpdateParams(const PlannerParams &params) override;

      PlannerOutput UpdatePlan(const PlannerInput &input) override;

   protected:
      ALIP ALIP_sag;
      ALIP ALIP_lat;
   };
} // namespace romoco

#endif // BIPED_PLANNER_ALIPPLANNER_HPP