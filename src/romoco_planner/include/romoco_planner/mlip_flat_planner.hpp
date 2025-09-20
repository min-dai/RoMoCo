
#ifndef ROMOCO_PLANNER_MLIPPLANNER_HPP
#define ROMOCO_PLANNER_MLIPPLANNER_HPP

#include "romoco_planner/flatfoot_fp_planner.hpp"
#include "romoco_planner/mlip.hpp"
#include "romoco_planner/planner_types.hpp"

namespace romoco
{
   /**
    * @class MLIPFlatPlanner
    * @brief A class for planning using the Multi-Linear Inverted Pendulum (MLIP) model.
    * @ingroup group_ro_planner
    */
   class MLIPFlatPlanner : public FlatFootFPPlanner
   {
   public:
      MLIPFlatPlanner();
      MLIPFlatPlanner(const PlannerParams &params);
      ~MLIPFlatPlanner() override = default;
      void Init(const PlannerParams &params) override;

      void UpdateParams(const PlannerParams &params) override;

      PlannerOutput UpdatePlan(const PlannerInput &input) override;

   private:
      MLIP MLIP_sag;
      MLIP MLIP_lat;
   };
} // namespace romoco
#endif // ROMOCO_PLANNER_MLIPPLANNER_HPP