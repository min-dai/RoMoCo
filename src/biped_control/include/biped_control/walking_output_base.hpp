#pragma once
#include "biped_core/output_base.hpp"
#include "biped_utils/filters.hpp"
#include "biped_core/biped_constants.hpp"
#include "biped_utils/yaml_parser.hpp"
#include "biped_planner/planner_types.hpp"
class WalkingOutputBase : public OutputBase
{
public:
    WalkingOutputBase(std::shared_ptr<RobotBasePinocchio> robot)
        : OutputBase(robot)
    {
    }

    // Transition occurs if the center of mass (COM) velocity is low during the double support (DS) phase
    // or if the system is at the end of the single support (SS) phase.
    // virtual bool isReadyToTransition() const override;

    enum class Domain
    {
        domain_OA,
        domain_FA,
        domain_UA
    };
    struct DomainContactStatus
    {
        FootContactStatus leftC;
        FootContactStatus rightC;
        StanceStatus stance;

        void NextStance()
        {
            stance = (stance == StanceStatus::LeftStance) ? StanceStatus::RightStance : StanceStatus::LeftStance;
        }

        bool isLeftStance() const { return stance == StanceStatus::LeftStance; }
    } domain;

protected:
    PlannerOutput planner_output_;
    PlannerInput planner_input_;
    PlannerParams planner_params_;

    struct ConfigBase
    {
        YAMLParser yaml_parser;

        double dt_lowpass;
        double velX_dt_cutoff;
        double velY_dt_cutoff;

        // offset for hardware mainly
        double vx_offset;
        double vy_offset;

        double TSS, TDS;
        double znom;
        double zsw_max, zsw_neg;
        double stepWidthNominal = 0.25;

        Eigen::VectorXd bezierSwingHorizontal;
        Eigen::VectorXd bezierComVertical;

        // walking safety params
        double maxStepSize = 1, velXmax = 2, velYmax = 0.5;

        FrictionParams fric_params;

        void InitConfigBase(const std::string &config_file, const RobotType &robot_type);
    };



    // filters for desired walking beahviors from the joysticks
    control_utilities::LowPassFilter lowpass_vel_x_des_ = control_utilities::LowPassFilter(NAN, NAN);
    control_utilities::LowPassFilter lowpass_vel_y_des_ = control_utilities::LowPassFilter(NAN, NAN);

    void ComputeHolonomicConstraints();
    void ComputeFrictionConstriants(const FrictionParams &fric_params);
};