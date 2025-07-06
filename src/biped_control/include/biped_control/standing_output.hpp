#pragma once

#include "biped_core/output_base.hpp"

#include "biped_utils/filters.hpp"
#include "biped_utils/yaml_parser.hpp"

#include "biped_command/radio_slider_map.hpp"
class StandingOutput : public OutputBase
{
public:
    StandingOutput(const std::string &config_file, std::shared_ptr<RobotBasePinocchio> robot);

    bool isReadyToTransition() const override
    {
        return this->updated.readyToTransition;
    }

    void UpdateOutput(const Eigen::VectorXd &radio, const double &t, const double &t_old) override;

private:
    Contact contact;

    void ComputeActual() override;

    void ComputeDesired(const Eigen::VectorXd &reference);

    void ComputeHolonomicConstraints();

    void ComputeFrictionConstriants();

    int nY;

    enum OutputIndex
    {
        xCOM = 0,
        yCOM = 1,
        zCOM = 2,
        deltaYaw = 3,
        pitch = 4,
        roll = 5
    };
    Kinematics3D com2supportbase;
    Kinematics3D deltaYaw_cross;
    Kinematics3D leftMB, rightMB;

    struct Config
    {
        VectorXd yd_lowpass_dt_cutoff;

        double dt_lowpass = 0.001;

        double stand2step_y_offset;

        double x_offset;
        double x_range;
        double y_range;
        double z_lb;
        double z_ub;

        double pitch_range, roll_range, yaw_range;

        YAMLParser yaml_parser;

        FrictionParams fric_params;

        void Init(RobotType robot_type);
    } config;

    struct Updated
    {
        bool isInitialized = false;
        bool readyToTransition = false;

        bool queueTransition = false;

        double initial_height = 0.;
        double t;
    } updated;

    control_utilities::LowPassFilterVec lowpassyd = control_utilities::LowPassFilterVec(NAN, NAN *VectorXd::Ones(1), 1);
};
