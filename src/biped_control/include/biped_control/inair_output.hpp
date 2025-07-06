#pragma once

#include "biped_core/output_base.hpp"

#include "biped_utils/filters.hpp"
#include "biped_utils/yaml_parser.hpp"
#include "biped_command/radio_slider_map.hpp"
class InAirOutput : public OutputBase
{
public:
    InAirOutput(const std::string &config_file, std::shared_ptr<RobotBasePinocchio> robot);

    void UpdateOutput(const Eigen::VectorXd &reference, const double &t, const double &t_old) override;

    void ComputeActual() override;

    void ComputeDesired(const Eigen::VectorXd &reference);

    void ComputeHolonomicConstraints();

private:
    enum OutputIndex
    {
        xLeftFoot = 0,
        yLeftFoot = 1,
        zLeftFoot = 2,
        yawLeftHip = 3,
        deltaPitchLeftFoot = 4,
        deltaRollLeftFoot = 5,
        xRightFoot = 6,
        yRightFoot = 7,
        zRightFoot = 8,
        yawRightHip = 9,
        deltaPitchRightFoot = 10,
        deltaRollRightFoot = 11
    };

    int nY = 12;

    Kinematics3D left_foot2base, right_foot2base;

    struct Config
    {
        VectorXd yd_lowpass_dt_cutoff;

        double dt_lowpass = 0.001;

        double swingZ_lb = 0.2;
        double swingZ_ub = 0.8;
        double swingX_backward = -0.;
        double swingX_forward = 0.;
        double swingY_inner = 0.0;
        double swingY_outer = 0.0;
        double yaw_lb = -0.1;
        double yaw_ub = 0.1;
        double pitch_lb = -0.1;
        double pitch_ub = 0.1;
        double roll_lb = -0.1;
        double roll_ub = 0.1;

        YAMLParser yaml_parser;

        void Init(RobotType robot_type);
    } config;

    struct Updated
    {
        bool isInitialized = false;
        // Eigen::VectorXd yd_prev = VectorXd::Zero(12);
        // Eigen::VectorXd dyd_prev = VectorXd::Zero(12);
        double t;
    } updated;

    control_utilities::LowPassFilterVec lowpassyd = control_utilities::LowPassFilterVec(NAN, NAN *VectorXd::Ones(1), 1);
};