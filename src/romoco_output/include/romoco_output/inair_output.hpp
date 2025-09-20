#ifndef ROMOCO_INAIR_OUTPUT_HPP
#define ROMOCO_INAIR_OUTPUT_HPP

#include "romoco_core/output_base.hpp"

#include "romoco_utils/filters.hpp"
#include "romoco_utils/yaml_parser.hpp"

namespace romoco
{
    /**
     * @class InAirOutput
     * @brief An output class for generating desired outputs when the biped robot is in the air.
     * @ingroup group_output
     * This class extends the OutputBase class to provide specific implementations for generating desired outputs
     * when the biped robot is in the air.
     */
    class InAirOutput : public OutputBase
    {
    public:
        InAirOutput(const std::string &config_file, std::shared_ptr<romoco::robot::RobotBasePinocchio> robot);

        void UpdateOutput(const DesiredCommand &command, const double &t, const double &t_old) override;

        void ComputeActual() override;

        void ComputeDesired(const DesiredCommand &command);

        void ComputeHolonomicConstraints();

    private:
        // enum OutputIndex
        // {
        //     xLeftFoot = 0,
        //     yLeftFoot = 1,
        //     zLeftFoot = 2,
        //     yawLeftHip = 3,
        //     deltaPitchLeftFoot = 4,
        //     deltaRollLeftFoot = 5,
        //     xRightFoot = 6,
        //     yRightFoot = 7,
        //     zRightFoot = 8,
        //     yawRightHip = 9,
        //     deltaPitchRightFoot = 10,
        //     deltaRollRightFoot = 11
        // };

        Kinematics3D left_foot2base, right_foot2base;

        struct Config
        {
            Eigen::VectorXd yd_lowpass_dt_cutoff;

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
            double t;
        } updated;

        control_utilities::LowPassFilterVec lowpassyd = control_utilities::LowPassFilterVec(NAN, NAN *Eigen::VectorXd::Ones(1), 1);
    };

} // namespace romoco
#endif // ROMOCO_INAIR_OUTPUT_HPP