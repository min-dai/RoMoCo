#ifndef HLIP_HPP
#define HLIP_HPP

#include <romoco_core/biped_constants.hpp>
#include <Eigen/Dense>
#include "romoco_planner/lip_base.hpp"

namespace romoco
{
    /**
     * @class HLIP
     * @brief A class for solving the Hybrid Linear Inverted Pendulum (HLIP) model.
     * @ingroup group_ro_planner
     */
    class HLIP : public LIPBase
    {

    public:
        HLIP(bool useMomentum_ = true);
        void set_use_dcm(bool use_dcm)
        {
            use_dcm_ = use_dcm;
        }
        void Init(double z0, double Ts, double Td, int orbitPeriod, double vel, double stepwidth);

        int orbitPeriod = 0; // 1 is P1, 2 is P2

        struct Params
        {

            Eigen::Matrix2d ASS, ADS;

            Eigen::MatrixXd A_S2S = Eigen::MatrixXd::Zero(2, 2);
            Eigen::VectorXd B_S2S = Eigen::MatrixXd::Zero(2, 1);

            double velDes = 0;

            double a_DCM, b_DCM;

        } params;

        Eigen::Vector2d Kdeadbeat;
        Eigen::Vector2d Klqr;

        double klqr_DCM;

        void UpdateHLIP(double z0, double Ts, double Td);

        void UpdateDesiredWalking(double vel, double uLeftDes);

        double SolveDesiredStepSize(Eigen::Vector2d x_preimpact, StanceStatus stanceLeg);

        struct P1
        {
            // desired walking state for P1 orbits
            Eigen::VectorXd Xdes = Eigen::VectorXd::Zero(2);
            double Udes = 0;
            Eigen::Vector2d K;
            Eigen::Vector2d StepX;

            double DCM_des;
            double klqr_DCM;

        } p1;

        double SolveDesiredStepSizeP1(Eigen::Vector2d state);
        double SolveDesiredStepSizeDCMP1(Eigen::Vector2d state);

        double SolveDesiredStepSizeP2(Eigen::Vector2d state, StanceStatus stanceLeg);
        double SolveDesiredStepSizeDCMP2(Eigen::Vector2d state, StanceStatus stanceLeg);

        struct P2
        {
            // desired walking state for P2 orbits
            Eigen::VectorXd XleftDes = Eigen::VectorXd::Zero(2);
            Eigen::VectorXd XrightDes = Eigen::VectorXd::Zero(2);

            double UleftDes = 0;
            double UrightDes = 0;

            Eigen::Vector2d K;
            Eigen::Vector2d StepX;

            double DCM_leftDes, DCM_rightDes, klqr_DCM;
        } p2;

    private:
        bool use_dcm_ = false;
        double Ts_, Td_, T_;
    };
} // namespace romoco
#endif // HLIP_HPP