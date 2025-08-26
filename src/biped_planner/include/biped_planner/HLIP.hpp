#ifndef HLIP_HPP
#define HLIP_HPP

#include <biped_types/biped_constants.hpp>
#include <Eigen/Dense>

#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <biped_utils/hyperbolic.hpp>
#include <biped_utils/algebraic_riccati.hpp>


using namespace std;

class HLIP
{

public:
    HLIP(bool useMomentum_ = true);
    void set_useDCM(bool useDCM)
    {
        useDCM_ = useDCM;
    }
    void Init(double z0, double Ts, double Td, int orbitPeriod, double vel, double stepwidth);

    int orbitPeriod = 0; // 1 is P1, 2 is P2

    struct Params
    {
        double z0, Ts, Td, T, lambda;
        double grav = 9.81;

        Eigen::Matrix2d ASS, ADS;

        Eigen::MatrixXd A_S2S = Eigen::MatrixXd::Zero(2, 2);
        Eigen::VectorXd B_S2S = Eigen::MatrixXd::Zero(2, 1);

        double velDes = 0;

        double a_DCM, b_DCM;
    } params;

    Eigen::Vector2d Kdeadbeat;
    Eigen::Vector2d Klqr;

    double klqr_DCM;

    void updateHLIP(double z0, double Ts, double Td);

    void updateDesiredWalking(double vel, double uLeftDes);

    Eigen::Vector2d getDesiredStepSizeDeadbeat(double p, double v, StanceStatus stanceLeg);

    struct P1
    {
        // desired walking state for P1 orbits
        Eigen::VectorXd Xdes = Eigen::VectorXd::Zero(2);
        double Udes = 0;
        Eigen::Vector2d K;
        Eigen::Vector2d StepX;

        Eigen::Vector2d getDeadbeatStepSize(double p, double v, double lambda);

        double DCM_des;
        double klqr_DCM;
        Eigen::Vector2d getDeadbeatStepSize_DCM(double p, double v, double lambda, double z0);

    } p1;

    struct P2
    {
        // desired walking state for P2 orbits
        Eigen::VectorXd XleftDes = Eigen::VectorXd::Zero(2);
        Eigen::VectorXd XrightDes = Eigen::VectorXd::Zero(2);
        Eigen::VectorXd Xdes = Eigen::VectorXd::Zero(2);
        double UleftDes = 0;
        double UrightDes = 0;
        double Udes = 0;

        Eigen::Vector2d K;
        Eigen::Vector2d StepX;
        Eigen::Vector2d getDeadbeatStepSize(double p, double v, double lambda, StanceStatus stanceLeg);

        double DCM_leftDes, DCM_rightDes, klqr_DCM;
        Eigen::Vector2d getDeadbeatStepSize_DCM(double p, double v, double lambda, double z0, StanceStatus stanceLeg);
    } p2;

    Eigen::Vector2d get_LIPsol(double t, Eigen::Vector2d X0);

    double getOrbitalEnergy(double p, double Ly);

    Eigen::Vector2d solve_deadbeat_gain(Eigen::Matrix2d A, Eigen::Vector2d B);

private:
    bool useDCM_ = false;
    bool useMomentum_ = true;
};

#endif // HLIP_HPP