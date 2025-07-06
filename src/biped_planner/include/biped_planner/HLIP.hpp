#ifndef HLIP_HPP
#define HLIP_HPP

#include <biped_core/biped_constants.hpp>
#include <Eigen/Dense>

#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <biped_utils/hyperbolic.hpp>
#include <biped_utils/algebraic_riccati.hpp>

using namespace Eigen;
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

        Matrix2d ASS, ADS;

        MatrixXd A_S2S = MatrixXd::Zero(2, 2);
        VectorXd B_S2S = MatrixXd::Zero(2, 1);

        double velDes = 0;

        double a_DCM, b_DCM;
    } params;

    Vector2d Kdeadbeat;
    Vector2d Klqr;

    double klqr_DCM;

    void updateHLIP(double z0, double Ts, double Td);

    void updateDesiredWalking(double vel, double uLeftDes);

    Vector2d getDesiredStepSizeDeadbeat(double p, double v, StanceStatus stanceLeg);

    struct P1
    {
        // desired walking state for P1 orbits
        VectorXd Xdes = VectorXd::Zero(2);
        double Udes = 0;
        Vector2d K;
        Vector2d StepX;

        Vector2d getDeadbeatStepSize(double p, double v, double lambda);

        double DCM_des;
        double klqr_DCM;
        Vector2d getDeadbeatStepSize_DCM(double p, double v, double lambda, double z0);

    } p1;

    struct P2
    {
        // desired walking state for P2 orbits
        VectorXd XleftDes = VectorXd::Zero(2);
        VectorXd XrightDes = VectorXd::Zero(2);
        VectorXd Xdes = VectorXd::Zero(2);
        double UleftDes = 0;
        double UrightDes = 0;
        double Udes = 0;

        Vector2d K;
        Vector2d StepX;
        Vector2d getDeadbeatStepSize(double p, double v, double lambda, StanceStatus stanceLeg);

        double DCM_leftDes, DCM_rightDes, klqr_DCM;
        Vector2d getDeadbeatStepSize_DCM(double p, double v, double lambda, double z0, StanceStatus stanceLeg);
    } p2;

    Vector2d get_LIPsol(double t, Vector2d X0);

    double getOrbitalEnergy(double p, double Ly);

    Vector2d solve_deadbeat_gain(Matrix2d A, Vector2d B);

private:
    bool useDCM_ = false;
    bool useMomentum_ = true;
};

#endif // HLIP_HPP