#ifndef MLIP_HPP
#define MLIP_HPP

#include <biped_core/biped_constants.hpp>
#include <Eigen/Dense>


#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <biped_utils/hyperbolic.hpp>
#include <biped_utils/algebraic_riccati.hpp>



using namespace Eigen;
using namespace std;

class MLIP
{

public:
    MLIP(bool useMomentum_=true);
    void Init(double z0, double TOA, double TFA, double TUA, int orbitPeriod, double vel, double footlength, double stepwidth);

    int orbitPeriod = 0; // 1 is P1, 2 is P2

    int mode_heel2toe = 1;
    int mode_flat = 0;
    int mode_toe2heel = -1;

    

    struct Params
    {
        bool useMomentum = true;
        

        double footlength;

        double l_heel2toe;
        double l_flat = 0;
        double l_toe2heel;

        double z0, TOA, TFA, TUA, T, lambda;
        double grav = 9.81;

        Matrix3d A;

        MatrixXd A2_S2S_h2t = MatrixXd::Zero(2, 2);
        MatrixXd B2_S2S_h2t = MatrixXd::Zero(2, 1);
        MatrixXd C2_S2S_h2t = MatrixXd::Zero(2, 1);

        MatrixXd A2_S2S_flat = MatrixXd::Zero(2, 2);
        MatrixXd B2_S2S_flat = MatrixXd::Zero(2, 1);
        MatrixXd C2_S2S_flat = MatrixXd::Zero(2, 1);

        MatrixXd A2_S2S_t2h = MatrixXd::Zero(2, 2);
        MatrixXd B2_S2S_t2h = MatrixXd::Zero(2, 1);
        MatrixXd C2_S2S_t2h = MatrixXd::Zero(2, 1);

        double velDes = 0;

        Matrix3d getAconvT(double T);
        void getABC_S2S(double l, MatrixXd &As2s, MatrixXd &Bs2s, MatrixXd &Cs2s);
    } params;

    Vector2d Kdeadbeat_h2t, Kdeadbeat_flat, Kdeadbeat_t2h;
    Vector2d Klqr_h2t, Klqr_flat, Klqr_t2h;

    void updateMLIP(double z0, double TOA, double TFA, double TUA);

    void updateDesiredWalking(double vel, double stepWidth);

    void solveXdesFAminus_XdesFAplus(double l, Vector2d Xdes, double Udes, Vector2d &XdesFAminus, Vector2d &XdesFAplus);

    void solveXdesUAminus_XdesUAplus(double l, Vector2d Xdes, double Udes, Vector2d &XdesFAminus, Vector2d &XdesFAplus);

    Vector2d getDesiredStepSizeDeadbeat(Vector2d X, bool isFlatFoot, int stanceLeg);

    struct P1
    {
        // desired walking state for P1 orbits
        Vector2d Xdes_h2t, Xdes_flat, Xdes_t2h;
        double Udes_h2t, Udes_flat, Udes_t2h;

        Vector2d XdesFAminus_h2t, XdesFAminus_flat, XdesFAminus_t2h;
        Vector2d XdesFAplus_h2t, XdesFAplus_flat, XdesFAplus_t2h;

        Vector2d K_h2t, K_flat, K_t2h;
        Vector2d StepX;

        double Ku = 0; //.1;

        int mode;

        bool isFlatFoot;

    } p1;

    double getStepSize_P1(Vector2d Xtoe, Vector2d Xheel, Vector2d Xmid, bool is_mode_fixed, int &mode);

    double getStepSize_P1_varimode(Vector2d Xtoe, Vector2d Xheel, Vector2d Xmid, int &mode);

    double getStepSize_P1_fixedmode(Vector2d Xtoe, Vector2d Xheel, Vector2d Xmid, int mode, double deltau_prev);

    struct P2
    {
        // desired walking state for P2 orbits
        VectorXd XleftDes = VectorXd::Zero(2);
        VectorXd XrightDes = VectorXd::Zero(2);
        VectorXd Xdes = VectorXd::Zero(2);
        double UleftDes = 0;
        double UrightDes = 0;
        double Udes = 0;
        double sigma2 = 0;
        double d2 = 0;
        Vector2d K;
        Vector2d StepX;

        double Ku = 0; //.1;

        Vector2d XdesFAminus_left, XdesFAminus_right;
        Vector2d XdesFAplus_left, XdesFAplus_right;

    } p2;
    double getStepSize_P2(Vector2d X, StanceStatus stanceLegIdx, double deltau_prev);

    double solve_Ts();

    Vector2d get_MLIPsol2(double t, Vector3d X0, double dpzmp);
    Vector3d get_MLIPsol3(double t, Vector3d X0, double dpzmp);

    double getOrbitalEnergy(double p, double Ly);

    Vector2d solve_deadbeat_gain(Matrix2d A, Vector2d B);
};

#endif // MLIP_HPP