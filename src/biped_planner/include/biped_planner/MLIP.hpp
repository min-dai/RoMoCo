#ifndef MLIP_HPP
#define MLIP_HPP

#include <biped_types/biped_constants.hpp>
#include <Eigen/Dense>


#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <biped_utils/hyperbolic.hpp>
#include <biped_utils/algebraic_riccati.hpp>




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

        Eigen::Matrix3d A;

        Eigen::MatrixXd A2_S2S_h2t = Eigen::MatrixXd::Zero(2, 2);
        Eigen::MatrixXd B2_S2S_h2t = Eigen::MatrixXd::Zero(2, 1);
        Eigen::MatrixXd C2_S2S_h2t = Eigen::MatrixXd::Zero(2, 1);

        Eigen::MatrixXd A2_S2S_flat = Eigen::MatrixXd::Zero(2, 2);
        Eigen::MatrixXd B2_S2S_flat = Eigen::MatrixXd::Zero(2, 1);
        Eigen::MatrixXd C2_S2S_flat = Eigen::MatrixXd::Zero(2, 1);

        Eigen::MatrixXd A2_S2S_t2h = Eigen::MatrixXd::Zero(2, 2);
        Eigen::MatrixXd B2_S2S_t2h = Eigen::MatrixXd::Zero(2, 1);
        Eigen::MatrixXd C2_S2S_t2h = Eigen::MatrixXd::Zero(2, 1);

        double velDes = 0;

        Eigen::Matrix3d getAconvT(double T);
        void getABC_S2S(double l, Eigen::MatrixXd &As2s, Eigen::MatrixXd &Bs2s, Eigen::MatrixXd &Cs2s);
    } params;

    Eigen::Vector2d Kdeadbeat_h2t, Kdeadbeat_flat, Kdeadbeat_t2h;
    Eigen::Vector2d Klqr_h2t, Klqr_flat, Klqr_t2h;

    void updateMLIP(double z0, double TOA, double TFA, double TUA);

    void updateDesiredWalking(double vel, double stepWidth);

    void solveXdesFAminus_XdesFAplus(double l, Eigen::Vector2d Xdes, double Udes, Eigen::Vector2d &XdesFAminus, Eigen::Vector2d &XdesFAplus);

    void solveXdesUAminus_XdesUAplus(double l, Eigen::Vector2d Xdes, double Udes, Eigen::Vector2d &XdesFAminus, Eigen::Vector2d &XdesFAplus);

    Eigen::Vector2d getDesiredStepSizeDeadbeat(Eigen::Vector2d X, bool isFlatFoot, int stanceLeg);

    struct P1
    {
        // desired walking state for P1 orbits
        Eigen::Vector2d Xdes_h2t, Xdes_flat, Xdes_t2h;
        double Udes_h2t, Udes_flat, Udes_t2h;

        Eigen::Vector2d XdesFAminus_h2t, XdesFAminus_flat, XdesFAminus_t2h;
        Eigen::Vector2d XdesFAplus_h2t, XdesFAplus_flat, XdesFAplus_t2h;

        Eigen::Vector2d K_h2t, K_flat, K_t2h;
        Eigen::Vector2d StepX;

        double Ku = 0; //.1;

        int mode;

        bool isFlatFoot;

    } p1;

    double getStepSize_P1(Eigen::Vector2d Xtoe, Eigen::Vector2d Xheel, Eigen::Vector2d Xmid, bool is_mode_fixed, int &mode);

    double getStepSize_P1_varimode(Eigen::Vector2d Xtoe, Eigen::Vector2d Xheel, Eigen::Vector2d Xmid, int &mode);

    double getStepSize_P1_fixedmode(Eigen::Vector2d Xtoe, Eigen::Vector2d Xheel, Eigen::Vector2d Xmid, int mode, double deltau_prev);

    struct P2
    {
        // desired walking state for P2 orbits
        Eigen::VectorXd XleftDes = Eigen::VectorXd::Zero(2);
        Eigen::VectorXd XrightDes = Eigen::VectorXd::Zero(2);
        Eigen::VectorXd Xdes = Eigen::VectorXd::Zero(2);
        double UleftDes = 0;
        double UrightDes = 0;
        double Udes = 0;
        double sigma2 = 0;
        double d2 = 0;
        Eigen::Vector2d K;
        Eigen::Vector2d StepX;

        double Ku = 0; //.1;

        Eigen::Vector2d XdesFAminus_left, XdesFAminus_right;
        Eigen::Vector2d XdesFAplus_left, XdesFAplus_right;

    } p2;
    double getStepSize_P2(Eigen::Vector2d X, StanceStatus stanceLegIdx, double deltau_prev);

    double solve_Ts();

    Eigen::Vector2d get_MLIPsol2(double t, Eigen::Vector3d X0, double dpzmp);
    Eigen::Vector3d get_MLIPsol3(double t, Eigen::Vector3d X0, double dpzmp);

    double getOrbitalEnergy(double p, double Ly);

    Eigen::Vector2d solve_deadbeat_gain(Eigen::Matrix2d A, Eigen::Vector2d B);
};

#endif // MLIP_HPP