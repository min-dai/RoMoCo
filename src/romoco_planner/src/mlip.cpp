

#include "romoco_planner/mlip.hpp"
#include <romoco_utils/algebraic_riccati.hpp>
#include <unsupported/Eigen/MatrixFunctions> //matrix exponential
#include <iostream>
namespace romoco
{

    MLIP::MLIP(bool useMomentum) : LIPBase(useMomentum) {
                                   };

    void MLIP::Init(double z0, double TOA, double TFA, double TUA, int orbitPeriod, double vel, double footlength, double stepwidth = 0)
    {
        this->orbitPeriod = orbitPeriod;
        params.footlength = footlength;
        updateMLIP(z0, TOA, TFA, TUA);
        UpdateDesiredWalking(vel, stepwidth);
    }

    void MLIP::updateMLIP(double z0, double TOA, double TFA, double TUA)
    {
        z0_ = z0;

        params.TOA = TOA;
        params.TFA = TFA;
        params.TUA = TUA;
        params.T = TOA + TFA + TUA;

        params.l_heel2toe = params.footlength;
        params.l_toe2heel = -params.footlength;
        params.l_flat = 0;

        params.A = A3();
        params.getABC_S2S(use_momentum_, params.l_heel2toe, lambda(), z0_, params.A2_S2S_h2t, params.B2_S2S_h2t, params.C2_S2S_h2t);
        params.getABC_S2S(use_momentum_, params.l_flat, lambda(), z0_, params.A2_S2S_flat, params.B2_S2S_flat, params.C2_S2S_flat);
        params.getABC_S2S(use_momentum_, params.l_toe2heel, lambda(), z0_, params.A2_S2S_t2h, params.B2_S2S_t2h, params.C2_S2S_t2h);

        Kdeadbeat_h2t = SolveDeadbeatGain(params.A2_S2S_h2t, params.B2_S2S_h2t);
        Kdeadbeat_flat = SolveDeadbeatGain(params.A2_S2S_flat, params.B2_S2S_flat);
        Kdeadbeat_t2h = SolveDeadbeatGain(params.A2_S2S_t2h, params.B2_S2S_t2h);

        double r = 5;

        double eps = 0.00000001;

        Klqr_h2t = -SolveDlqrGain(params.A2_S2S_h2t, params.B2_S2S_h2t, Eigen::MatrixXd::Identity(2, 2), Eigen::MatrixXd::Identity(1, 1) * r, eps);
        Klqr_flat = -SolveDlqrGain(params.A2_S2S_flat, params.B2_S2S_flat, Eigen::MatrixXd::Identity(2, 2), Eigen::MatrixXd::Identity(1, 1) * r, eps);
        Klqr_t2h = -SolveDlqrGain(params.A2_S2S_t2h, params.B2_S2S_t2h, Eigen::MatrixXd::Identity(2, 2), Eigen::MatrixXd::Identity(1, 1) * r, eps);

        p1.K_h2t = Klqr_h2t;

        p1.K_flat = Klqr_flat;
        p1.K_t2h = Klqr_t2h;
        r = 1;

        Klqr_flat = -SolveDlqrGain(params.A2_S2S_flat, params.B2_S2S_flat, Eigen::MatrixXd::Identity(2, 2), Eigen::MatrixXd::Identity(1, 1) * r, eps);

        p2.K = Klqr_flat;
    }

    Eigen::Matrix3d MLIP::Params::getA3convT(bool use_momentum, double T, double lam, double z0)
    {
        Eigen::MatrixXd Aconv = Eigen::MatrixXd::Zero(3, 3);
        if (use_momentum)
        {
            Aconv << sinh(T * lam) / lam, (2 * pow(sinh((T * lam) / 2), 2)) / (pow(lam, 2) * z0), T - sinh(T * lam) / lam,
                2 * z0 * pow(sinh((T * lam) / 2), 2), sinh(T * lam) / lam, -2 * z0 * pow(sinh((T * lam) / 2), 2),
                0, 0, T;
        }
        else
        {
            Aconv << sinh(T * lam) / lam, 2 * pow(sinh((T * lam) / 2), 2) / pow(lam, 2), T - sinh(T * lam) / lam,
                (2 * pow(sinh((T * lam) / 2), 2)), sinh(T * lam) / lam, -(2 * pow(sinh((T * lam) / 2), 2)),
                0, 0, T;
        }
        return Aconv;
    }

    void MLIP::Params::getABC_S2S(bool use_momentum, double l, double lam, double z0, Eigen::MatrixXd &As2s, Eigen::MatrixXd &Bs2s, Eigen::MatrixXd &Cs2s)
    {
        double Teps = .01;

        Eigen::MatrixXd Abar_OA = (TOA * A).exp();
        Eigen::MatrixXd BOA = Eigen::MatrixXd::Zero(3, 1);
        BOA(2) = (TOA > Teps) ? 1 / TOA : 0.;
        Eigen::MatrixXd Aconv_OA = getA3convT(use_momentum, TOA, lam, z0);
        Eigen::MatrixXd Bbar_OA = Aconv_OA * BOA;

        Eigen::MatrixXd Bdelta(3, 1), Cdelta(3, 1);
        Bdelta << -1, 0, -1;
        Bdelta(2) = (TOA > Teps) ? Bdelta(2) : 0.;
        Cdelta << -l, 0, -l;

        Eigen::MatrixXd Abar_FA = (TFA * A).exp();
        Eigen::MatrixXd BFA = Eigen::MatrixXd::Zero(3, 1);
        BFA(2) = (TFA > Teps) ? 1 / TFA : 0.;
        Eigen::MatrixXd Aconv_FA = getA3convT(use_momentum, TFA, lam, z0);
        Eigen::MatrixXd Cbar_FA = Aconv_FA * BFA * l;

        Eigen::MatrixXd Abar_UA = (TUA * A).exp();

        Eigen::MatrixXd A3s2s = Abar_UA * Abar_FA * Abar_OA;
        Eigen::MatrixXd B3s2s = Abar_UA * Abar_FA * (Bbar_OA + Bdelta);
        Eigen::MatrixXd C3s2s = Abar_UA * Abar_FA * Cdelta + Abar_UA * Cbar_FA;

        As2s = A3s2s.block(0, 0, 2, 2);
        Bs2s = B3s2s.block(0, 0, 2, 1);
        Cs2s = C3s2s.block(0, 0, 2, 1);
    }

    void MLIP::UpdateDesiredWalking(double vel, double stepWidth)
    {

        params.velDes = vel;

        switch (orbitPeriod)
        {
        case P1orbit:
            p1.Udes_h2t = params.velDes * params.T - params.l_heel2toe;
            p1.Xdes_h2t = (Eigen::MatrixXd::Identity(2, 2) - params.A2_S2S_h2t).inverse() * (params.B2_S2S_h2t * p1.Udes_h2t + params.C2_S2S_h2t);
            solveXdesFAminus_XdesFAplus(params.l_heel2toe, p1.Xdes_h2t, p1.Udes_h2t, p1.XdesFAminus_h2t, p1.XdesFAplus_h2t);

            p1.Udes_flat = params.velDes * params.T - params.l_flat;
            p1.Xdes_flat = (Eigen::MatrixXd::Identity(2, 2) - params.A2_S2S_flat).inverse() * (params.B2_S2S_flat * p1.Udes_flat + params.C2_S2S_flat);
            solveXdesFAminus_XdesFAplus(params.l_flat, p1.Xdes_flat, p1.Udes_flat, p1.XdesFAminus_flat, p1.XdesFAplus_flat);

            p1.Udes_t2h = params.velDes * params.T - params.l_toe2heel;
            p1.Xdes_t2h = (Eigen::MatrixXd::Identity(2, 2) - params.A2_S2S_t2h).inverse() * (params.B2_S2S_t2h * p1.Udes_t2h + params.C2_S2S_t2h);
            solveXdesFAminus_XdesFAplus(params.l_toe2heel, p1.Xdes_t2h, p1.Udes_t2h, p1.XdesFAminus_t2h, p1.XdesFAplus_t2h);

            break;
        case P2orbit:
            // p2.UleftDes+p2.UrightDes = 2*velDes*T
            // assume SS always UA
            // l = 0
            if (stepWidth == 0)
            {
                std::cerr << "Warning: stepWidth is zero, not permitted!" << std::endl;
            }
            else
            {
                p2.UleftDes = -stepWidth;
                p2.UrightDes = 2 * params.velDes * params.T - p2.UleftDes;
                p2.XleftDes = (Eigen::MatrixXd::Identity(2, 2) - params.A2_S2S_flat * params.A2_S2S_flat).inverse() * (params.A2_S2S_flat * params.B2_S2S_flat * p2.UleftDes + params.B2_S2S_flat * p2.UrightDes + params.A2_S2S_flat * params.C2_S2S_flat + params.C2_S2S_flat);
                p2.XrightDes = (Eigen::MatrixXd::Identity(2, 2) - params.A2_S2S_flat * params.A2_S2S_flat).inverse() * (params.A2_S2S_flat * params.B2_S2S_flat * p2.UrightDes + params.B2_S2S_flat * p2.UleftDes + params.A2_S2S_flat * params.C2_S2S_flat + params.C2_S2S_flat);

                solveXdesUAminus_XdesUAplus(0, p2.XrightDes, p2.UrightDes, p2.XdesFAminus_left, p2.XdesFAplus_left);
                solveXdesUAminus_XdesUAplus(0, p2.XleftDes, p2.UleftDes, p2.XdesFAminus_right, p2.XdesFAplus_right);

                // cout << "p2 = "<< p2.XdesFAplus_left << p2.XdesFAplus_right << endl;
            }

            break;
        default:
            std::cerr << "orbit type is wrong!" << std::endl;
            break;
        }
    }

    void MLIP::solveXdesFAminus_XdesFAplus(double l, Eigen::Vector2d Xdes, double Udes, Eigen::Vector2d &XdesFAminus, Eigen::Vector2d &XdesFAplus)
    {
        double Teps = .01;

        Eigen::MatrixXd Abar_OA = (params.TOA * params.A).exp();
        Eigen::MatrixXd BOA = Eigen::MatrixXd::Zero(3, 1);
        BOA(2) = (params.TOA > Teps) ? 1 / params.TOA : 0.;
        Eigen::MatrixXd Aconv_OA = params.getA3convT(use_momentum_, params.TOA, lambda(), z0_);
        Eigen::MatrixXd Bbar_OA = Aconv_OA * BOA;

        Eigen::MatrixXd Bdelta(3, 1), Cdelta(3, 1);
        Bdelta << -1, 0, -1;
        Bdelta(2) = (params.TOA > Teps) ? Bdelta(2) : 0.;
        Cdelta << -l, 0, -l;

        Eigen::MatrixXd Abar_FA = (params.TFA * params.A).exp();
        Eigen::MatrixXd BFA = Eigen::MatrixXd::Zero(3, 1);
        BFA(2) = (params.TFA > Teps) ? 1 / params.TFA : 0.;
        Eigen::MatrixXd Aconv_FA = params.getA3convT(use_momentum_, params.TFA, lambda(), z0_);
        Eigen::MatrixXd Cbar_FA = Aconv_FA * BFA * l;

        Eigen::MatrixXd Abar_UA = (params.TUA * params.A).exp();

        Eigen::Vector3d Xdes3, XdesFAminus3, XdesFAplus3;

        Xdes3 << Xdes, 0;
        XdesFAminus3 = Abar_UA.inverse() * Xdes3;

        XdesFAplus3 = Abar_OA * Xdes3 + (Bbar_OA + Bdelta) * Udes + Cdelta;

        // XdesFAminus3 = Abar_FA*XdesFAplus3 + Cbar_FA;

        XdesFAminus << XdesFAminus3(0), XdesFAminus3(1);
        XdesFAplus << XdesFAplus3(0), XdesFAplus3(1);

        // cout << "l = " << l<< endl;
        // cout << "udes =" << Udes<< endl;
        // cout << "XdesFAminus= " <<  XdesFAminus.transpose() << endl;
        // cout <<  "XdesFAplus= " << XdesFAplus.transpose() << endl;
    }

    void MLIP::solveXdesUAminus_XdesUAplus(double l, Eigen::Vector2d Xdes, double Udes, Eigen::Vector2d &XdesFAminus, Eigen::Vector2d &XdesFAplus)
    {
        double Teps = .01;

        Eigen::MatrixXd Abar_OA = (params.TOA * params.A).exp();
        Eigen::MatrixXd BOA = Eigen::MatrixXd::Zero(3, 1);
        BOA(2) = (params.TOA > Teps) ? 1 / params.TOA : 0.;
        Eigen::MatrixXd Aconv_OA = params.getA3convT(use_momentum_, params.TOA, lambda(), z0_);
        Eigen::MatrixXd Bbar_OA = Aconv_OA * BOA;

        Eigen::MatrixXd Bdelta(3, 1), Cdelta(3, 1);
        Bdelta << -1, 0, -1;
        Bdelta(2) = (params.TOA > Teps) ? Bdelta(2) : 0.;
        Cdelta << -l, 0, -l;

        Eigen::MatrixXd Abar_FA = (params.TFA * params.A).exp();
        Eigen::MatrixXd BFA = Eigen::MatrixXd::Zero(3, 1);
        BFA(2) = (params.TFA > Teps) ? 1 / params.TFA : 0.;
        Eigen::MatrixXd Aconv_FA = params.getA3convT(use_momentum_, params.TFA, lambda(), z0_);
        Eigen::MatrixXd Cbar_FA = Aconv_FA * BFA * l;

        Eigen::MatrixXd Abar_UA = (params.TUA * params.A).exp();

        Eigen::Vector3d Xdes3, XdesUAminus3, XdesUAplus3;

        Xdes3 << Xdes, 0;
        // XdesFAminus3 = Abar_UA.inverse()*Xdes3;

        XdesUAplus3 = Abar_OA * Xdes3 + (Bbar_OA + Bdelta) * Udes + Cdelta;

        XdesUAminus3 = Abar_UA * XdesUAplus3;

        XdesFAminus << XdesUAminus3(0), XdesUAminus3(1);
        XdesFAplus << XdesUAplus3(0), XdesUAplus3(1);

        // cout << "l = " << l<< endl;
        // cout << "udes =" << Udes<< endl;
        // cout << "XdesFAminus= " <<  XdesFAminus.transpose() << endl;
        // cout <<  "XdesFAplus= " << XdesFAplus.transpose() << endl;
    }

    // double MLIP::getStepSize_P1_varimode(Eigen::Vector2d Xtoe, Eigen::Vector2d Xheel, Eigen::Vector2d Xmid, int &mode)
    // {
    //     //     // cout << "*****************" <<endl;
    //     //     //get initial mode from desired velocity, steplength= u(k=0)
    //     double l, stepLength;
    //     //     if (params.velDes > params.footlength/params.T){
    //     //         //solve using heel to toe
    //     //         l = params.l_heel2toe;
    //     //         mode = 1;
    //     //     }else if (params.velDes < -params.footlength/params.T){
    //     //         // toe to heel
    //     //         l = params.l_toe2heel;
    //     //         mode = -1;
    //     //     }else{
    //     //         // flat
    //     //         l = params.l_flat;
    //     //         mode = 0;
    //     //     }
    //     // // cout << "params.velDes "<< params.velDes << endl;
    //     // // cout << "params.footlength " << params.footlength <<endl;
    //     //     stepLength = getStepSize_P1_fixedmode(Xtoe, Xheel, Xmid, mode);
    //     // // cout << "mode guess: " << mode << endl;
    //     // // cout << "Xtoe = " << Xtoe.transpose() <<endl;
    //     // // cout << "Xheel = " << Xheel.transpose() <<endl;
    //     // // cout << "Xmid = " << Xmid.transpose() <<endl;
    //     // // cout << "stepLength = " << stepLength <<endl;

    //     //     //to prevent mode mismatch, i.e. want to walk forward but solved u(k=0) is walking backward
    //     //     //solve next iteration steplenth u(k=1)  using mode determined by u(k=0)
    //     //     if ( (stepLength + l) > params.footlength   ){
    //     //         //solve using heel to toe
    //     //         l = params.l_heel2toe;
    //     //         mode = 1;
    //     //     }else if (  (stepLength + l) < -params.footlength ){
    //     //         // toe to heel
    //     //         l = params.l_toe2heel;
    //     //         mode = -1;
    //     //     }else{
    //     //         // flat
    //     //         l = params.l_flat;
    //     //         mode = 0;
    //     //     }
    //     //     stepLength = getStepSize_P1_fixedmode(Xtoe, Xheel, Xmid, mode);

    //     // // cout << "mode0: " << mode << endl;
    //     // // cout << "stepLength = " << stepLength <<endl;

    //     //     // check if solved u(k=1) corresponds to the mode used
    //     //     // if mode alternates, meaning around boundary of two mode, then choose flat gait
    //     //     int mode_k1 ;
    //     //     if ( (stepLength + l) > params.footlength   ){
    //     //         //solve using heel to toe
    //     //         mode_k1 = 1;
    //     //     }else if (  (stepLength + l) < -params.footlength ){
    //     //         // toe to heel
    //     //         mode_k1 = -1;
    //     //     }else{
    //     //         // flat
    //     //         mode_k1 = 0;
    //     //     }
    //     // // cout << "mode1= " << mode << endl;
    //     //     if (mode_k1 != mode){
    //     //         //choose flat gait
    //     //         mode = 0;

    //     //         stepLength = getStepSize_P1_fixedmode(Xtoe, Xheel, Xmid, mode);
    //     //     }
    //     // // cout << "mode final = "<< mode << endl;
    //     // // cout << "stepLength = " << stepLength <<endl;
    //     // // cout << "*****************" <<endl;
    //     return stepLength;
    // }

    double MLIP::getStepSize_P1_fixedmode(Eigen::Vector2d Xtoe, Eigen::Vector2d Xheel, Eigen::Vector2d Xmid, int mode)
    {
        double stepLength;
        if (mode == mode_heel2toe)
        {
            // heel to toe
            stepLength = p1.K_h2t.transpose() * (Xtoe - p1.Xdes_h2t) + p1.Udes_h2t;
        }
        else if (mode == mode_toe2heel)
        {
            // toe to heel
            stepLength = p1.K_t2h.transpose() * (Xheel - p1.Xdes_t2h) + p1.Udes_t2h;
        }
        else if (mode == mode_flat)
        {
            // flat
            stepLength = p1.K_flat.transpose() * (Xmid - p1.Xdes_flat) + p1.Udes_flat;

            // // for stepping on toe test
            // stepLength = p1.K_flat.transpose()*(Xtoe - p1.Xdes_flat) + p1.Udes_flat;

            // cout << "stepLength = " << stepLength << endl;
        }
        else
        {
            std::cout << "WALKING MODE UNDEFINED! EXITING" << std::endl;
            exit(-1);
        }
        return stepLength;
    }

    double MLIP::getStepSize_P2(Eigen::Vector2d X, StanceStatus stanceLegIdx)
    {
        double stepLength;
        p2.Xdes = (stanceLegIdx == StanceStatus::LeftStance) ? p2.XleftDes : p2.XrightDes;
        p2.Udes = (stanceLegIdx == StanceStatus::LeftStance) ? p2.UleftDes : p2.UrightDes;
        stepLength = p2.K.transpose() * (X - p2.Xdes) + p2.Udes;
        return stepLength;
    }

    Eigen::Vector3d MLIP::SolveLIPWithZMP(double t, Eigen::Vector3d X0, double dpzmp)
    {
        // given X(0), solve for X(t)
        Eigen::Vector3d sol3;
        Eigen::MatrixXd Aconv = params.getA3convT(use_momentum_, t, lambda(), z0_);
        Eigen::VectorXd B(3);
        B << 0, 0, 1;
        sol3 = (t * A3()).exp() * X0 + Aconv * B * dpzmp;
        return sol3;
    }
} // namespace romoco