

#include "biped_planner/MLIP.hpp"

MLIP::MLIP(bool useMomentum_) {
    params.useMomentum = useMomentum_;
};

void MLIP::Init(double z0, double TOA, double TFA, double TUA, int orbitPeriod, double vel, double footlength, double stepwidth = 0)
{

    this->orbitPeriod = orbitPeriod;

    params.footlength = footlength;
    updateMLIP(z0, TOA, TFA, TUA);
    updateDesiredWalking(vel, stepwidth);
}

void MLIP::updateMLIP(double z0, double TOA, double TFA, double TUA)
{
    params.z0 = z0;
    params.TOA = TOA;
    params.TFA = TFA;
    params.TUA = TUA;

    params.l_heel2toe = params.footlength;
    params.l_toe2heel = -params.footlength;
    params.l_flat = 0;

    params.T = TOA + TFA + TUA;
    params.lambda = sqrt(params.grav / params.z0);

    // S2S dynamics for different foot contact scenario
    if (params.useMomentum)
    {
        params.A << 0, 1 / params.z0, 0,
            params.grav, 0, -params.grav,
            0, 0, 0;
    }
    else
    {
        params.A << 0, 1, 0,
            params.grav / params.z0, 0, -params.grav / params.z0,
            0, 0, 0;
    }
    params.getABC_S2S(params.l_heel2toe, params.A2_S2S_h2t, params.B2_S2S_h2t, params.C2_S2S_h2t);
    params.getABC_S2S(params.l_flat, params.A2_S2S_flat, params.B2_S2S_flat, params.C2_S2S_flat);
    params.getABC_S2S(params.l_toe2heel, params.A2_S2S_t2h, params.B2_S2S_t2h, params.C2_S2S_t2h);

    Kdeadbeat_h2t = solve_deadbeat_gain(params.A2_S2S_h2t, params.B2_S2S_h2t);
    Kdeadbeat_flat = solve_deadbeat_gain(params.A2_S2S_flat, params.B2_S2S_flat);
    Kdeadbeat_t2h = solve_deadbeat_gain(params.A2_S2S_t2h, params.B2_S2S_t2h);

    double r = 5;

    double eps = 0.00000001;

    Klqr_h2t = -solve_dlqr_gain(params.A2_S2S_h2t, params.B2_S2S_h2t, MatrixXd::Identity(2, 2), MatrixXd::Identity(1, 1) * r, eps);
    Klqr_flat = -solve_dlqr_gain(params.A2_S2S_flat, params.B2_S2S_flat, MatrixXd::Identity(2, 2), MatrixXd::Identity(1, 1) * r, eps);
    Klqr_t2h = -solve_dlqr_gain(params.A2_S2S_t2h, params.B2_S2S_t2h, MatrixXd::Identity(2, 2), MatrixXd::Identity(1, 1) * r, eps);

    // p1.K = Kdeadbeat;
    // p2.K = Kdeadbeat;

    p1.K_h2t = Klqr_h2t;

    p1.K_flat = Klqr_flat;
    p1.K_t2h = Klqr_t2h;
    r = 1;

    Klqr_flat = -solve_dlqr_gain(params.A2_S2S_flat, params.B2_S2S_flat, MatrixXd::Identity(2, 2), MatrixXd::Identity(1, 1) * r, eps);

    p2.K = Klqr_flat;
}

Matrix3d MLIP::Params::getAconvT(double T)
{
    MatrixXd Aconv = MatrixXd::Zero(3, 3);
    if (useMomentum)
    {
        Aconv << sinh(T * lambda) / lambda, (2 * pow(sinh((T * lambda) / 2), 2)) / (pow(lambda, 2) * z0), T - sinh(T * lambda) / lambda,
            2 * z0 * pow(sinh((T * lambda) / 2), 2), sinh(T * lambda) / lambda, -2 * z0 * pow(sinh((T * lambda) / 2), 2),
            0, 0, T;
    }
    else
    {
        Aconv << sinh(T * lambda) / lambda, 2 * pow(sinh((T * lambda) / 2), 2) / pow(lambda, 2), T - sinh(T * lambda) / lambda,
            (2 * pow(sinh((T * lambda) / 2), 2)), sinh(T * lambda) / lambda, -(2 * pow(sinh((T * lambda) / 2), 2)),
            0, 0, T;
    }
    return Aconv;
}

void MLIP::Params::getABC_S2S(double l, MatrixXd &As2s, MatrixXd &Bs2s, MatrixXd &Cs2s)
{
    double Teps = .01;

    MatrixXd Abar_OA = (TOA * A).exp();
    MatrixXd BOA = MatrixXd::Zero(3, 1);
    BOA(2) = (TOA > Teps) ? 1 / TOA : 0.;
    MatrixXd Aconv_OA = getAconvT(TOA);
    MatrixXd Bbar_OA = Aconv_OA * BOA;

    MatrixXd Bdelta(3, 1), Cdelta(3, 1);
    Bdelta << -1, 0, -1;
    Bdelta(2) = (TOA > Teps) ? Bdelta(2) : 0.;
    Cdelta << -l, 0, -l;

    MatrixXd Abar_FA = (TFA * A).exp();
    MatrixXd BFA = MatrixXd::Zero(3, 1);
    BFA(2) = (TFA > Teps) ? 1 / TFA : 0.;
    MatrixXd Aconv_FA = getAconvT(TFA);
    MatrixXd Cbar_FA = Aconv_FA * BFA * l;

    MatrixXd Abar_UA = (TUA * A).exp();

    MatrixXd A3s2s = Abar_UA * Abar_FA * Abar_OA;
    MatrixXd B3s2s = Abar_UA * Abar_FA * (Bbar_OA + Bdelta);
    MatrixXd C3s2s = Abar_UA * Abar_FA * Cdelta + Abar_UA * Cbar_FA;

    As2s = A3s2s.block(0, 0, 2, 2);
    Bs2s = B3s2s.block(0, 0, 2, 1);
    Cs2s = C3s2s.block(0, 0, 2, 1);
}

void MLIP::updateDesiredWalking(double vel, double stepWidth)
{

    params.velDes = vel;

    switch (orbitPeriod)
    {
    case P1orbit:
        p1.Udes_h2t = params.velDes * params.T - params.l_heel2toe;
        p1.Xdes_h2t = (MatrixXd::Identity(2, 2) - params.A2_S2S_h2t).inverse() * (params.B2_S2S_h2t * p1.Udes_h2t + params.C2_S2S_h2t);
        solveXdesFAminus_XdesFAplus(params.l_heel2toe, p1.Xdes_h2t, p1.Udes_h2t, p1.XdesFAminus_h2t, p1.XdesFAplus_h2t);

        p1.Udes_flat = params.velDes * params.T - params.l_flat;
        p1.Xdes_flat = (MatrixXd::Identity(2, 2) - params.A2_S2S_flat).inverse() * (params.B2_S2S_flat * p1.Udes_flat + params.C2_S2S_flat);
        solveXdesFAminus_XdesFAplus(params.l_flat, p1.Xdes_flat, p1.Udes_flat, p1.XdesFAminus_flat, p1.XdesFAplus_flat);

        p1.Udes_t2h = params.velDes * params.T - params.l_toe2heel;
        p1.Xdes_t2h = (MatrixXd::Identity(2, 2) - params.A2_S2S_t2h).inverse() * (params.B2_S2S_t2h * p1.Udes_t2h + params.C2_S2S_t2h);
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
                p2.XleftDes = (MatrixXd::Identity(2, 2) - params.A2_S2S_flat * params.A2_S2S_flat).inverse() * (params.A2_S2S_flat * params.B2_S2S_flat * p2.UleftDes + params.B2_S2S_flat * p2.UrightDes + params.A2_S2S_flat * params.C2_S2S_flat + params.C2_S2S_flat);
                p2.XrightDes = (MatrixXd::Identity(2, 2) - params.A2_S2S_flat * params.A2_S2S_flat).inverse() * (params.A2_S2S_flat * params.B2_S2S_flat * p2.UrightDes + params.B2_S2S_flat * p2.UleftDes + params.A2_S2S_flat * params.C2_S2S_flat + params.C2_S2S_flat);

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

void MLIP::solveXdesFAminus_XdesFAplus(double l, Vector2d Xdes, double Udes, Vector2d &XdesFAminus, Vector2d &XdesFAplus)
{
    double Teps = .01;

    double TOA = params.TOA;
    double TFA = params.TFA;
    double TUA = params.TUA;

    MatrixXd Abar_OA = (TOA * params.A).exp();
    MatrixXd BOA = MatrixXd::Zero(3, 1);
    BOA(2) = (TOA > Teps) ? 1 / TOA : 0.;
    MatrixXd Aconv_OA = params.getAconvT(TOA);
    MatrixXd Bbar_OA = Aconv_OA * BOA;

    MatrixXd Bdelta(3, 1), Cdelta(3, 1);
    Bdelta << -1, 0, -1;
    Bdelta(2) = (TOA > Teps) ? Bdelta(2) : 0.;
    Cdelta << -l, 0, -l;

    MatrixXd Abar_FA = (TFA * params.A).exp();
    MatrixXd BFA = MatrixXd::Zero(3, 1);
    BFA(2) = (TFA > Teps) ? 1 / TFA : 0.;
    MatrixXd Aconv_FA = params.getAconvT(TFA);
    MatrixXd Cbar_FA = Aconv_FA * BFA * l;

    MatrixXd Abar_UA = (TUA * params.A).exp();

    Vector3d Xdes3, XdesFAminus3, XdesFAplus3;

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

void MLIP::solveXdesUAminus_XdesUAplus(double l, Vector2d Xdes, double Udes, Vector2d &XdesFAminus, Vector2d &XdesFAplus)
{
    double Teps = .01;

    double TOA = params.TOA;
    double TFA = params.TFA;
    double TUA = params.TUA;

    MatrixXd Abar_OA = (TOA * params.A).exp();
    MatrixXd BOA = MatrixXd::Zero(3, 1);
    BOA(2) = (TOA > Teps) ? 1 / TOA : 0.;
    MatrixXd Aconv_OA = params.getAconvT(TOA);
    MatrixXd Bbar_OA = Aconv_OA * BOA;

    MatrixXd Bdelta(3, 1), Cdelta(3, 1);
    Bdelta << -1, 0, -1;
    Bdelta(2) = (TOA > Teps) ? Bdelta(2) : 0.;
    Cdelta << -l, 0, -l;

    MatrixXd Abar_FA = (TFA * params.A).exp();
    MatrixXd BFA = MatrixXd::Zero(3, 1);
    BFA(2) = (TFA > Teps) ? 1 / TFA : 0.;
    MatrixXd Aconv_FA = params.getAconvT(TFA);
    MatrixXd Cbar_FA = Aconv_FA * BFA * l;

    MatrixXd Abar_UA = (TUA * params.A).exp();

    Vector3d Xdes3, XdesUAminus3, XdesUAplus3;

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

double MLIP::getStepSize_P1_varimode(Vector2d Xtoe, Vector2d Xheel, Vector2d Xmid, int &mode)
{
    //     // cout << "*****************" <<endl;
    //     //get initial mode from desired velocity, steplength= u(k=0)
    double l, stepLength;
    //     if (params.velDes > params.footlength/params.T){
    //         //solve using heel to toe
    //         l = params.l_heel2toe;
    //         mode = 1;
    //     }else if (params.velDes < -params.footlength/params.T){
    //         // toe to heel
    //         l = params.l_toe2heel;
    //         mode = -1;
    //     }else{
    //         // flat
    //         l = params.l_flat;
    //         mode = 0;
    //     }
    // // cout << "params.velDes "<< params.velDes << endl;
    // // cout << "params.footlength " << params.footlength <<endl;
    //     stepLength = getStepSize_P1_fixedmode(Xtoe, Xheel, Xmid, mode);
    // // cout << "mode guess: " << mode << endl;
    // // cout << "Xtoe = " << Xtoe.transpose() <<endl;
    // // cout << "Xheel = " << Xheel.transpose() <<endl;
    // // cout << "Xmid = " << Xmid.transpose() <<endl;
    // // cout << "stepLength = " << stepLength <<endl;

    //     //to prevent mode mismatch, i.e. want to walk forward but solved u(k=0) is walking backward
    //     //solve next iteration steplenth u(k=1)  using mode determined by u(k=0)
    //     if ( (stepLength + l) > params.footlength   ){
    //         //solve using heel to toe
    //         l = params.l_heel2toe;
    //         mode = 1;
    //     }else if (  (stepLength + l) < -params.footlength ){
    //         // toe to heel
    //         l = params.l_toe2heel;
    //         mode = -1;
    //     }else{
    //         // flat
    //         l = params.l_flat;
    //         mode = 0;
    //     }
    //     stepLength = getStepSize_P1_fixedmode(Xtoe, Xheel, Xmid, mode);

    // // cout << "mode0: " << mode << endl;
    // // cout << "stepLength = " << stepLength <<endl;

    //     // check if solved u(k=1) corresponds to the mode used
    //     // if mode alternates, meaning around boundary of two mode, then choose flat gait
    //     int mode_k1 ;
    //     if ( (stepLength + l) > params.footlength   ){
    //         //solve using heel to toe
    //         mode_k1 = 1;
    //     }else if (  (stepLength + l) < -params.footlength ){
    //         // toe to heel
    //         mode_k1 = -1;
    //     }else{
    //         // flat
    //         mode_k1 = 0;
    //     }
    // // cout << "mode1= " << mode << endl;
    //     if (mode_k1 != mode){
    //         //choose flat gait
    //         mode = 0;

    //         stepLength = getStepSize_P1_fixedmode(Xtoe, Xheel, Xmid, mode);
    //     }
    // // cout << "mode final = "<< mode << endl;
    // // cout << "stepLength = " << stepLength <<endl;
    // // cout << "*****************" <<endl;
    return stepLength;
}

double MLIP::getStepSize_P1_fixedmode(Vector2d Xtoe, Vector2d Xheel, Vector2d Xmid, int mode, double deltau_prev)
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
        //  cout << "p1.Xdes_flat = " << p1.Xdes_flat.transpose() << endl;
        //  cout << "p1.K_flat = " << p1.K_flat.transpose() << endl;
        //  cout << "p1.Udes_flat = " << p1.Udes_flat << endl;
        //  cout << "Xmid = " << Xmid.transpose() << endl;
        stepLength = p1.K_flat.transpose() * (Xmid - p1.Xdes_flat) + p1.Udes_flat; // + p1.Ku*deltau_prev;

        // // for stepping on toe test
        // stepLength = p1.K_flat.transpose()*(Xtoe - p1.Xdes_flat) + p1.Udes_flat;// + p1.Ku*deltau_prev;

        // cout << "stepLength = " << stepLength << endl;
    }
    else
    {
        cout << "WALKING MODE UNDEFINED! EXITING" << endl;
        exit(-1);
    }
    return stepLength;
}

double MLIP::getStepSize_P2(Vector2d X, StanceStatus stanceLegIdx, double deltau_prev)
{

    // Vector2d Xnow, dXnow;
    // Xnow << p, v;

    //     dXnow << v, pow(params.lambda, 2) * p;

    double stepLength;

    p2.Xdes = (stanceLegIdx == StanceStatus::LeftStance) ? p2.XleftDes : p2.XrightDes;
    p2.Udes = (stanceLegIdx == StanceStatus::LeftStance) ? p2.UleftDes : p2.UrightDes;

    // cout << "p2.Xdes = " << p2.XleftDes << ", " << p2.XrightDes << endl;

    // cout << "p2.Udes = " << p2.Udes << endl;

    stepLength = p2.K.transpose() * (X - p2.Xdes) + p2.Udes + p2.Ku * deltau_prev;
    // dstepLength = K.transpose() * dXnow;

    return stepLength;
}

///////////////////////////////// other LIP related helper functions ////////////////////////
double MLIP::solve_Ts()
{
    double Ts = 0;

    ///////// TODO: what is this?
    // double xcdes = this->param.xratio * this->param.ldes;
    // double lam = sqrt(this->config.g / this->cache.z0LIP);
    // double a = this->cache.xcLIP;
    // double b = this->cache.Ly / lam / this->cache.z0LIP;
    // double c = xcdes;
    // Ts = 1 / lam * log((c + sqrt(-pow(a, 2) + pow(b, 2) + pow(c, 2))) / (a + b));

    return Ts;
}

Vector3d MLIP::get_MLIPsol3(double t, Vector3d X0, double dpzmp)
{
    // given X(0), solve for X(t)
    Vector3d sol3;
    MatrixXd Aconv = params.getAconvT(t);
    VectorXd B(3);
    B << 0, 0, 1;
    sol3 = (t * params.A).exp() * X0 + Aconv * B * dpzmp;

    // cout << "X0 = " << X0 << ", t =" << t << ", dpzmp =" << dpzmp << endl;
    // cout << "Aconv "<< Aconv << endl;
    // cout << "(t*params.A).exp() " << (t*params.A).exp() << endl;
    return sol3;
}

Vector2d MLIP::get_MLIPsol2(double t, Vector3d X0, double dpzmp)
{
    // given X(0), solve for X(t)
    Vector3d sol3;
    MatrixXd Aconv = params.getAconvT(t);
    VectorXd B(3);
    B << 0, 0, 1;
    sol3 = (t * params.A).exp() * X0 + Aconv * B * dpzmp;
    Vector2d sol2 = sol3.segment(0, 2);

    return sol2;
}

double MLIP::getOrbitalEnergy(double p, double Ly)
{
    // get the orbital energy in SSP
    // p is the postion w.r.t. contact pivot, Ly is the momentum about the contact pivot
    return pow(Ly / params.z0, 2) - params.grav / params.z0 * pow(p, 2);
}

Vector2d MLIP::solve_deadbeat_gain(Matrix2d A, Vector2d B)
{
    // explict solution for 2-by-2 matrix only
    Matrix2d Atmp;
    Atmp << -B(0), -B(1),
        A(1, 1) * B(0) - A(0, 1) * B(1), A(0, 0) * B(1) - A(1, 0) * B(0);
    MatrixXd Btmp(2, 1);
    Btmp << A(0, 0) + A(1, 1),
        A(0, 1) * A(1, 0) - A(0, 0) * A(1, 1);

    MatrixXd Ktmp = Atmp.inverse() * Btmp;
    Vector2d Kdeadbeat(Ktmp(0, 0), Ktmp(1, 0));
    return Kdeadbeat;
};
