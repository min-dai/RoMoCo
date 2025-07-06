

#include <biped_planner/HLIP.hpp>

HLIP::HLIP(bool useMomentum) : useMomentum_(useMomentum) {};

void HLIP::Init(double z0, double Ts, double Td, int orbitPeriod, double vel, double stepwidth = 0)
{

    this->orbitPeriod = orbitPeriod;
    updateHLIP(z0, Ts, Td);
    updateDesiredWalking(vel, stepwidth);
}

void HLIP::updateHLIP(double z0, double Ts, double Td)
{
    params.z0 = z0;
    params.Ts = Ts;
    params.Td = Td;

    params.T = Ts + Td;
    params.lambda = sqrt(params.grav / params.z0);

    // S2S dynamics
    MatrixXd Busw(2, 1);

    if (useMomentum_)
    {
        params.ASS << 0, 1 / params.z0,
            params.grav, 0;
        params.ADS << 0, 1 / params.z0,
            0, 0;
    }
    else
    {
        params.ASS << 0, 1,
            params.grav / params.z0, 0;
        params.ADS << 0, 1,
            0, 0;
    }
    Busw << -1, 0;
    params.B_S2S = (params.Ts * params.ASS).exp() * Busw;
    params.A_S2S = (params.Ts * params.ASS).exp() * (params.Td * params.ADS).exp();

    Kdeadbeat = solve_deadbeat_gain(params.A_S2S, params.B_S2S);

    Klqr = -solve_dlqr_gain(params.A_S2S, params.B_S2S, MatrixXd::Identity(2, 2), MatrixXd::Identity(1, 1) * 20, 0.00000001);

    //TODO:division by zero when Td=0
    params.a_DCM = exp(params.lambda * params.T);
    params.b_DCM = -exp(params.lambda * params.Ts) * (exp(params.lambda * params.Td) - 1) / (params.lambda * params.Td);
    VectorXd klqr_DCM_vector = -solve_dlqr_gain(MatrixXd::Identity(1, 1) * params.a_DCM, MatrixXd::Identity(1, 1) * params.b_DCM, MatrixXd::Identity(1, 1), MatrixXd::Identity(1, 1), 0.00000001);
    klqr_DCM = klqr_DCM_vector(0);

    // p1.K = Kdeadbeat;
    // p2.K = Kdeadbeat;

    p1.K = Klqr;
    p2.K = Klqr;

    p1.klqr_DCM = klqr_DCM;
    p2.klqr_DCM = klqr_DCM;
}

void HLIP::updateDesiredWalking(double vel, double stepWidth)
{

    params.velDes = vel;

    double DistSum = vel * params.T; // total traveled distance in SSPand DSP.

    switch (orbitPeriod)
    {
    case P1orbit:
        // cout << "params.velDes = " <<params.velDes <<endl;
        p1.Udes = params.velDes * params.T;
        p1.Xdes = (MatrixXd::Identity(2, 2) - params.A_S2S).inverse() * params.B_S2S * p1.Udes;
        p1.DCM_des = 1. / (1. - params.a_DCM) * (params.b_DCM) * p1.Udes;
        // cout << "p1.DCM_des  " << p1.DCM_des <<endl;
        break;
    case P2orbit:
        // p2.UleftDes+p2.UrightDes = 2*velDes*T
        if (stepWidth == 0)
        {
            // P1 orbit is trivially P2 orbit
            p2.UleftDes = vel * params.T;
            p2.UrightDes = vel * params.T;
            p2.XleftDes = (MatrixXd::Identity(2, 2) - params.A_S2S).inverse() * params.B_S2S * p2.UleftDes;
            p2.XrightDes = (MatrixXd::Identity(2, 2) - params.A_S2S).inverse() * params.B_S2S * p2.UrightDes;
        }
        else
        {
            p2.UleftDes = -stepWidth;
            p2.UrightDes = 2 * params.velDes * params.T - p2.UleftDes;
            p2.XleftDes = (MatrixXd::Identity(2, 2) - params.A_S2S * params.A_S2S).inverse() * (params.A_S2S * params.B_S2S * p2.UleftDes + params.B_S2S * p2.UrightDes);
            p2.XrightDes = (MatrixXd::Identity(2, 2) - params.A_S2S * params.A_S2S).inverse() * (params.A_S2S * params.B_S2S * p2.UrightDes + params.B_S2S * p2.UleftDes);

            p2.DCM_leftDes = 1. / (1. - params.a_DCM * params.a_DCM) * (params.a_DCM * params.b_DCM * p2.UleftDes + params.b_DCM * p2.UrightDes);
            p2.DCM_rightDes = 1. / (1. - params.a_DCM * params.a_DCM) * (params.a_DCM * params.b_DCM * p2.UrightDes + params.b_DCM * p2.UleftDes);
            // cout << "p2.DCM_leftDes  " << p2.DCM_leftDes << ", p2.DCM_rightDes  "  << p2.DCM_rightDes  <<endl;
        }

        break;
    default:

        break;
    }
}

Vector2d HLIP::getDesiredStepSizeDeadbeat(double p, double v, StanceStatus stanceLeg)
{
    Vector2d out;

    if (useDCM_)
    {
        out = (orbitPeriod == P1orbit) ? p1.getDeadbeatStepSize_DCM(p, v, params.lambda, params.z0) : p2.getDeadbeatStepSize_DCM(p, v, params.lambda, params.z0, stanceLeg);
    }
    else
    {
        out = (orbitPeriod == P1orbit) ? p1.getDeadbeatStepSize(p, v, params.lambda) : p2.getDeadbeatStepSize(p, v, params.lambda, stanceLeg);
    }

    return out;
};

Vector2d HLIP::P1::getDeadbeatStepSize(double p, double v, double lambda)
{
    Vector2d Xnow, dXnow;
    Xnow << p, v;

    dXnow << v, pow(lambda, 2) * p;

    double stepLength = K.transpose() * (Xnow - this->Xdes) + this->Udes;

    double dstepLength = K.transpose() * dXnow;

    // cout << "p1.Xdes_flat = " << Xdes.transpose() << endl;
    //     cout << "p1.K_flat = " << K.transpose() << endl;
    //     cout << "p1.Udes_flat = " << Udes << endl;
    //     cout << "Xmid = " << Xnow.transpose() << endl;

    StepX << stepLength, dstepLength;

    return StepX;
}

Vector2d HLIP::P2::getDeadbeatStepSize(double p, double v, double lambda, StanceStatus stanceLeg)
{

    Vector2d Xnow, dXnow;
    Xnow << p, v;

    dXnow << v, pow(lambda, 2) * p;

    double stepLength, dstepLength;

    Xdes = (stanceLeg == StanceStatus::LeftStance) ? XleftDes : XrightDes;
    Udes = (stanceLeg == StanceStatus::LeftStance) ? UleftDes : UrightDes;

    // cout << "p2.Xdes = " << XleftDes << ", " << XrightDes << endl;

    // cout << "p2.Udes = " << Udes << endl;

    stepLength = K.transpose() * (Xnow - Xdes) + Udes;
    dstepLength = K.transpose() * dXnow;
    // cout << "p2.stepLength = " << stepLength << endl;
    StepX << stepLength, dstepLength;
    return StepX;
}

Vector2d HLIP::P1::getDeadbeatStepSize_DCM(double p, double v, double lambda, double z0)
{

    double DCM_now = p + v / lambda / z0;
    double dDCM_now = v + pow(lambda, 2) * p / lambda / z0;

    double stepLength = klqr_DCM * (DCM_now - DCM_des) + this->Udes;

    double dstepLength = klqr_DCM * dDCM_now;

    // cout << "p1.Xdes_flat = " << Xdes.transpose() << endl;
    //     cout << "p1.K_flat = " << K.transpose() << endl;
    //     cout << "p1.Udes_flat = " << Udes << endl;
    //     cout << "Xmid = " << Xnow.transpose() << endl;

    StepX << stepLength, dstepLength;

    return StepX;
}

Vector2d HLIP::P2::getDeadbeatStepSize_DCM(double p, double v, double lambda, double z0, StanceStatus stanceLeg)
{

    double DCM_now = p + v / lambda / z0;
    double dDCM_now = v + pow(lambda, 2) * p / lambda / z0;

    double dcm_Des = (stanceLeg == StanceStatus::LeftStance) ? DCM_leftDes : DCM_rightDes;
    double Udes = (stanceLeg == StanceStatus::LeftStance) ? UleftDes : UrightDes;

    double stepLength = klqr_DCM * (DCM_now - dcm_Des) + Udes;

    double dstepLength = klqr_DCM * dDCM_now;

    StepX << stepLength, dstepLength;
    return StepX;
}

Vector2d HLIP::get_LIPsol(double t, Vector2d X0)
{
    // given X(0), solve for X(t)
    Vector2d sol;
    sol = (t * params.ASS).exp() * X0;
    return sol;
}

double HLIP::getOrbitalEnergy(double p, double Ly)
{
    // get the orbital energy in SSP
    // p is the postion w.r.t. contact pivot, Ly is the momentum about the contact pivot
    return pow(Ly / params.z0, 2) - params.grav /
                                        params.z0 * pow(p, 2);
}

Vector2d HLIP::solve_deadbeat_gain(Matrix2d A, Vector2d B)
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
