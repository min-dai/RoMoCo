

#include <romoco_planner/hlip.hpp>
#include <iostream>
#include <romoco_utils/algebraic_riccati.hpp>

#include <unsupported/Eigen/MatrixFunctions> //matrix exponential
namespace romoco
{

HLIP::HLIP(bool useMomentum) : LIPBase(useMomentum) {
};

void HLIP::Init(double z0, double Ts, double Td, int orbitPeriod, double vel, double stepwidth = 0)
{
    this->orbitPeriod = orbitPeriod;
    UpdateHLIP(z0, Ts, Td);
    UpdateDesiredWalking(vel, stepwidth);
}

void HLIP::UpdateHLIP(double z0, double Ts, double Td)
{
    z0_ = z0;
    Ts_ = Ts;
    Td_ = Td;
    T_ = Ts + Td;

    // S2S dynamics
    Eigen::MatrixXd Busw(2, 1);

    params.ASS = A2_ss();
    params.ADS = A2_ds_constant_vel();

    Busw << -1, 0;
    params.B_S2S = (Ts_ * params.ASS).exp() * Busw;
    params.A_S2S = (Ts_ * params.ASS).exp() * (Td_ * params.ADS).exp();

    Kdeadbeat = SolveDeadbeatGain(params.A_S2S, params.B_S2S);

    Klqr = -SolveDlqrGain(params.A_S2S, params.B_S2S, Eigen::MatrixXd::Identity(2, 2), Eigen::MatrixXd::Identity(1, 1) * 20, 0.00000001);


    params.a_DCM = exp(lambda() * T_);
    if (Td_ != 0)
    {
        params.b_DCM = -exp(lambda() * Ts_) * (exp(lambda() * Td_) - 1) / (lambda() * Td_);
    }else{
        params.b_DCM = -exp(lambda() * Ts_);
    }
    Eigen::VectorXd klqr_DCM_vector = -SolveDlqrGain(Eigen::MatrixXd::Identity(1, 1) * params.a_DCM, Eigen::MatrixXd::Identity(1, 1) * params.b_DCM, Eigen::MatrixXd::Identity(1, 1), Eigen::MatrixXd::Identity(1, 1), 0.00000001);
    klqr_DCM = klqr_DCM_vector(0);



    p1.K = Klqr;
    p2.K = Klqr;

    p1.klqr_DCM = klqr_DCM;
    p2.klqr_DCM = klqr_DCM;
}

void HLIP::UpdateDesiredWalking(double vel, double stepWidth)
{

    params.velDes = vel;

    switch (orbitPeriod)
    {
    case P1orbit:
        // p1.Udes = velDes*T
        p1.Udes = params.velDes * T_;
        p1.Xdes = (Eigen::MatrixXd::Identity(2, 2) - params.A_S2S).inverse() * params.B_S2S * p1.Udes;
        p1.DCM_des = 1. / (1. - params.a_DCM) * (params.b_DCM) * p1.Udes;
        break;
    case P2orbit:
        // p2.UleftDes+p2.UrightDes = 2*velDes*T
        p2.UleftDes = -stepWidth;
        p2.UrightDes = 2 * params.velDes * T_ - p2.UleftDes;
        p2.XleftDes = (Eigen::MatrixXd::Identity(2, 2) - params.A_S2S * params.A_S2S).inverse() * (params.A_S2S * params.B_S2S * p2.UleftDes + params.B_S2S * p2.UrightDes);
        p2.XrightDes = (Eigen::MatrixXd::Identity(2, 2) - params.A_S2S * params.A_S2S).inverse() * (params.A_S2S * params.B_S2S * p2.UrightDes + params.B_S2S * p2.UleftDes);

        p2.DCM_leftDes = 1. / (1. - params.a_DCM * params.a_DCM) * (params.a_DCM * params.b_DCM * p2.UleftDes + params.b_DCM * p2.UrightDes);
        p2.DCM_rightDes = 1. / (1. - params.a_DCM * params.a_DCM) * (params.a_DCM * params.b_DCM * p2.UrightDes + params.b_DCM * p2.UleftDes);

        break;
    default:

        break;
    }
}

double HLIP::SolveDesiredStepSize(Eigen::Vector2d x_preimpact, StanceStatus stanceLeg)
{
    double footplacement;

    if (use_dcm_)
    {
        footplacement = (orbitPeriod == P1orbit) ? SolveDesiredStepSizeDCMP1(x_preimpact) : SolveDesiredStepSizeDCMP2(x_preimpact, stanceLeg);
    }
    else
    {
        footplacement = (orbitPeriod == P1orbit) ? SolveDesiredStepSizeP1(x_preimpact) : SolveDesiredStepSizeP2(x_preimpact, stanceLeg);
    }

    return footplacement;
};

double HLIP::SolveDesiredStepSizeP1(Eigen::Vector2d x_preimpact)
{
    double stepLength = p1.K.transpose() * (x_preimpact - p1.Xdes) + p1.Udes;
    return stepLength;
};

double  HLIP::SolveDesiredStepSizeP2(Eigen::Vector2d x_preimpact, StanceStatus stanceLeg)
{
    Eigen::Vector2d Xdes = (stanceLeg == StanceStatus::LeftStance) ? p2.XleftDes : p2.XrightDes;
    double Udes = (stanceLeg == StanceStatus::LeftStance) ? p2.UleftDes : p2.UrightDes;
    double stepLength = p2.K.transpose() * (x_preimpact - Xdes) + Udes;
    return stepLength;
};

double HLIP::SolveDesiredStepSizeDCMP1(Eigen::Vector2d x_preimpact)
{
    double DCM_now = x_preimpact(0) + x_preimpact(1) / lambda() / z0_;
    double stepLength = p1.klqr_DCM * (DCM_now - p1.DCM_des) + p1.Udes;
    return stepLength;
}
double HLIP::SolveDesiredStepSizeDCMP2(Eigen::Vector2d x_preimpact, StanceStatus stanceLeg)
{
    double DCM_now = x_preimpact(0) + x_preimpact(1) / lambda() / z0_;
    double dcm_Des = (stanceLeg == StanceStatus::LeftStance) ? p2.DCM_leftDes : p2.DCM_rightDes;
    double Udes = (stanceLeg == StanceStatus::LeftStance) ? p2.UleftDes : p2.UrightDes;
    double stepLength = p2.klqr_DCM * (DCM_now - dcm_Des) + Udes;
    return stepLength;
};


} // namespace romoco