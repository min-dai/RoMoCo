//outdated code

#include <HLIP/AdaptiveHLIP.hpp>

AdaptiveHLIP::AdaptiveHLIP() {
    ros::param::get("/cassie/locomotion/stepping/adaptive/outputTrackingSag", p1a.outputTracking);
    ros::param::get("/cassie/locomotion/stepping/adaptive/outputTrackingLat", p2a.outputTracking);
    ros::param::get("/cassie/locomotion/stepping/adaptive/algorithm",algorithm);
    ros::param::get("/cassie/locomotion/stepping/adaptive/outputTracking_ActualStepSize",outputTracking_ActualStepSize);

};
void AdaptiveHLIP::Init(double z0, double Ts, double Td, int orbitPeriod, double vel, double stepwidth = 0) {

    this->orbitPeriod = orbitPeriod;
    updateHLIP(z0, Ts, Td);
    initEstimator();
    updateDesiredWalking(vel, stepwidth);
}

void AdaptiveHLIP::initEstimator(){
    if (orbitPeriod == P1orbit){
        p1a.initialize_estimator(params, p1);
        
    }else if(orbitPeriod == P2orbit){
        p2a.initialize_estimator(params, p2);
    }
}

void AdaptiveHLIP::getABCfromTheta(MatrixXd Theta, MatrixXd &A, MatrixXd &B, MatrixXd &C)
{
    A = Theta.block(0, 0, 2, 2).transpose();
    B = Theta.block(2, 0, 1, 2).transpose();
    C = Theta.block(3, 0, 1, 2).transpose();
};
void AdaptiveHLIP::getABCDEFfromTheta(MatrixXd Theta, MatrixXd &A, MatrixXd &B, MatrixXd &C, MatrixXd &D, MatrixXd &E, MatrixXd &F)
{
    A = Theta.block(0, 0, 2, 2).transpose();
    B = Theta.block(2, 0, 1, 2).transpose();
    C = Theta.block(3, 0, 1, 2).transpose();
    D = Theta.block(0, 2, 2, 1).transpose();
    E = Theta.block(2, 2, 1, 1).transpose();
    F = Theta.block(3, 2, 1, 1).transpose();
};

Vector2d AdaptiveHLIP::solveDeadbeatControllerGain(MatrixXd Ahat, MatrixXd Bhat)
{
    Vector2d Khat;

    // // solve for deadbeat gain
    // Matrix2d Mat;
    // Vector2d Vec;
    // Mat << -Bhat(0, 0), -Bhat(1, 0),
    //     Ahat(1, 1) * Bhat(0, 0) - Ahat(0, 1) * Bhat(1, 0), Ahat(0, 0) * Bhat(1, 0) - Bhat(0, 0) * Ahat(1, 0);
    // Vec << Ahat(0, 0) + Ahat(1, 1),
    //     Ahat(0, 1) * Ahat(1, 0) - Ahat(0, 0) * Ahat(1, 1);
    // Khat = (Mat.completeOrthogonalDecomposition().solve(Vec));

    //solve for LQR gain
    Khat = -solve_dlqr_gain(Ahat,Bhat,MatrixXd::Identity(2,2),MatrixXd::Identity(1,1)*10,0.00000001);

    return Khat;
};

void AdaptiveHLIP::solveOutputTrackingGain(MatrixXd Ahat, MatrixXd Bhat, MatrixXd Chat, MatrixXd Dhat, MatrixXd Ehat, MatrixXd Fhat, VectorXd Khat,VectorXd Xdes_hat, double ref, double& kf, double& bf) {
    

    if (!outputTracking_ActualStepSize){
        p1a.Dhat.setZero();
        p1a.Ehat.setConstant(1.);
        p1a.Fhat.setZero();
    }
    
    MatrixXd r(1, 1);
    MatrixXd M(1, 2);
    r << ref;
    M = (Dhat + Ehat * Khat.transpose()) * (MatrixXd::Identity(2, 2) - Ahat - Bhat * Khat.transpose()).inverse();
    MatrixXd tmp;
    if (r(0, 0) == 0.) {
        tmp = -(M * Bhat + Ehat).inverse() * (M * Chat + Fhat);
        bf = tmp(0, 0);
        kf = 0.;
    }
    else {
        tmp = (M * Bhat * r + Ehat * r).inverse() * (r - Fhat - M * Chat);
        bf = 0;
        kf = tmp(0, 0);
    }

    // MatrixXd r(1, 1);
    // MatrixXd M(1, 1);
    // r << ref;
    // M = (Dhat + Ehat * Khat.transpose()) * (MatrixXd::Identity(2, 2) - Ahat - Bhat * Khat.transpose()).inverse()*(-Bhat*Khat*Xdes_hat + Bhat*obj.Udes + Chat) - Ehat*Khat*Xdes_hat + Ehat*Udes + Fhat -r;
    // MatrixXd tmp;

    // tmp = ((Dhat + Ehat * Khat.transpose())* (MatrixXd::Identity(2, 2) - Ahat - Bhat * Khat.transpose()).inverse()*Bhat + Ehat).inverse()*M;
    // bf = tmp(0,0);




}

Vector2d AdaptiveHLIP::getDesiredStepSizeDeadbeat(double p, double v, int stanceLeg) {
    Vector2d out = (orbitPeriod == P1orbit) ?
        p1a.getDeadbeatStepSize(p, v, params.lambda) :
        p2a.getDeadbeatStepSize(p, v, params.lambda, stanceLeg);
    return out;
};

void AdaptiveHLIP::updateEstimator(Vector2d xkm, double ukm, Vector2d xk, double ykm, int stanceLegIdx)
{


        if (orbitPeriod == P1orbit)
        {
            p1a.updateEstimationParams(xkm, ukm, xk, ykm, algorithm, k);
            if (p1a.outputTracking){
                getABCDEFfromTheta(p1a.Theta, p1a.Ahat, p1a.Bhat, p1a.Chat, p1a.Dhat, p1a.Ehat, p1a.Fhat);
            }else{
                getABCfromTheta(p1a.Theta, p1a.Ahat, p1a.Bhat, p1a.Chat);
            }
            p1a.Xdes_hat = (MatrixXd::Identity(2, 2) - p1a.Ahat).completeOrthogonalDecomposition().solve(p1a.Chat + p1a.Bhat * p1a.Udes);
            
            p1a.Khat = solveDeadbeatControllerGain(p1a.Ahat, p1a.Bhat);
            if (p1a.outputTracking){
                solveOutputTrackingGain(p1a.Ahat, p1a.Bhat, p1a.Chat, p1a.Dhat, p1a.Ehat, p1a.Fhat, p1a.Khat, p1a.Xdes, p1a.Udes, p1a.kf, p1a.bf);
            }
            std::cout << "P1 estimation updated" << std::endl;
            std::cout << "kf = " << p1a.kf << std::endl;
            std::cout << "bf = " << p1a.bf << std::endl;
            std::cout << "Dhat = " << p1a.Dhat << std::endl;
            std::cout << "Ehat = " << p1a.Ehat << std::endl;
            std::cout << "Fhat = " << p1a.Fhat << std::endl;

            // //direct method, no update on k
            // p1a.kf = p1a.kf + (ukm - p1a.Udes)*.05;
            // std::cout << "P1 estimation updated" << std::endl;
            // std::cout << "kf = " << p1a.kf << std::endl;

        }
        else if (orbitPeriod == P2orbit)
        {
            p2a.updateEstimationParams(xkm, ukm, xk, ykm, stanceLegIdx, algorithm, k);
            if (p1a.outputTracking){
                getABCDEFfromTheta(p2a.ThetaL, p2a.AhatL, p2a.BhatL, p2a.ChatL, p2a.DhatL, p2a.EhatL, p2a.FhatL);
            getABCDEFfromTheta(p2a.ThetaR, p2a.AhatR, p2a.BhatR, p2a.ChatR, p2a.DhatR, p2a.EhatR, p2a.FhatR);
            }else{
            getABCfromTheta(p2a.ThetaL, p2a.AhatL, p2a.BhatL, p2a.ChatL);
            getABCfromTheta(p2a.ThetaR, p2a.AhatR, p2a.BhatR, p2a.ChatR);
            
            }
            p2a.Xdes_hatL = (MatrixXd::Identity(2, 2) - p2a.AhatR * p2a.AhatL).inverse() * (p2a.AhatR * p2a.BhatL * p2a.UleftDes + p2a.BhatR * p2a.UrightDes + p2a.AhatR * p2a.ChatL + p2a.ChatR);
            p2a.Xdes_hatR = (MatrixXd::Identity(2, 2) - p2a.AhatL * p2a.AhatR).inverse() * (p2a.AhatL * p2a.BhatR * p2a.UrightDes + p2a.BhatL * p2a.UleftDes + p2a.AhatL * p2a.ChatR + p2a.ChatL);

            // p2a.Khat = (stanceLegIdx == leftStance) ? solveDeadbeatControllerGain(p2a.AhatR, p2a.BhatR) : solveDeadbeatControllerGain(p2a.AhatL, p2a.BhatL);

            p2a.KhatL = solveDeadbeatControllerGain(p2a.AhatL, p2a.BhatL);
            p2a.KhatR = solveDeadbeatControllerGain(p2a.AhatR, p2a.BhatR);

            if (p2a.outputTracking){

                p2a.solveOutputTrackingGainP2();
            }
            
            

            std::cout << "P2 estimation updated" << std::endl;

            std::cout << "-Kx^* + u*" << -p2a.Khat.transpose()*p2a.Xdes_hatL +p2a.UleftDes << std::endl;
            std::cout << "kf*r" << p2a.kfL*p2a.UleftDes << std::endl;

            std::cout << "kf_L,R = " << p2a.kfL << "," << p2a.kfR << std::endl;
            std::cout << "bf_L,R = " << p2a.bfL << "," << p2a.bfR << std::endl;

        }
        else
        {
            ROS_WARN("stance leg idx is wrong!");
        }



    
}

void AdaptiveHLIP::updateDesiredWalking(double vel, double stepWidth) {
    if (vel < 0.02)
        vel = 0;

    // if (abs(vel_old - vel) < 0.01 ) {
    //     p1a
    // }  

    params.velDes = vel;

    double DistSum = vel * params.T; // total traveled distance in SSPand DSP.


    switch (orbitPeriod) {
    case P1orbit:
        p1a.Udes =  params.velDes * params.T;

        p1a.Xdes_hat = (MatrixXd::Identity(2, 2) - p1a.Ahat).completeOrthogonalDecomposition().solve(p1a.Chat + p1a.Bhat * p1a.Udes);
        
        if (p1a.outputTracking){
            solveOutputTrackingGain(p1a.Ahat, p1a.Bhat, p1a.Chat, p1a.Dhat, p1a.Ehat, p1a.Fhat,p1a.Khat,p1a.Xdes,p1a.Udes,p1a.kf,p1a.bf);
        }  

        break;
    case P2orbit:
        // p2.UleftDes+p2.UrightDes = 2*velDes*T
        if (stepWidth == 0){
            p2a.UleftDes = vel * params.T;
            p2a.UrightDes = vel * params.T;
        }else{
            p2a.UleftDes = -stepWidth;
            p2a.UrightDes = 2 * params.velDes * params.T - p2a.UleftDes;
        }

        p2a.Xdes_hatL = (MatrixXd::Identity(2, 2) - p2a.AhatR * p2a.AhatL).inverse() * (p2a.AhatR * p2a.BhatL * p2a.UleftDes + p2a.BhatR * p2a.UrightDes + p2a.AhatR * p2a.ChatL + p2a.ChatR);
        p2a.Xdes_hatR = (MatrixXd::Identity(2, 2) - p2a.AhatL * p2a.AhatR).inverse() * (p2a.AhatL * p2a.BhatR * p2a.UrightDes + p2a.BhatL * p2a.UleftDes + p2a.AhatL * p2a.ChatR + p2a.ChatL);
        if (p2a.outputTracking){

            p2a.solveOutputTrackingGainP2();
        }
        break;
    default:
        ROS_WARN("orbit type is wrong!");
        break;
    }

    vel_old = vel;
}

/*
void AdaptiveHLIP::updateThetawithAlgorithm(int algorithm,MatrixXd Gamma, VectorXd Phi, VectorXd e, int k,  MatrixXd &P, MatrixXd &Theta){
    
    if (algorithm == AdaptiveHLIP::ProjectionAlgorithm)
        Theta = Theta + Gamma * Phi * e.transpose() / (Phi.transpose() * Phi);

    if (algorithm == AdaptiveHLIP::OrthogonalizedProjectionAlgorithm && Phi.transpose() * P * Phi > 0.00005){
        Theta = Theta + Gamma * P * Phi * e.transpose() / (Phi.transpose() * P * Phi);
        P = P - P*Phi*Phi.transpose()*P / (Phi.transpose() * P * Phi);
    }
        

    if (algorithm == AdaptiveHLIP::LeastSquareAlgorithm){
        P = P - P*Phi*Phi.transpose()*P/  (Phi.transpose()* (P + MatrixXd::Identity(4,4)) * Phi);
        Theta = Theta + P * Phi * e.transpose() / (Phi.transpose() * Phi);
    }
        
    if (algorithm == AdaptiveHLIP::WeightedLeastSquareAlgorithm){
        double ak = sqrt(k + 1);
        P = P - ak*P*Phi*Phi.transpose()*P/  (Phi.transpose()* (P + MatrixXd::Identity(4,4)) * Phi);
        Theta = Theta + sqrt(ak)*P * Phi * e.transpose() / (Phi.transpose() * Phi);
    }  
}

*/

void AdaptiveHLIP::P1A::initialize_estimator(Params params, P1 p1)
{
    Xdes = p1.Xdes;
    sigma1 = p1.sigma1;



    Udes = p1.Udes;
    Ahat = params.A_S2S;
    Bhat = params.B_S2S;

    Khat = p1.K;
    Xdes_hat = p1.Xdes;
    Chat = (MatrixXd::Identity(2, 2) - Ahat) * Xdes_hat - Bhat * Udes;
    
    if (outputTracking) {
        Theta.resize(4, 3);
        Dhat = MatrixXd::Zero(1, 2);
        Ehat = MatrixXd::Identity(1, 1);
        Fhat = MatrixXd::Zero(1, 1);
        Theta << Ahat.transpose(), Dhat.transpose(),
            Bhat.transpose(), Ehat.transpose(),
            Chat.transpose(), Fhat.transpose();
    }
    else {
        Theta.resize(4, 2);
        Theta << Ahat.transpose(), Bhat.transpose(), Chat.transpose();
    }
   

    double gamma;
    ros::param::get("/cassie/locomotion/stepping/adaptive/Xgain", gamma);


    Gamma = gamma * MatrixXd::Identity(4, 4);

    P = MatrixXd::Identity(4, 4);
    std::cout << "P1 estimator started!" << std::endl;
    std::cout << "Xdes_hat = " << Xdes_hat.transpose() << std::endl;


    kf = 0;
};


void AdaptiveHLIP::P1A::updateEstimationParams(Vector2d xkm, double ukm, Vector2d xk, double ykm, int algorithm, int k)
{
    VectorXd Phi(4);
    Phi << xkm,
        ukm,
        1;
    VectorXd e;

    if (outputTracking){
        e.resize(3);
        e << xk - Theta.transpose() * Phi, 
             ykm - Udes;
    }else{
        e.resize(2);
        e << xk - Theta.transpose() * Phi;
    }
    



    if (algorithm == AdaptiveHLIP::ProjectionAlgorithm)
        Theta = Theta + Gamma * Phi * e.transpose() / (Phi.transpose() * Phi);

    if (algorithm == AdaptiveHLIP::OrthogonalizedProjectionAlgorithm && Phi.transpose() * P * Phi > 0.00005){
        Theta = Theta + Gamma * P * Phi * e.transpose() / (Phi.transpose() * P * Phi);
        P = P - P*Phi*Phi.transpose()*P / (Phi.transpose() * P * Phi);
    }
        

    if (algorithm == AdaptiveHLIP::LeastSquareAlgorithm){
        // M = Phi.transpose() * Phi;

        P = P - Gamma* P*Phi*Phi.transpose()*P/  (Phi.transpose()* (P + MatrixXd::Identity(4,4)) * Phi);
        Theta = Theta + Gamma* P * Phi * e.transpose() / (Phi.transpose() * Phi);
    }
        
    if (algorithm == AdaptiveHLIP::WeightedLeastSquareAlgorithm){
        double ak = sqrt(k + 1);
        P = P - ak*P*Phi*Phi.transpose()*P/  (Phi.transpose()* (P + MatrixXd::Identity(4,4)) * Phi);
        Theta = Theta + sqrt(ak)*P * Phi * e.transpose() / (Phi.transpose() * Phi);
    }  

      



};

Vector2d AdaptiveHLIP::P1A::getDeadbeatStepSize(double p, double v, double lambda)
{
    Vector2d Xnow, dXnow;
    Xnow << p, v;
    dXnow << v, pow(lambda, 2) * p;
    double stepLength,dstepLength;

    if (outputTracking){
        stepLength = Khat.transpose() * Xnow + kf*Udes + bf;
        // stepLength = Khat.transpose() * (Xnow - Xdes_hat) + Udes + bf;
    }else{
        stepLength = Khat.transpose() * (Xnow - Xdes_hat) + Udes;

    }

    // //for direct method
    // double stepLength = Khat.transpose() * Xnow + kf*Udes;

    dstepLength = Khat.transpose() * dXnow;

    StepX << stepLength, dstepLength;

    // std::cout << "Xdes_hat = " << Xdes_hat.transpose() << std::endl;
    // std::cout << "Ucommand = " << stepLength << std::endl;
    return StepX;
}

void AdaptiveHLIP::P2A::initialize_estimator(Params params, P2 p2)
{

    XleftDes = p2.XleftDes;
    XrightDes = p2.XrightDes;
    UleftDes = p2.UleftDes;
    UrightDes = p2.UrightDes;

    sigma2 = p2.sigma2;

    d2 = p2.d2;


    Ahat = params.A_S2S;
    Bhat = params.B_S2S;

    Khat = p2.K;
    AhatL = Ahat;
    AhatR = Ahat;
    BhatL = Bhat;
    BhatR = Bhat;

    KhatL = Khat;
    KhatR = Khat;

    Xdes_hatL = XleftDes;
    Xdes_hatR = XrightDes;

    ChatR = Xdes_hatL - Ahat*Xdes_hatR - Bhat * UrightDes;
    ChatL = Xdes_hatR - Ahat*Xdes_hatL - Bhat * UleftDes;

    if (outputTracking) {
        Theta.resize(4, 3);
        ThetaL.resize(4, 3);
        ThetaR.resize(4, 3);

        Dhat = MatrixXd::Zero(1, 2);
        Ehat = MatrixXd::Identity(1, 1);
        Fhat = MatrixXd::Zero(1, 1);
        ThetaL << Ahat.transpose(), Dhat.transpose(),
            Bhat.transpose(), Ehat.transpose(),
            ChatL.transpose(), Fhat.transpose();
        ThetaR << Ahat.transpose(), Dhat.transpose(),
            Bhat.transpose(), Ehat.transpose(),
            ChatR.transpose(), Fhat.transpose();

        DhatL = Dhat;
        EhatL = Ehat;
        FhatL = Fhat;
        DhatR = Dhat;
        EhatR = Ehat;
        FhatR = Fhat;

    }
    else {
        Theta.resize(4, 2);
        ThetaL.resize(4, 2);
        ThetaR.resize(4, 2);

        ThetaL << Ahat.transpose(), Bhat.transpose(), ChatL.transpose();
        ThetaR << Ahat.transpose(), Bhat.transpose(), ChatR.transpose();
    }

    

    double gamma;
    ros::param::get("/cassie/locomotion/stepping/adaptive/Ygain", gamma);
    Gamma = gamma * MatrixXd::Identity(4, 4);


    PL = MatrixXd::Identity(4, 4);
    PR = MatrixXd::Identity(4, 4);
    std::cout << "P2 estimator started!" << std::endl;
};

void AdaptiveHLIP::P2A::updateEstimationParams(Vector2d xkm, double ukm, Vector2d xk, double ykm, int stanceLegIdx, int algorithm, int k){
    VectorXd Phi(4);
    Phi << xkm,
        ukm,
        1;   
    

    MatrixXd Theta(ThetaL);
    MatrixXd P(PL);
    
    Theta = (stanceLegIdx == leftStance) ? ThetaR : ThetaL;
    P = (stanceLegIdx == leftStance) ? PR : PL;
    double udes = (stanceLegIdx == leftStance) ? UrightDes : UleftDes;

    // switch (stanceLegIdx)
    // {
    // case leftStance:
    //     e = xk - ThetaR.transpose()*Phi;
    //     ThetaR = ThetaR + Gamma * Phi * e.transpose() / (Phi.transpose()*Phi);
    //     break;
    // case rightStance:
    //     e = xk - ThetaL.transpose()*Phi;
    //     ThetaL = ThetaL + Gamma * Phi * e.transpose() / (Phi.transpose()*Phi);
    //     break;
    // default:
    //     ROS_WARN("leg idx is wrong!");
    //     break;
    // }



    VectorXd e;

    if (outputTracking){
        e.resize(3);
        e << xk - Theta.transpose() * Phi, 
             ykm - udes;
    }else{
        e.resize(2);
        e << xk - Theta.transpose() * Phi;
    }


    if (algorithm == ProjectionAlgorithm){
        Theta = Theta + Gamma * Phi * e.transpose() / (Phi.transpose() * Phi);
    }
        

    if (algorithm == OrthogonalizedProjectionAlgorithm && Phi.transpose() * P * Phi > 0.00005){
        Theta = Theta + Gamma * P * Phi * e.transpose() / (Phi.transpose() * P * Phi);
        P = P - P*Phi*Phi.transpose()*P / (Phi.transpose() * P * Phi);
    }
        

    if (algorithm == LeastSquareAlgorithm){
        P = P - Gamma*P*Phi*Phi.transpose()*P/  (Phi.transpose()* (P + MatrixXd::Identity(4,4)) * Phi);
        Theta = Theta + Gamma*P * Phi * e.transpose() / (Phi.transpose() * Phi);
    }
        
    if (algorithm == WeightedLeastSquareAlgorithm){
        double ak;
        std::cout << "Phi.transpose() * P * Phi = " << Phi.transpose() * P * Phi << std::endl; 
        if (Phi.transpose() * P * Phi > 0.1){
            ak = 1;
        }else{
            ak = 0.001;
        }
        P = P - ak*P*Phi*Phi.transpose()*P/  (Phi.transpose()* (P + MatrixXd::Identity(4,4)) * Phi);
        Theta = Theta + sqrt(ak)*P * Phi * e.transpose() / (Phi.transpose() * Phi);
    }  

    if (stanceLegIdx == leftStance)
        ThetaR = Theta;
    if (stanceLegIdx == rightStance)
        ThetaL = Theta;    


};

Vector2d AdaptiveHLIP::P2A::getDeadbeatStepSize(double p, double v, double lambda, int stanceLegIdx)
{

    Vector2d Xnow, dXnow;
    Xnow << p, v;
    dXnow << v, pow(lambda, 2) * p;
    double stepLength,dstepLength;

    Xdes_hat = (stanceLegIdx == leftStance) ? Xdes_hatL : Xdes_hatR;
    Udes = (stanceLegIdx == leftStance) ? UleftDes : UrightDes;
    kf = (stanceLegIdx == leftStance) ? kfL : kfR;
    bf = (stanceLegIdx == leftStance) ? bfL : bfR;
    Khat = (stanceLegIdx == leftStance) ? KhatL : KhatR;

    if (outputTracking){
        stepLength = Khat.transpose() * Xnow + kf*Udes + bf; 
    }else{
        stepLength = Khat.transpose() * (Xnow - Xdes_hat) + Udes;
    }

    StepX << stepLength, dstepLength;
    return StepX;
}

void AdaptiveHLIP::P2A::solveOutputTrackingGainP2(){
    MatrixXd ML(1,2),MR(1,2);

    ML = (DhatL+EhatL*KhatL)*(MatrixXd::Identity(2,2)-(AhatR+BhatR*KhatR)*(AhatL+BhatL*KhatL)).inverse();
    MR = (DhatR+EhatR*KhatR)*(MatrixXd::Identity(2,2)-(AhatL+BhatL*KhatL)*(AhatR+BhatR*KhatR)).inverse();

    MatrixXd rL = UleftDes*MatrixXd::Identity(1,1);
    MatrixXd rR = UrightDes*MatrixXd::Identity(1,1);

    Matrix2d tmp;
    tmp << ML*(AhatR+BhatR*KhatR)*BhatL+EhatL, ML*BhatR,
           MR*BhatL, MR*(AhatL+BhatL*KhatL)*BhatR+EhatR;

    Vector2d tmp_v, b;
    tmp_v << rL - FhatL - ML*( (AhatR+BhatR*KhatR)*ChatL + ChatR),
             rR - FhatR - MR*( (AhatL+BhatL*KhatL)*ChatR + ChatL);

    b << tmp.completeOrthogonalDecomposition().solve(tmp_v);

    if (rL(0,0) != 0){
        kfL = b(0)/rL(0,0);
        bfL = 0;
    }else{
        bfL = b(0);
        kfL = 0;
    }

    if (rR(0,0) != 0){
        kfR = b(1)/rR(0,0);
        bfR = 0;
    }else{
        bfR = b(1);
        kfR = 0;
    }





}