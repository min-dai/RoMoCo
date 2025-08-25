#include <biped_estimation/contact_kf.hpp>

using namespace std;

ContactKf::ContactKf(const std::string &config_file, int n_encoders)
{
   config.yaml_parser.Init(config_file);
   config.Init();

   MatrixXd I3 = MatrixXd::Identity(3, 3);
   Qc.resize(15, 15);
   Qc.setZero();
   Qc.block(0, 0, 3, 3) = config.base_process_std * config.base_process_std * I3;
   Qc.block(3, 3, 3, 3) = config.base_process_std * config.base_process_std * I3;
   Qc.block(6, 6, 3, 3) = config.foothold_std * config.foothold_std * I3;
   Qc.block(9, 9, 3, 3) = config.foothold_std * config.foothold_std * I3;
   Qc.block(12, 12, 3, 3) = config.acceleration_bias_std * config.acceleration_bias_std * I3;

   Rc = MatrixXd::Zero(n_encoders, n_encoders);


   Fk_ = MatrixXd::Identity(15,15);

   // Hk_ is constant
   Hk_ = MatrixXd::Zero(6, 15);
   Hk_.block(0, 0, 3, 3) = -I3;
   Hk_.block(3, 0, 3, 3) = -I3;
   Hk_.block(0, 6, 3, 3) = I3;
   Hk_.block(3, 9, 3, 3) = I3;

   Sk_ = MatrixXd::Zero(6, 6);
   Kk_ = MatrixXd::Zero(15, 6);
}

void ContactKf::Config::Init(){
    base_process_std = yaml_parser.get_double("base_process_std");
    foothold_std = yaml_parser.get_double("foothold_std");
    encoder_std = yaml_parser.get_double("encoder_std");
    acceleration_bias_std = yaml_parser.get_double("acceleration_bias_std");
}

void ContactKf::Update(double t, const VectorXd& aIn, const MatrixXd& R, double lC, double rC, const Vector3d &plf, const Vector3d &prf, const MatrixXd &J_lf, const MatrixXd &J_rf) {

   //  EulerAnglesZYXd eul_imu(robot->q(BaseRotZ),robot->q(BaseRotY), robot->q(BaseRotX));
   //  R = eul_imu.toRotationMatrix();

    // Does nothing if initialized already
    InitContact(t, lC, rC, plf, prf);

    // Update timer
    double dt = t - tprev_;
    tprev_ = t;

    // If filter enabled, run
    if (is_initialized_){
        // Update contacts
        // compute the prediction step
        PredictStep(dt,R,aIn,lC,rC);
        vel_pre =  getvel();

        if (lC > 0.5 || rC > 0.5){
            // Compute the measurement
            UpdateStep(plf, prf, J_lf, J_rf);
        }
    } 


}

void ContactKf::InitContact(double t, double lC, double rC, const VectorXd& plf, const VectorXd& prf)
{
   // We initialize if time has passed and if both feet are in contact with the ground
   if (!is_initialized_)
   {
      if (lC > 0.2 && rC > 0.2)
      {
         this->x_hat(2, 0) = -(plf(2) + prf(2)) / 2.0;
         this->x_hat.col(2) = this->x_hat.col(0) + plf;
         this->x_hat.col(3) = this->x_hat.col(0) + prf;

         MatrixXd tmpI = MatrixXd::Identity(3, 3);
         P.resize(15, 15);
         P.setZero();
         P.block(0, 0, 3, 3) = config.init_position_std * config.init_position_std * tmpI;
         P.block(3, 3, 3, 3) = config.init_velocity_std * config.init_velocity_std * tmpI;
         P.block(6, 6, 3, 3) = config.init_foothold_std * config.init_foothold_std * tmpI;
         P.block(9, 9, 3, 3) = config.init_foothold_std * config.init_foothold_std * tmpI;
         P.block(12, 12, 3, 3) = config.init_acc_bias_std * config.init_acc_bias_std * tmpI;

         this->is_initialized_ = true;
         this->tprev_ = t;
      }
      else
      {
         // Don't start filter until both feet touch the ground
         // return;
      }
   }
}

void ContactKf::PredictStep(double dt, const Matrix3d& R, const Vector3d& acc_measured, double lC, double rC){
    VectorXd pm(3), vm(3), plfm(3), prfm(3), bam(3);
    this->UnpackState(this->x_hat,pm,vm,plfm,prfm,bam);

    // Sensor model: a = acc_measured - bam;

    // Propagate the state
    VectorXd p(3), v(3), plf(3), prf(3), ba(3);
    p = pm + vm*dt + 0.5*(R*(acc_measured - bam) + this->g_)*dt*dt;
    v = vm + (R*(acc_measured - bam) + this->g_)*dt;
    ba = bam;
    plf = plfm;
    prf = prfm;

    // Write as standard form 
    // xhat_k = Fk * x_k-1 + Bk * uk + wk, wk ~ N(0,Qk)
    //       p             v           plf         prf         ba
    // Fk = [eye(3),     eye(3)*dt,   zeros(3),   zeros(3),  -Rotm*dt^2/2; 
    //       zeros(3),   eye(3),      zeros(3),   zeros(3),  -Rotm*dt;
    //       zeros(3),   zeros(3),    eye(3),     zeros(3),  zeros(3);
    //       zeros(3),   zeros(3),    zeros(3),   eye(3),    zeros(3); 
    //       zeros(3),   zeros(3),    zeros(3),   zeros(3),  eye(3)];
    MatrixXd eye3 = MatrixXd::Identity(3,3);
    Fk_.block(0,0,3,3)   = eye3;
    Fk_.block(0,3,3,3)   = eye3*dt;
    Fk_.block(0,12,3,3)  = -R*(dt*dt/2.0);
    Fk_.block(3,12,3,3)  = -R*dt;

    // MatrixXd Ft = MatrixXd::Identity(12,12);
    // Ft.block(0,3,3,3)   = eye3*dt;
    
    MatrixXd Qk_ = this->Qc;

    if (lC < rC){
        Qk_.block(6,6,3,3) = 1000000.*eye3;
        Qk_.block(9,9,3,3) = config.foothold_std * config.foothold_std * eye3;
    } else if (lC > rC){
        Qk_.block(6,6,3,3) = config.foothold_std * config.foothold_std * eye3;
        Qk_.block(9,9,3,3) = 1000000.*eye3;
    } else{
        Qk_.block(6,6,3,3) = config.foothold_std * config.foothold_std * eye3;
        Qk_.block(9,9,3,3) = config.foothold_std * config.foothold_std * eye3;
    }

    this->P = Fk_*this->P*Fk_.transpose() + Qk_;

    // pack the state to use in Update
    this->PackState(this->x_hat,p,v,plf,prf,ba);

}


//@param: plf_enc, prf_enc: 3-by-1 vector, position of left/right foot w.r.t. pelvis, in world frame
//@param: J_lf, J_rf, 3-by-nlegjoints: Jacobians of left/right foot position w.r.t. left/right joints 
void ContactKf::UpdateStep(const Vector3d& plf_enc, const Vector3d& prf_enc, const MatrixXd& J_lf, const MatrixXd& J_rf){
   // yk = zk - Hk * xhat_k|k-1

    // Compute h(x,z)
    VectorXd p(3), v(3), plf(3), prf(3), ba(3);
    this->UnpackState(this->x_hat,p,v,plf,prf,ba);
    
   //  MatrixXd JLf(3,18), JRf(3,18);
   //  SymFunction::J_position_leftFoot(JLf,q_zeroAngle);
   //  SymFunction::J_position_rightFoot(JRf,q_zeroAngle);
   //  MatrixXd J_computed_lf(3,6), J_computed_rf(3,6);
   //  J_computed_lf = JLf.block(0,LeftHipRoll,3,6);
   //  J_computed_rf = JRf.block(0,RightHipRoll,3,6);
    VectorXd y;
    y.resize(6); y.setZero();
    y.segment(0,3) = plf_enc - (plf - p);
    y.segment(3,3) = prf_enc - (prf - p);

    // Compute covariance from encoders
    Rc.setZero();
    Rc.block(0,0,3,3) = J_lf*Renc*J_lf.transpose();
    Rc.block(3,3,3,3) = J_rf*Renc*J_rf.transpose();


    
   //  MatrixXd eye3 = MatrixXd::Identity(3,3);
   //  if (lC < .1){
   //      Rc.block(0,0,3,3) = 1000000.*eye3;
   //  } else if (rC < .1){
   //      Rc.block(3,3,3,3) = 1000000.*eye3;
   //  } 



    // Kalman gain
 
    VectorXd dx = VectorXd::Zero(15);
    


    Sk_ = Hk_*P*Hk_.transpose() + Rc;
    Kk_ = (P*Hk_.transpose())*Sk_.inverse();

    dx = Kk_*y;
    P = (MatrixXd::Identity(15,15) - Kk_*Hk_)*P;

    // Pack state
    this->x_hat.col(0) = this->x_hat.col(0) + dx.segment(0,3);  // p
    this->x_hat.col(1) = this->x_hat.col(1) + dx.segment(3,3);  // v
    this->x_hat.col(2) = this->x_hat.col(2) + dx.segment(6,3);  // plf
    this->x_hat.col(3) = this->x_hat.col(3) + dx.segment(9,3);  // prf
    this->x_hat.col(4) = this->x_hat.col(4) + dx.segment(12,3); // ba




}


void ContactKf::Reset(){
   is_initialized_ = false;
}

void ContactKf::UnpackState(MatrixXd& x, VectorXd& p, VectorXd& v, VectorXd& plf, VectorXd& prf, VectorXd& ba){
    p.resize(3); v.resize(3); plf.resize(3); prf.resize(3); ba.resize(3);
    p   = x.col(0);
    v   = x.col(1);
    plf = x.col(2);
    prf = x.col(3);
    ba  = x.col(4);

}

void ContactKf::PackState(MatrixXd& x, const VectorXd& p, const VectorXd& v, const VectorXd& plf, const VectorXd& prf, const VectorXd& ba){
    x.resize(3,5); x.setZero();
    x.col(0) = p;
    x.col(1) = v;
    x.col(2) = plf;
    x.col(3) = prf;
    x.col(4) = ba;
}




