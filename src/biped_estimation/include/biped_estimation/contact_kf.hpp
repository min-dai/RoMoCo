#ifndef CONTACT_KF_HPP
#define CONTACT_KF_HPP

#include "biped_utils/filters.hpp"

#include "biped_utils/yaml_parser.hpp"


class ContactKf
{

public:
   ContactKf(const std::string &config_file, int n_encoders);


   void Update(double t, const VectorXd& aIn, const MatrixXd& R, double lC, double rC, const Vector3d &plf, const Vector3d &prf, const MatrixXd &J_lf, const MatrixXd &J_rf);

   void InitContact(double t, double lC, double rC, const VectorXd& plf, const VectorXd& prf);

   void Reset();

   void UnpackState(MatrixXd &x, VectorXd &p, VectorXd &v, VectorXd &plf, VectorXd &prf, VectorXd &ba);
   void PackState(MatrixXd &x, const VectorXd &p, const VectorXd &v, const VectorXd &plf, const VectorXd &prf, const VectorXd &ba);


   Vector3d getvel() { return x_hat.col(1); }

   Vector3d vel = Vector3d::Zero();
   Vector3d vel_pre = Vector3d::Zero();
   MatrixXd x_hat; // A Posteriori state

   VectorXd yk = VectorXd::Zero(6);


   struct Config
   {
      // Measurement noise
      double base_process_std;
      double foothold_std;
      double encoder_std;
      // sensor model
      double acceleration_bias_std;

      // Initialized std estimate for covariance
      double init_position_std = 0.01;
      double init_velocity_std = 0.01;
      double init_acc_bias_std = 0.1;
      double init_foothold_std = 0.025;

      YAMLParser yaml_parser;
      void Init();
   } config;

private:
   void PredictStep(double dt, const Matrix3d& R, const Vector3d& acc_measured, double lC, double rC);
   void UpdateStep(const Vector3d& plf_enc, const Vector3d& prf_enc, const MatrixXd& J_lf, const MatrixXd& J_rf);

   

   Vector3d g_;


   MatrixXd Fk_;
   MatrixXd Hk_;
   MatrixXd Sk_;
   MatrixXd Kk_;

   MatrixXd P;  // A Posteriori covariance
   MatrixXd Qc; // Continuous covariance, state transition
   MatrixXd Rc; // Continuous covariance, measurement




   MatrixXd Renc;
   bool is_initialized_;

   double tprev_;

};

#endif
