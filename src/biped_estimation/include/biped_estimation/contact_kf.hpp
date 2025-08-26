#ifndef CONTACT_KF_HPP
#define CONTACT_KF_HPP

#include "biped_utils/filters.hpp"

#include "biped_utils/yaml_parser.hpp"


class ContactKf
{

public:
   ContactKf(const std::string &config_file, int n_encoders);


   void Update(double t, const Eigen::VectorXd& aIn, const Eigen::MatrixXd& R, double lC, double rC, const Eigen::Vector3d &plf, const Eigen::Vector3d &prf, const Eigen::MatrixXd &J_lf, const Eigen::MatrixXd &J_rf);

   void InitContact(double t, double lC, double rC, const Eigen::VectorXd& plf, const Eigen::VectorXd& prf);

   void Reset();

   void UnpackState(Eigen::MatrixXd &x, Eigen::VectorXd &p, Eigen::VectorXd &v, Eigen::VectorXd &plf, Eigen::VectorXd &prf, Eigen::VectorXd &ba);
   void PackState(Eigen::MatrixXd &x, const Eigen::VectorXd &p, const Eigen::VectorXd &v, const Eigen::VectorXd &plf, const Eigen::VectorXd &prf, const Eigen::VectorXd &ba);


   Eigen::Vector3d getvel() { return x_hat.col(1); }

   Eigen::Vector3d vel = Eigen::Vector3d::Zero();
   Eigen::Vector3d vel_pre = Eigen::Vector3d::Zero();
   Eigen::MatrixXd x_hat; // A Posteriori state

   Eigen::VectorXd yk = Eigen::VectorXd::Zero(6);


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
   void PredictStep(double dt, const Eigen::Matrix3d& R, const Eigen::Vector3d& acc_measured, double lC, double rC);
   void UpdateStep(const Eigen::Vector3d& plf_enc, const Eigen::Vector3d& prf_enc, const Eigen::MatrixXd& J_lf, const Eigen::MatrixXd& J_rf);

   Eigen::Vector3d g_;


   Eigen::MatrixXd Fk_;
   Eigen::MatrixXd Hk_;
   Eigen::MatrixXd Sk_;
   Eigen::MatrixXd Kk_;

   Eigen::MatrixXd P;  // A Posteriori covariance
   Eigen::MatrixXd Qc; // Continuous covariance, state transition
   Eigen::MatrixXd Rc; // Continuous covariance, measurement



   Eigen::MatrixXd Renc;
   bool is_initialized_;

   double tprev_;

};

#endif
