#pragma once

#include <memory>
#include "biped_core/robot_base_pinocchio.hpp"
#include "biped_core/outputs.hpp"
#include <Eigen/Dense>
#include "biped_utils/bezier_tools.hpp"

using namespace bezier_tools;

class OutputBase : public Outputs
{
public:
    OutputBase(std::shared_ptr<RobotBasePinocchio> robot);

    //virtual destructor
    virtual ~OutputBase() {};

    //TODO: virtual or not
    virtual void ForwardPosIk(const VectorXd &qk,  VectorXd &fk, MatrixXd &Jk);
    

    virtual void reset(){};

    virtual void UpdateOutput(const VectorXd &reference, const double &t, const double &t_old) = 0;

    virtual void ComputeActual() = 0;

    virtual bool isReadyToTransition() const { return true; }  // Base implementation

    virtual std::vector<VectorXd> CollectLog() const { return std::vector<VectorXd>{VectorXd::Zero(0)}; } // Base implementation

    virtual VectorXd NaiveUpperJointsDesired(const VectorXd& default_q){return default_q;};


protected:
    void setBezierDesiredOutputs(const VectorXd &alpha, const double &tau, const double &dtau, const int &OutputIdx);

    void setZeroOutputs(const int &OutputIdx);

    static std::vector<int> generate_full_y_idx(int nY){
        std::vector<int> full_y_idx;
        for (int i = 0; i < nY; i++)
        {
              full_y_idx.push_back(i);
        }
        return full_y_idx;
  }

    std::shared_ptr<RobotBasePinocchio> robot_;
    

    struct Contact
    {
        FootContactStatus leftC;
        FootContactStatus rightC;
    };
};
