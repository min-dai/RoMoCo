#ifndef ROMOCO_OUTPUT_BASE_HPP
#define ROMOCO_OUTPUT_BASE_HPP

#include <memory>
#include "romoco_core/robot_base_pinocchio.hpp"
#include <Eigen/Dense>

#include "romoco_core/biped_commands.hpp"

namespace romoco
{
/**
 * @class OutputBase
 * @brief An abstract base class for robot output generation, providing common functionality for different output strategies.
 * @ingroup group_controller
 * This class serves as a foundation for generating desired outputs for biped robots, handling the selection of actuated joints, 
 * and management of output states. It is designed to be extended by specific output strategies.
 */
class OutputBase 
{
public:
    OutputBase(std::shared_ptr<romoco::robot::RobotBasePinocchio> robot);

    int nh(){return Jh_.rows();}
    int nFc(){return Afric_.cols();} //number of forces corresponds to Contact Force
    int nu(){return actuated_u_idx_.size();}
    int ny(){return active_y_idx_.size();}
    int nfric(){return bfric_ub_.size();} //rows of friction cone constraints

    Eigen::MatrixXd B(){return robot_->B()(Eigen::all, actuated_u_idx_);}

    Eigen::VectorXd ya_full() const { return ya_; }
    Eigen::VectorXd dya_full() const { return dya_; }
    Eigen::MatrixXd Jya_full() const { return Jya_; }
    Eigen::VectorXd dJyadq_full() const { return dJyadq_; }
    Eigen::VectorXd yd_full() const { return yd_; }
    Eigen::VectorXd dyd_full() const { return dyd_; }
    Eigen::VectorXd d2yd_full() const { return d2yd_; }

    Eigen::VectorXd ya() const { return ya_(active_y_idx_); }
    Eigen::VectorXd dya() const { return dya_(active_y_idx_); }
    Eigen::MatrixXd Jya() const { return Jya_(active_y_idx_, Eigen::all); }
    Eigen::VectorXd dJyadq() const { return dJyadq_(active_y_idx_); }
    Eigen::VectorXd yd() const { return yd_(active_y_idx_); }
    Eigen::VectorXd dyd() const { return dyd_(active_y_idx_); }
    Eigen::VectorXd d2yd() const { return d2yd_(active_y_idx_); }
    Eigen::MatrixXd Jh() const { return Jh_; }
    Eigen::VectorXd dJhdq() const { return dJhdq_; }
    Eigen::MatrixXd Afric() const { return Afric_; }
    Eigen::VectorXd bfric_ub() const { return bfric_ub_; }


    /**
     * @brief Maps the reduced motor commands to the full motor space.
     * @param u The input vector containing reduced motor commands.
     * @return Eigen::VectorXd A vector containing the full motor commands.
     */
    Eigen::VectorXd MapToFullMotors(const Eigen::VectorXd &u);
    //select actuated joints from full joint vector q or qdot, as we assume robot_->nq() = robot_->nv()
    /**
     * @brief Selects and returns the actuated joint values from the input joint vector.
     * @param q_in The input joint vector containing all joint values.
     * @return Eigen::VectorXd A vector containing only the actuated joint values.
     * @throws std::invalid_argument if the size of \p q_in does not match the robot's joint size.
     */
    Eigen::VectorXd SelectActuatedStates(const Eigen::VectorXd &q_in);
    /**
     * @brief Selects and returns the actuated motor values from the input motor vector.
     * @param u_in The input motor vector containing all motor values.
     * @return Eigen::VectorXd A vector containing only the actuated motor values.
     * @throws std::invalid_argument if the size of \p u_in does not match the robot's actuator size.
     */
    Eigen::VectorXd SelectActuatedMotors(const Eigen::VectorXd &u_in);
    /**
     * @brief Selects and returns the active output values from the input output vector.
     * @param y_in The input output vector containing all output values.
     * @return Eigen::VectorXd A vector containing only the active output values.
     * @throws std::invalid_argument if the size of \p y_in is smaller than the number of outputs.
     */
    Eigen::VectorXd SelectActiveOutputs(const Eigen::VectorXd &y_in);


    //virtual destructor
    virtual ~OutputBase() {};

    virtual void Reset(){};

    virtual void UpdateOutput(const DesiredCommand &command, const double &t, const double &t_old) = 0;

    virtual void ComputeActual() = 0;

    virtual bool is_ready_to_transit() const { return true; }  // Base implementation

    virtual std::vector<Eigen::VectorXd> CollectLog() const { return std::vector<Eigen::VectorXd>{Eigen::VectorXd::Zero(0)}; } // Base implementation

    virtual Eigen::VectorXd NaiveUpperJointsDesired(const Eigen::VectorXd& default_q){return default_q;};

    void ForwardPosIK(const Eigen::VectorXd &qk,  Eigen::VectorXd &fk, Eigen::MatrixXd &Jk);


protected:
    Eigen::VectorXd ya_;
    Eigen::VectorXd dya_;
    Eigen::MatrixXd Jya_;
    Eigen::VectorXd dJyadq_;
    Eigen::VectorXd yd_;
    Eigen::VectorXd dyd_;
    Eigen::VectorXd d2yd_;
    Eigen::MatrixXd Jh_;
    Eigen::VectorXd dJhdq_;
    Eigen::MatrixXd Afric_;
    Eigen::VectorXd bfric_ub_;


    int nY_;

    //actuated indices in actuator list
    //mostly just full list {0,1,2,...,nu}
    //use to deactivate stance ankle torque for LIP walking
    std::vector<int> actuated_u_idx_;
    std::vector<int> actuated_q_idx_;
    std::vector<int> active_y_idx_;

    void SetOutputSize(int nq, int ny);

    void SetBezierDesiredOutputs(const Eigen::VectorXd &alpha, const double &tau, const double &dtau, const int &OutputIdx);

    void SetBezierDesiredOutputsZeroD2yd(const Eigen::VectorXd &alpha, const double &tau, const double &dtau, const int &OutputIdx);


    std::vector<int> generate_full_y_idx(int nY){
        std::vector<int> full_y_idx;
        for (int i = 0; i < nY; i++)
        {
              full_y_idx.push_back(i);
        }
        return full_y_idx;
  }

    std::shared_ptr<romoco::robot::RobotBasePinocchio> robot_;
    

    struct Contact
    {
        FootContactStatus leftC;
        FootContactStatus rightC;
    };

private:
    Eigen::VectorXd MapU2FullIdx(const Eigen::VectorXd &u, const std::vector<int> &actuated_u_idx, const int &full_size);
};

} // namespace romoco

#endif // ROMOCO_OUTPUT_BASE_HPP