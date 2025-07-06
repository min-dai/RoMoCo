#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <vector>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/center-of-mass-derivatives.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/centroidal-derivatives.hpp>

#include <pinocchio/algorithm/model.hpp>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include "biped_utils/kinematics.hpp"
#include "biped_core/biped_constants.hpp"
#include "biped_core/friction_params.hpp"

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::VectorXd;
class RobotBasePinocchio
{
public:
    // Constructor for RobotBasePinocchio.
    // @param urdf_path: Path to the robot's URDF file.
    // @param locked_joints_names: Names of joints to be locked (fixed) in the model.
    // @param locked_joints_q: Values for the locked joints (must match the size of locked_joints_names).
    explicit RobotBasePinocchio(const std::string &urdf_path, const std::vector<std::string> &locked_joints_names = {}, const VectorXd &locked_joints_q = VectorXd::Zero(0));
    // virtual destructor
    virtual ~RobotBasePinocchio() = default;

    // define the robot type: LineFoot or PlaneFoot
    virtual RobotType robot_type() const = 0;

    void Init();

    double mass() const { return mass_; }
    int nq() const { return model_.nq; }
    int nv() const { return model_.nv; }
    int nu() const { return nu_; }
    const VectorXd &q() const { return q_; }
    const VectorXd &dq() const { return dq_; }

    const MatrixXd &D() const { return data_.M; }
    const VectorXd &G() const { return data_.g; }
    const VectorXd &H() const { return data_.nle; }
    const MatrixXd &B() const { return B_; }
    const VectorXd &u_lb() const { return u_lb_; }
    const VectorXd &u_ub() const { return u_ub_; }

    VectorXd hg_angular() const { return data_.hg.angular(); }
    VectorXd hg_linear() const { return data_.hg.linear(); }
    // hg stacks linear and angular momentum
    VectorXd hg() const { return data_.hg.toVector(); }
    // Ag stacks linear and angular momentum matrix, top 3 rows are linear, bottom 3 rows are angular
    MatrixXd Ag() const { return data_.Ag; }
    MatrixXd dAg() const { return data_.dAg; }

    MatrixXd ComputeCentroidalMomentumMatrix();
    MatrixXd ComputeCentroidalMomentumMatrixTimeVariation();

    virtual std::vector<int> actuated_q_idx(AnkleMotorStatus left_ankle_status, AnkleMotorStatus right_ankle_status) const = 0;
    virtual std::vector<int> actuated_u_idx(AnkleMotorStatus left_ankle_status, AnkleMotorStatus right_ankle_status) const = 0;

    // Kinematics
    const Kinematics3D &com_kinematics() const { return com_.kinematics; }
    const Kinematics3D &base_kinematics() const { return base_.kinematics; }

    virtual Kinematics3D left_toe_kinematics() const = 0;
    virtual Kinematics3D right_toe_kinematics() const = 0;
    virtual Kinematics3D left_heel_kinematics() const = 0;
    virtual Kinematics3D right_heel_kinematics() const = 0;

    const Kinematics3D &left_ankle_kinematics() const { return left_ankle_.kinematics; }
    const Kinematics3D &right_ankle_kinematics() const { return right_ankle_.kinematics; }
    const Kinematics3D &left_mid_foot_kinematics() const { return left_mid_foot_.kinematics; }
    const Kinematics3D &right_mid_foot_kinematics() const { return right_mid_foot_.kinematics; }
    const Kinematics3D &left_below_ankle_kinematics() const { return left_below_ankle_.kinematics; }
    const Kinematics3D &right_below_ankle_kinematics() const { return right_below_ankle_.kinematics; }

    const Kinematics3D &left_hip_kinematics() const { return left_hip_.kinematics; }
    const Kinematics3D &right_hip_kinematics() const { return right_hip_.kinematics; }
    const Kinematics1D &left_hip_yaw_kinematics() const { return left_hip_yaw_.kinematics; }
    const Kinematics1D &right_hip_yaw_kinematics() const { return right_hip_yaw_.kinematics; }

    // for robots with internal holonomic constraints, like closed loop linkages
    virtual void GetInternalHolonomicConstraints(MatrixXd &Jh, VectorXd &dJhdq)
    {
        Jh.resize(0, nv());
        dJhdq.setZero(0);
    }
    // for ground contact
    virtual void GetContactHolonomicConstraints(const FootContactStatus leftC, const FootContactStatus rightC, MatrixXd &Jh, VectorXd &dJhdq, const Matrix3d Rground = MatrixXd::Identity(3, 3)) = 0;
    virtual void GetFrictionCone(const FrictionParams fric_params, const FootContactStatus leftC, const FootContactStatus rightC, MatrixXd &Acone, VectorXd &bcone) = 0;

    // update kinematics and dynamics using pinocchio
    void UpdateAll(const Eigen::VectorXd &q, const Eigen::VectorXd &dq);
    virtual void UpdateDynamics(const Eigen::VectorXd &q, const Eigen::VectorXd &dq);
    void UpdateKinematics(const Eigen::VectorXd &q, const Eigen::VectorXd &dq);

    void UpdateAllZeroBase();
    void UpdateKinematicsZeroBase();

    void ComputeForwardKinematics(const Eigen::VectorXd &q, const Eigen::VectorXd &dq);

    VectorXd ComputeCentroidalMomentum();
    VectorXd ComputeCentroidalAngularMomentum();

    struct FrameKinematics3D
    {
        pinocchio::FrameIndex frame_id;
        bool local_world_aligned = true; // if false, it is local frame
        Kinematics3D kinematics;
        // Default constructor: Initializes frame_id to an invalid value
        FrameKinematics3D() : frame_id(pinocchio::FrameIndex(-1)) {}
        // Constructor with frame ID
        FrameKinematics3D(pinocchio::FrameIndex id) : frame_id(id) {}
        // WARNING: Update must be called after second order forward kinematics (e.g., pinocchio::forwardKinematics with acceleration)
        // because this function relies on velocity and acceleration data in pinocchio::Data being up-to-date.
        // Calling Update before these quantities are computed may result in incorrect kinematic values.
        void Update(pinocchio::Model &model, pinocchio::Data &data);
        void Init(int nv);

        Matrix3d RotationMatrix(const pinocchio::Data &data) const
        {
            return data.oMf[frame_id].rotation();
        }
    };

    struct COMKinematics3D
    {
        Kinematics3D kinematics;
        void Update(pinocchio::Model &model, pinocchio::Data &data);
        void Init(int nv);
    };

    struct JointKinematics1D
    {
        int joint_id;
        Kinematics1D kinematics;
        // default constructor
        JointKinematics1D() : joint_id(-1) {}
        // constructor with joint id
        JointKinematics1D(int id) : joint_id(id) {}
        void Update(const Eigen::VectorXd &q, const Eigen::VectorXd &dq);
        void Init(int nv);
    };

    virtual std::vector<std::reference_wrapper<FrameKinematics3D>> GetAllFrameKinematics() = 0;
    virtual std::vector<std::reference_wrapper<JointKinematics1D>> GetAllJointKinematics();
    virtual std::vector<std::pair<std::reference_wrapper<RobotBasePinocchio::FrameKinematics3D>, std::string>> GetFrameIds() = 0;

    // defined in final robot type classes
    virtual void AddFrames() = 0;
    virtual void InitJointKinematics() = 0;
    virtual void InitActuation() = 0;
    virtual Kinematics1D GetLeftFootDeltaPitch() { return Kinematics1D(0, nv()); };
    virtual Kinematics1D GetRightFootDeltaPitch() { return Kinematics1D(0, nv()); };
    virtual Kinematics1D GetLeftFootDeltaRoll() { return Kinematics1D(0, nv()); };
    virtual Kinematics1D GetRightFootDeltaRoll() { return Kinematics1D(0, nv()); };
    virtual Kinematics1D GetBaseDeltaPitch() { return Kinematics1D(0, nv()); };
    virtual Kinematics1D GetBaseDeltaRoll() { return Kinematics1D(0, nv()); };

    double GetLeftToeYaw() const;
    double GetRightToeYaw() const;
    Eigen::Matrix3d GetLeftToeRyaw() const;
    Eigen::Matrix3d GetRightToeRyaw() const;
    Eigen::Matrix3d GetBaseR_yaw() const
    {
        return Eigen::AngleAxisd(q_(BaseRotZ), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    }

protected:
    pinocchio::Model model_; // Pinocchio model
    pinocchio::Data data_;   // Pinocchio data

    // Some frame and joint kinematics needed
    FrameKinematics3D left_below_ankle_, right_below_ankle_;
    FrameKinematics3D left_mid_foot_, right_mid_foot_;
    FrameKinematics3D left_ankle_, right_ankle_;
    FrameKinematics3D left_hip_, right_hip_;

    JointKinematics1D left_hip_yaw_, right_hip_yaw_;

    COMKinematics3D com_;
    FrameKinematics3D base_;
    FrameKinematics3D baseF_, baseB_, baseL_, baseR_;

    int nu_;

    Eigen::VectorXd q_;  // Joint positions
    Eigen::VectorXd dq_; // Joint velocities

    MatrixXd B_; // Declare the input matrix
    VectorXd u_lb_;
    VectorXd u_ub_;

private:
    double mass_;
    std::vector<pinocchio::JointIndex> JointNamesToIds(const std::vector<std::string> &joint_names);
};
