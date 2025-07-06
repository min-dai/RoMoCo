
#include "biped_core/robot_base_pinocchio.hpp"

RobotBasePinocchio::RobotBasePinocchio(const std::string &urdf_path, const std::vector<std::string> &locked_joints_names, const VectorXd &locked_joints_q)
{
    pinocchio::JointModelComposite jointComposite(2);
    jointComposite.addJoint(pinocchio::JointModelTranslation());
    jointComposite.addJoint(pinocchio::JointModelSphericalZYX());

    pinocchio::urdf::buildModel(urdf_path,jointComposite, model_);

    if (locked_joints_names.size() > 0)
    {
        std::vector<pinocchio::JointIndex> locked_joints_ids = JointNamesToIds(locked_joints_names);

        Eigen::VectorXd q_neutral = pinocchio::neutral(model_);
        for (size_t i = 0; i < locked_joints_ids.size(); ++i)
        {
            pinocchio::JointIndex joint_id = locked_joints_ids[i];
            const auto &joint_model = model_.joints[joint_id];
            int idx_q = joint_model.idx_q();
            q_neutral(idx_q) = locked_joints_q(i);
        }

        model_ = pinocchio::buildReducedModel(model_, locked_joints_ids, q_neutral);
    }

    std::cout << "List of joints in the model:" << std::endl;
    for (pinocchio::JointIndex joint_id = 0; joint_id < model_.joints.size(); ++joint_id)
        std::cout << "\t- " << model_.names[joint_id] << std::endl;


    mass_ = pinocchio::computeTotalMass(model_);
    std::cout << "Total mass: " << mass_ << std::endl;
}


void RobotBasePinocchio::Init()
{
   // Add the initialization of the model specific variables here
   AddFrames();

   // print all frame names
   for (int i = 0; i < model_.frames.size(); i++)
   {
      std::cout << "frame name: " << model_.frames[i].name << std::endl;
   }

   // Now assign frame IDs
   std::vector<std::pair<std::reference_wrapper<RobotBasePinocchio::FrameKinematics3D>, std::string>> frame_ids = GetFrameIds();
   
   for (auto &frame_id : frame_ids)
   {
      frame_id.first.get().frame_id = model_.getFrameId(frame_id.second);
   }


   data_ = pinocchio::Data(model_);

   InitJointKinematics();
   InitActuation();

   // Initialize the frame kinematics
   for (auto &frame : GetAllFrameKinematics())
   {
      frame.get().Init(nv());
   }
   for (auto &joint : GetAllJointKinematics())
   {
      joint.get().Init(nv());
   }
   com_.Init(nv());
}

std::vector<pinocchio::JointIndex> RobotBasePinocchio::JointNamesToIds(const std::vector<std::string> &joint_names)
{
    std::vector<pinocchio::JointIndex> joint_ids;
    for (const auto &joint_name : joint_names)
    {
        if (!model_.existJointName(joint_name))
        {
            throw std::runtime_error("Joint \"" + joint_name + "\" does not exist in the model.");
        }
        joint_ids.push_back(model_.getJointId(joint_name));
    }
    return joint_ids;
}

void RobotBasePinocchio::UpdateAll(const Eigen::VectorXd &q, const Eigen::VectorXd &dq)
{
    q_ = q;
    dq_ = dq;
    UpdateKinematics(q, dq);
    UpdateDynamics(q, dq);
}

void RobotBasePinocchio::UpdateDynamics(const Eigen::VectorXd &q, const Eigen::VectorXd &dq)
{
    // Compute the joint space inertia matrix (M)
    pinocchio::crba(model_, data_, q);
    // Make sure the inertia matrix is symmetric
    data_.M.triangularView<Eigen::StrictlyLower>() = data_.M.transpose().triangularView<Eigen::StrictlyLower>();
    // Compute the generalized gravity forces (G)
    pinocchio::computeGeneralizedGravity(model_, data_, q);
    // Compute the non-linear effects (Coriolis, centrifugal, etc.) (H)
    pinocchio::nonLinearEffects(model_, data_, q, dq);


    // Optional: Compute centroidal dynamics if needed
    // Eigen::MatrixXd A = pinocchio::ccrba(model, data, q, qdot);
    // Eigen::MatrixXd dA = pinocchio::dccrba(model, data, q, qdot);
}

void RobotBasePinocchio::UpdateKinematics(const Eigen::VectorXd &q, const Eigen::VectorXd &dq)
{
    //TODO: tmperorary
    q_ = q;
    dq_ = dq;
    
    ComputeForwardKinematics(q,dq);
    for (auto &frame : GetAllFrameKinematics())
    {
        frame.get().Update(model_, data_);
    }
    for (auto &joint : GetAllJointKinematics())
    {
        joint.get().Update(q, dq);
    }
    com_.Update(model_, data_);
}

void RobotBasePinocchio::UpdateKinematicsZeroBase()
{
    q_.head(6).setZero();
    dq_.head(6).setZero();
    UpdateKinematics(q_, dq_);
}

void RobotBasePinocchio::UpdateAllZeroBase()
{
    UpdateKinematicsZeroBase();
    UpdateDynamics(q_, dq_);
}

VectorXd RobotBasePinocchio::ComputeCentroidalAngularMomentum()
{
    // Compute the centroidal momentum
    pinocchio::computeCentroidalMomentum(model_, data_);
    return data_.hg.angular();
}

VectorXd RobotBasePinocchio::ComputeCentroidalMomentum()
{
    // Compute the centroidal momentum
    pinocchio::computeCentroidalMomentum(model_, data_);
    VectorXd hg(6);
    hg.head(3) = data_.hg.angular();
    hg.tail(3) = data_.hg.linear();
    return hg;
}


void RobotBasePinocchio::ComputeForwardKinematics(const Eigen::VectorXd &q, const Eigen::VectorXd &dq)
{
    // Update the joint placements and spatial velocities according to the current joint configuration and velocity.
    pinocchio::forwardKinematics(model_, data_, q, dq, Eigen::VectorXd::Zero(nv()));
    // Updates data.oMf for all frames
    pinocchio::updateFramePlacements(model_, data_);
    
    //get COM position, velocity, and dJdq in acceleration
    pinocchio::centerOfMass(model_, data_, pinocchio::KinematicLevel::ACCELERATION,false);

    // Computes the full model Jacobian
    // This Jacobian does not correspond to any specific joint frame Jacobian. From this Jacobian, it is then possible to easily extract the Jacobian of a specific joint frame.
    // pinocchio::getJointJacobian for doing this specific extraction.
    pinocchio::computeJointJacobians(model_, data_);


}

double RobotBasePinocchio::GetLeftToeYaw() const
{
    Matrix3d Rleft = left_below_ankle_.RotationMatrix(data_);
    Eigen::EulerAnglesZYXd eul = eulerZYX(Rleft);
    return eul.alpha();
}

double RobotBasePinocchio::GetRightToeYaw() const
{
    Matrix3d Rright = right_below_ankle_.RotationMatrix(data_);
    Eigen::EulerAnglesZYXd eul = eulerZYX(Rright);
    return eul.alpha();
}

Eigen::Matrix3d RobotBasePinocchio::GetLeftToeRyaw() const
{
    double yaw = GetLeftToeYaw();
    return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

Eigen::Matrix3d RobotBasePinocchio::GetRightToeRyaw() const
{
    double yaw = GetRightToeYaw();
    return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}


Eigen::MatrixXd RobotBasePinocchio::ComputeCentroidalMomentumMatrix()
{
    pinocchio::computeCentroidalMap(model_, data_, q_);   
    return data_.Ag;
}

Eigen::MatrixXd RobotBasePinocchio::ComputeCentroidalMomentumMatrixTimeVariation()
{
    pinocchio::computeCentroidalMapTimeVariation(model_, data_, q_, dq_);
    return data_.dAg;
}


std::vector<std::reference_wrapper<RobotBasePinocchio::JointKinematics1D>> RobotBasePinocchio::GetAllJointKinematics()
{
   return {
       left_hip_yaw_, right_hip_yaw_};
}

void RobotBasePinocchio::FrameKinematics3D::Update(pinocchio::Model &model, pinocchio::Data &data)
{
    pinocchio::ReferenceFrame ref_frame = local_world_aligned ? pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED : pinocchio::ReferenceFrame::LOCAL;
    kinematics.position = data.oMf[frame_id].translation();
    kinematics.velocity = pinocchio::getFrameVelocity(model, data, frame_id, ref_frame).linear();
    MatrixXd Jtmp = Eigen::MatrixXd::Zero(6, model.nv);
    pinocchio::getFrameJacobian(model, data, frame_id, ref_frame, Jtmp);
    kinematics.jacobian = Jtmp.topRows(3);
    pinocchio::Motion dJdqtmp = pinocchio::getFrameClassicalAcceleration(model, data, frame_id, ref_frame);
    kinematics.dJdq = dJdqtmp.linear();
}

void RobotBasePinocchio::FrameKinematics3D::Init(int nv)
{
    kinematics.position = Eigen::Vector3d::Zero();
    kinematics.velocity = Eigen::Vector3d::Zero();
    kinematics.jacobian = Eigen::MatrixXd::Zero(3, nv);
    kinematics.dJdq = Eigen::Vector3d::Zero();
}


void RobotBasePinocchio::COMKinematics3D::Update(pinocchio::Model &model, pinocchio::Data &data)
{
    kinematics.position = data.com[0];
    kinematics.velocity = data.vcom[0];
    kinematics.jacobian = pinocchio::jacobianCenterOfMass(model, data, false);
    kinematics.dJdq = data.acom[0];
}

void RobotBasePinocchio::COMKinematics3D::Init(int nv)
{
    kinematics.position = Eigen::Vector3d::Zero();
    kinematics.velocity = Eigen::Vector3d::Zero();
    kinematics.jacobian = Eigen::MatrixXd::Zero(3, nv);
    kinematics.dJdq = Eigen::Vector3d::Zero();
}

void RobotBasePinocchio::JointKinematics1D::Update(const Eigen::VectorXd &q, const Eigen::VectorXd &dq)
{
    kinematics.position.setConstant(q(joint_id));
    kinematics.velocity.setConstant(dq(joint_id));
}

void RobotBasePinocchio::JointKinematics1D::Init(int nv)
{
    kinematics.position = Eigen::VectorXd::Zero(1);
    kinematics.velocity = Eigen::VectorXd::Zero(1);
    kinematics.jacobian = Eigen::MatrixXd::Zero(1, nv);
    kinematics.jacobian(0, joint_id) = 1;
    kinematics.dJdq = Eigen::VectorXd::Zero(1);
}
