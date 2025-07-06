// test pinocchio


#include "g1_model_leg.hpp"

#include <gtest/gtest.h>
#include <cmath>
using namespace Eigen;
using namespace std;
class testG1Model : public ::testing::Test
{
    public:


};

// TEST_F(testG1Model, testbasicURDF)
// {
//     std::string urdf_path = std::string(getenv("HOME")) + "/biped_simulation/src/g1_simulation/g1_model_files/test.urdf";

    

//     pinocchio::Model model;
//     pinocchio::urdf::buildModel(urdf_path, model);
//     pinocchio::SE3 placement = pinocchio::SE3::Identity();
//     placement.translation() << 0.9, 0, 0;

//     pinocchio::JointIndex joint0_id = model.getJointId("joint0");
//     pinocchio::JointIndex joint1_id = model.getJointId("joint1");
//     model.addFrame(pinocchio::Frame("frame1", joint0_id, 0, placement, pinocchio::OP_FRAME));
//     pinocchio::FrameIndex frame1_id = model.getFrameId("frame1");
//     //so joint1 and frame1 should have same results
//     pinocchio::Data data(model);



//     Vector2d q(M_PI/2, 0);
//     Vector2d qdot(10,0);

//     pinocchio::forwardKinematics(model, data, q, qdot,VectorXd::Zero(model.nv));
//     pinocchio::updateFramePlacements(model, data); 
//     pinocchio::computeJointJacobians(model, data);
//     pinocchio::computeForwardKinematicsDerivatives(model, data, q, qdot, Eigen::VectorXd::Zero(model.nv));
//     pinocchio::computeJointJacobiansTimeVariation(model, data, q, qdot);



//     double L = 0.9;
//     // Frame position
//     pinocchio::SE3 frame_pose = data.oMf[frame1_id];
    
//     pinocchio::SE3 joint_pose = data.oMi[joint1_id];
//     Vector3d expected_position(cos(q(0))*L, sin(q(0))*L, 0);

//     // std::cout << "frame_id: " << frame_id << std::endl;
//     std::cout << "pinocchio frame Position: " << frame_pose.translation().transpose() << std::endl;
//     std::cout << "pinocchio joint Position: " << joint_pose.translation().transpose() << std::endl;
//     std::cout << "expected position: " << expected_position.transpose() << std::endl;

//     std::cout << "pinocchio Rotation: " << frame_pose.rotation() << std::endl;


//     // Frame velocity
//     pinocchio::Motion frame_velocity = pinocchio::getFrameVelocity(model, data, frame1_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
//     std::cout << "pinocchio frame Linear Velocity: " << frame_velocity.linear().transpose() << std::endl;
//     std::cout << "pinocchio frame Angular Velocity: " << frame_velocity.angular().transpose() << std::endl;

//     pinocchio::Motion joint_velocity= pinocchio::getVelocity(model, data, joint1_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
//     std::cout << "pinocchio joint Linear Velocity: " << joint_velocity.linear().transpose() << std::endl;
//     std::cout << "pinocchio joint Angular Velocity: " << joint_velocity.angular().transpose() << std::endl;

//     pinocchio::Motion joint_velocity2= pinocchio::getVelocity(model, data, joint1_id, pinocchio::ReferenceFrame::LOCAL);
//     std::cout << "LOCAL pinocchio joint Linear Velocity: " << joint_velocity2.linear().transpose() << std::endl;
//     std::cout << "LOCAL pinocchio joint Angular Velocity: " << joint_velocity2.angular().transpose() << std::endl;

//     pinocchio::Motion joint_velocity3= pinocchio::getVelocity(model, data, joint1_id, pinocchio::ReferenceFrame::WORLD);
//     std::cout << "WORLD pinocchio joint Linear Velocity: " << joint_velocity3.linear().transpose() << std::endl;
//     std::cout << "WORLD pinocchio joint Angular Velocity: " << joint_velocity3.angular().transpose() << std::endl;



    

//     //compare local, local_world_aligned, world again as for velocity

//     MatrixXd jaocbian_frame1= Eigen::MatrixXd::Zero(6, model.nv);

//     pinocchio::getFrameJacobian(model,data,frame1_id,pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jaocbian_frame1);
//     std::cout << "ALIGN pinocchio frame Jacobian: \n" << jaocbian_frame1 << std::endl;


//     MatrixXd jaocbian_frame2= Eigen::MatrixXd::Zero(6, model.nv);
//     pinocchio::getFrameJacobian(model,data,frame1_id,pinocchio::ReferenceFrame::LOCAL, jaocbian_frame2);
//     std::cout << "LOCAL pinocchio frame Jacobian: \n" << jaocbian_frame2 << std::endl;

//     MatrixXd jaocbian_frame3= Eigen::MatrixXd::Zero(6, model.nv);
//     pinocchio::getFrameJacobian(model,data,frame1_id,pinocchio::ReferenceFrame::WORLD, jaocbian_frame3);
//     std::cout << "WORLD pinocchio frame Jacobian: \n" << jaocbian_frame3 << std::endl;

//     // MatrixXd jaocbian_joint1= Eigen::MatrixXd::Zero(6, model.nv);
//     // pinocchio::getJointJacobian(model,data,joint1_id,pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jaocbian_joint1);
//     // std::cout << "pinocchio joint Jacobian: \n" << jaocbian_joint1 << std::endl;



//     // MatrixXd dJacobian_frame1 = Eigen::MatrixXd::Zero(6, model.nv);
//     // pinocchio::getFrameJacobianTimeVariation(model, data, frame1_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJacobian_frame1);
//     // std::cout << "Time Derivative of Jacobian frame1: \n" << dJacobian_frame1 << std::endl;
 


//     // MatrixXd dJacobian_joint1 = Eigen::MatrixXd::Zero(6, model.nv);
//     // pinocchio::getJointJacobianTimeVariation(model, data, joint1_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJacobian_joint1);
//     // std::cout << "Time Derivative of Jacobian joint1: \n" << dJacobian_joint1 << std::endl;




//     // MatrixXd dJacobian_expected = Eigen::MatrixXd::Zero(6, model.nv);
//     // dJacobian_expected << -cos(q(0))*L*qdot(0), 0, 
//     // -sin(q(0))*L*qdot(0), 0,
//     // 0,0,
//     // 0,0,
//     // 0,0,
//     // 0,0;
//     // std::cout << "expected Time Derivative of Jacobian: \n" << dJacobian_expected << std::endl;





//     // pinocchio::Motion dJdq_frame1 = pinocchio::getFrameAcceleration(model, data, frame1_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
//     // Eigen::VectorXd dJdq(6); // Assuming 6D motion vector
//     // dJdq<< dJdq_frame1.linear(), dJdq_frame1.angular();
//     // std::cout << "Frame dJdq from drift:" << dJdq.transpose() << std::endl;

//     // pinocchio::Motion dJdq_joint1 = pinocchio::getAcceleration(model, data, joint1_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
//     // Eigen::VectorXd dJdq_joint(6); // Assuming 6D motion vector
//     // dJdq_joint<< dJdq_joint1.linear(), dJdq_joint1.angular();
//     // std::cout << "Joint dJdq from drift:" << dJdq_joint.transpose() << std::endl;


//     // VectorXd dJdq_expected(6);
//     // dJdq_expected << -cos(q(0))*L*qdot(0)*qdot(0),  -sin(q(0))*L*qdot(0)*qdot(0), 0,0,0,0;
//     // std::cout << "expected dJdq: " << dJdq_expected.transpose() << std::endl;


//     // pinocchio::Motion dJdq_joint2 = pinocchio::getClassicalAcceleration(model, data, joint1_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
//     // dJdq_joint<< dJdq_joint2.linear(), dJdq_joint2.angular();
//     // std::cout << "Joint dJdq from drift classical:" << dJdq_joint.transpose() << std::endl;


//     // pinocchio::Motion dJdq_frame2 = pinocchio::getFrameClassicalAcceleration(model, data, frame1_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
//     // dJdq<< dJdq_frame2.linear(), dJdq_frame2.angular();
//     // std::cout << "Frame dJdq from drift classical:" << dJdq.transpose() << std::endl;

//     // std::cout << " Classical Accleration is Correct!!!!" << std::endl;



// }

// TEST_F(testG1Model, testG1Lower)
// {
//     std::string urdf_path = std::string(getenv("HOME")) + "/biped_simulation/src/g1_simulation/g1_model_files/g1_29_withsensor_with6dofbase_lowerbody.urdf";
    

//     pinocchio::Model model;
//     pinocchio::urdf::buildModel(urdf_path, model);

//     pinocchio::SE3 placement = pinocchio::SE3::Identity();

//     placement.translation() << 0.12, 0.03, -0.03;
//     pinocchio::JointIndex joint_id = model.getJointId("left_ankle_roll_joint");
//     std::cout << "joint_id: " << joint_id << std::endl;
//     // Add the initialization of the model specific variables here
//     model.addFrame(pinocchio::Frame("left_foot_LF", model.getJointId("left_ankle_roll_joint"), 0, placement, pinocchio::OP_FRAME));
//     model.addFrame(pinocchio::Frame("right_foot_LF", model.getJointId("right_ankle_roll_joint"), 0, placement, pinocchio::OP_FRAME));

//     placement.translation() << 0.0, 0.0, -0.03;
//     model.addFrame(pinocchio::Frame("left_below_ankle", model.getJointId("left_ankle_roll_joint"), 0, placement, pinocchio::OP_FRAME));

//     pinocchio::JointIndex joint1_id = model.getJointId("left_ankle_roll_joint");
//     pinocchio::FrameIndex frame1_id = model.getFrameId("left_below_ankle");
//     //so joint1 and frame1 should have same results
//     pinocchio::Data data(model);

//     VectorXd q = VectorXd::Zero(model.nq);
//     VectorXd qdot = VectorXd::Zero(model.nv);

//     //left knee vel set nonzero
//     qdot(9) = 1;

//     //print joint names
//     for (int i = 0; i < model.njoints; i++)
//     {
//         std::cout << "joint name: " << model.names[i] << std::endl;
//     }




//     pinocchio::forwardKinematics(model, data, q, qdot,VectorXd::Zero(model.nv));
//     pinocchio::updateFramePlacements(model, data); 
//     pinocchio::computeJointJacobians(model, data);
//     pinocchio::computeForwardKinematicsDerivatives(model, data, q, qdot, Eigen::VectorXd::Zero(model.nv));
//     pinocchio::computeJointJacobiansTimeVariation(model, data, q, qdot);
//     pinocchio::centerOfMass(model, data); //get pcom, cvom, acom(=dJcom*dq)




//     // Frame position
//     pinocchio::SE3 frame_pose = data.oMf[frame1_id];
    
//     pinocchio::SE3 joint_pose = data.oMi[joint1_id];

//     // std::cout << "frame_id: " << frame_id << std::endl;
//     std::cout << "pinocchio frame Position: " << frame_pose.translation().transpose() << std::endl;
//     std::cout << "pinocchio joint Position: " << joint_pose.translation().transpose() << std::endl;

//     std::cout << "pinocchio Rotation: " << frame_pose.rotation() << std::endl;


//     // Frame velocity
//     pinocchio::Motion frame_velocity = pinocchio::getFrameVelocity(model, data, frame1_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
//     std::cout << "pinocchio frame Linear Velocity: " << frame_velocity.linear().transpose() << std::endl;
//     std::cout << "pinocchio frame Angular Velocity: " << frame_velocity.angular().transpose() << std::endl;

//     pinocchio::Motion joint_velocity= pinocchio::getVelocity(model, data, joint1_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
//     std::cout << "pinocchio joint Linear Velocity: " << joint_velocity.linear().transpose() << std::endl;
//     std::cout << "pinocchio joint Angular Velocity: " << joint_velocity.angular().transpose() << std::endl;


//     MatrixXd jaocbian_frame1= Eigen::MatrixXd::Zero(6, model.nv);

//     pinocchio::getFrameJacobian(model,data,frame1_id,pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jaocbian_frame1);
//     std::cout << "pinocchio frame Jacobian: \n" << jaocbian_frame1 << std::endl;

//     MatrixXd jaocbian_joint1= Eigen::MatrixXd::Zero(6, model.nv);
//     pinocchio::getJointJacobian(model,data,joint1_id,pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jaocbian_joint1);
//     std::cout << "pinocchio joint Jacobian: \n" << jaocbian_joint1 << std::endl;

//     //compare local, local_world_aligned, world again as for velocity
//     MatrixXd jaocbian_frame2= Eigen::MatrixXd::Zero(6, model.nv);
//     pinocchio::getFrameJacobian(model,data,frame1_id,pinocchio::ReferenceFrame::LOCAL, jaocbian_frame2);
//     std::cout << "LOCAL pinocchio frame Jacobian: \n" << jaocbian_frame2 << std::endl;

//     MatrixXd jaocbian_frame3= Eigen::MatrixXd::Zero(6, model.nv);
//     pinocchio::getFrameJacobian(model,data,frame1_id,pinocchio::ReferenceFrame::WORLD, jaocbian_frame3);
//     std::cout << "WORLD pinocchio frame Jacobian: \n" << jaocbian_frame3 << std::endl;



//     MatrixXd dJacobian_frame1 = Eigen::MatrixXd::Zero(6, model.nv);
//     pinocchio::getFrameJacobianTimeVariation(model, data, frame1_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJacobian_frame1);
//     std::cout << "Time Derivative of Jacobian frame1: \n" << dJacobian_frame1 << std::endl;
 


//     MatrixXd dJacobian_joint1 = Eigen::MatrixXd::Zero(6, model.nv);
//     pinocchio::getJointJacobianTimeVariation(model, data, joint1_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJacobian_joint1);
//     std::cout << "Time Derivative of Jacobian joint1: \n" << dJacobian_joint1 << std::endl;


//     pinocchio::Motion dJdq_frame2 = pinocchio::getFrameClassicalAcceleration(model, data, frame1_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
//     Eigen::VectorXd dJdq(6); 
//     dJdq<< dJdq_frame2.linear(), dJdq_frame2.angular();
//     std::cout << "Frame dJdq from drift classical:" << dJdq.transpose() << std::endl;

//     pinocchio::Motion dJdq_joint2 = pinocchio::getClassicalAcceleration(model, data, joint1_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
//     Eigen::VectorXd dJdq_joint(6); // Assuming 6D motion vector
//     dJdq_joint<< dJdq_joint2.linear(), dJdq_joint2.angular();
//     std::cout << "Joint dJdq from drift classical:" << dJdq_joint.transpose() << std::endl;

//     MatrixXd Jcom = pinocchio::jacobianCenterOfMass(model, data);
//     std::cout << "Jcom: \n" << Jcom << std::endl;

// }


// TEST_F(testG1Model, testEigen)
// {
//     int idx = 1;
//     MatrixXd test = MatrixXd::Zero(3, 3);
//     test(idx, idx) = 1;
//     std::cout << test << std::endl;



// }



// TEST_F(testG1Model, testZeroYaw){

//     std::string urdf_path = std::string(getenv("HOME")) + "/biped_simulation/src/g1_simulation/g1_model_files/g1_29_withsensor_with6dofbase_lowerbody.urdf";
    

//     pinocchio::Model model;
//     pinocchio::urdf::buildModel(urdf_path, model);

//     pinocchio::SE3 placement = pinocchio::SE3::Identity();

//     placement.translation() << 0.12, 0.03, -0.03;
//     pinocchio::JointIndex joint_id = model.getJointId("left_ankle_roll_joint");
//     std::cout << "joint_id: " << joint_id << std::endl;
//     // Add the initialization of the model specific variables here
//     model.addFrame(pinocchio::Frame("left_foot_LF", model.getJointId("left_ankle_roll_joint"), 0, placement, pinocchio::OP_FRAME));
//     model.addFrame(pinocchio::Frame("right_foot_LF", model.getJointId("right_ankle_roll_joint"), 0, placement, pinocchio::OP_FRAME));

//     placement.translation() << 0.0, 0.0, -0.03;
//     model.addFrame(pinocchio::Frame("left_below_ankle", model.getJointId("left_ankle_roll_joint"), 0, placement, pinocchio::OP_FRAME));

//     pinocchio::JointIndex joint1_id = model.getJointId("left_ankle_roll_joint");
//     pinocchio::FrameIndex frame1_id = model.getFrameId("left_below_ankle");


//     //so joint1 and frame1 should have same results
//     pinocchio::Data data(model);

//     VectorXd q = VectorXd::Zero(model.nq);
//     VectorXd qdot = VectorXd::Zero(model.nv);


    

//     //set nonzero yaw
//     double yaw = 1.0;
//     q(3) = yaw;
//     qdot.tail(model.nv-6).setConstant(0.5);
//     // qdot.head(3).setConstant(0.5);
//     qdot.segment(3,3) = Vector3d(0.5, 0.5, 0.5);

//     Eigen::Matrix3d R_yaw;
//     R_yaw << cos(yaw), -sin(yaw), 0,
//              sin(yaw),  cos(yaw), 0,
//              0,         0,        1;
//     Eigen::Matrix3d R_yawpitchroll;
//     double pitch = 0;
//     double roll = 0;
//     R_yawpitchroll << cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll),
//                       sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll),
//                       -sin(pitch),          cos(pitch)*sin(roll),                           cos(pitch)*cos(roll);         

//     // q(4) = pitch;
//     // q(5) = roll;                    
//     pinocchio::forwardKinematics(model, data, q, qdot,VectorXd::Zero(model.nv));
//     pinocchio::updateFramePlacements(model, data); 
//     pinocchio::computeJointJacobians(model, data);
//     pinocchio::computeForwardKinematicsDerivatives(model, data, q, qdot, Eigen::VectorXd::Zero(model.nv));
//     pinocchio::computeJointJacobiansTimeVariation(model, data, q, qdot);
//     pinocchio::centerOfMass(model, data); //get pcom, cvom, acom(=dJcom*dq)


    

//     MatrixXd jaocbian_frame1= Eigen::MatrixXd::Zero(6, model.nv);

//     pinocchio::getFrameJacobian(model,data,frame1_id,pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jaocbian_frame1);

//     cout << "yaw = " << yaw << endl;

//     cout << "J = \n" << jaocbian_frame1.topRows(3) << endl;
//     cout << "vel = " << jaocbian_frame1.topRows(3)*qdot << endl;

//     pinocchio::Motion dJdq_frame = pinocchio::getFrameClassicalAcceleration(model, data, frame1_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
//     cout << "dJdq = " << dJdq_frame.linear() << "," <<  dJdq_frame.angular()<< endl;


//     MatrixXd R = R_yawpitchroll;
//     MatrixXd jaocbian_frame_zero_yaw = R.transpose() * jaocbian_frame1.topRows(3);
//     cout << "R * J: \n" << jaocbian_frame_zero_yaw << endl;

//     cout << "R * vel = " << (jaocbian_frame_zero_yaw*qdot).transpose() << endl;

//     cout << "R * dJdq = " << (R.transpose() * dJdq_frame.linear()).transpose() << "," << (R.transpose() * dJdq_frame.angular()).transpose() << endl;





//     q(3) = 0;
//     q(4) = 0;
//     q(5) = 0;

//     cout << "yaw = 0" << endl;

//     qdot.segment(3,3) = R * qdot.segment(3,3);

//     pinocchio::forwardKinematics(model, data, q, qdot,VectorXd::Zero(model.nv));
//     pinocchio::updateFramePlacements(model, data); 
//     pinocchio::computeJointJacobians(model, data);
//     pinocchio::computeForwardKinematicsDerivatives(model, data, q, qdot, Eigen::VectorXd::Zero(model.nv));
//     pinocchio::computeJointJacobiansTimeVariation(model, data, q, qdot);
//     pinocchio::centerOfMass(model, data); //get pcom, cvom, acom(=dJcom*dq)




//     pinocchio::getFrameJacobian(model,data,frame1_id,pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jaocbian_frame1);
//     dJdq_frame = pinocchio::getFrameClassicalAcceleration(model, data, frame1_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);

//     cout << "J: \n" << jaocbian_frame1.topRows(3) << endl;

//     cout << "vel = " << ((jaocbian_frame1.topRows(3))*qdot).transpose() << endl;

//     cout << "dJdq = " << (dJdq_frame.linear()).transpose() << "," <<  (dJdq_frame.angular()).transpose() << endl;


// }






TEST_F(testG1Model, testCentroidalMomentumFrame){

    std::string urdf_path = std::string(getenv("HOME")) + "/biped_simulation/src/g1_simulation/g1_model_files/g1_29_withsensor_with6dofbase_lowerbody.urdf";
    

    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);

    //so joint1 and frame1 should have same results
    pinocchio::Data data(model);

    VectorXd q = VectorXd::Zero(model.nq);
    VectorXd qdot = VectorXd::Zero(model.nv);

    q(3) = 0;
    qdot(6) = 1;
    qdot(9) = 1;
    qdot(12) = 1;
    qdot(15) = 1;

    pinocchio::forwardKinematics(model, data, q, qdot, Eigen::VectorXd::Zero(model.nv));
    pinocchio::computeCentroidalMomentum(model, data,q, qdot);
    cout << "yaw  = "  << q(3) << endl;
    VectorXd hcom_zeroyaw = data.hg.angular();
    cout << "hcom ang: " <<   data.hg.angular().transpose() << endl;


    q(3) = 3.1415926/2;
    qdot(6) = 1;
    qdot(9) = 1;
    qdot(12) = 1;
    qdot(15) = 1;

    pinocchio::forwardKinematics(model, data, q, qdot, Eigen::VectorXd::Zero(model.nv));
    pinocchio::computeCentroidalMomentum(model, data,q, qdot);
    cout << "yaw  = "  << q(3) << endl;
    cout << "hcom ang: " <<   data.hg.angular().transpose()  << endl;

    MatrixXd mat = AngleAxis<double>(-q(3), Vector3d(0, 0, 1)).toRotationMatrix();
    VectorXd hang = data.hg.angular();
    hang = mat * hang;
    cout << "R(-yaw)*hcom_ang" << endl;
    cout << "hcom ang: " << hang.transpose() << endl;

    //check if the hcom is correct
    ASSERT_TRUE(hcom_zeroyaw.isApprox(hang, 1e-6));
}