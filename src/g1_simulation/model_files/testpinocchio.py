import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
import numpy as np
import os
from pinocchio import casadi as cpin
import casadi


pin.SE3.__repr__ = pin.SE3.__str__
np.set_printoptions(precision=6, linewidth=300, suppress=True, threshold=1e6)


# urdf_path = os.path.join(os.path.dirname(__file__), "g1_29_withsensor_with6dofbase_lowerbody.urdf")
# model = pin.buildModelFromUrdf(urdf_path)
# data = model.createData()
# # print("robot nq: ", robot.model.nq)
# # print("robot nv: ", robot.model.nv)

# # q = pin.neutral(robot.model)

# # v = np.zeros(robot.model.nv)    



# Load the urdf file
urdf_path = os.path.join(os.path.dirname(__file__), 'g1_29_withsensor_with6dofbase_lowerbody.urdf')
model = pin.buildModelFromUrdf(urdf_path)
joint_id = model.getJointId("right_ankle_roll_joint")

frame_id = model.addFrame(pin.Frame("right_ankle_roll", joint_id, pin.SE3.Identity(), pin.OP_FRAME))

data = model.createData()
# Set the configuration to the initial position
q = pin.neutral(model)
q[0] = 0
q[3] = 0
q[4] = 1
q[5] = 0

q[6] =1
v = np.zeros(model.nv) 
v[0] = 0
v[3] = 1
v[4] = 2
v[5] = 3


pin.forwardKinematics(model, data, q,v, np.zeros(model.nv))
pin.updateFramePlacements(model, data)
pin.computeJointJacobians(model, data)
pin.computeJointJacobiansTimeVariation(model, data,q,v)
pin.computeForwardKinematicsDerivatives(model, data, q, v, np.zeros(model.nv))

#print all joint names
for i in range(model.njoints):
    print(model.names[i])


# pin.computeGeneralizedGravity(model, data, q, v)


# print("joint_id: ", joint_id)
# joint_pose = data.oMi[joint_id]
# print(joint_pose.translation)
# print(joint_pose.rotation)


# print('local vel', pin.getVelocity(model, data, joint_id, pin.LOCAL).linear, pin.getVelocity(model, data, joint_id, pin.LOCAL).angular)
# print('world vel', pin.getVelocity(model, data, joint_id, pin.WORLD).linear, pin.getVelocity(model, data, joint_id, pin.WORLD).angular)
# print('align vel', pin.getVelocity(model, data, joint_id, pin.LOCAL_WORLD_ALIGNED).linear, pin.getVelocity(model, data, joint_id, pin.LOCAL_WORLD_ALIGNED).angular)

# print('local Jacobian', pin.getJointJacobian(model, data, joint_id, pin.LOCAL))
# print('world Jacobian', pin.getJointJacobian(model, data, joint_id, pin.WORLD))
# print('align Jacobian', pin.getJointJacobian(model, data, joint_id, pin.LOCAL_WORLD_ALIGNED))


frame_pose = data.oMf[frame_id]
print(frame_pose.translation)
print(frame_pose.rotation)

print('local vel', pin.getFrameVelocity(model, data, frame_id, pin.LOCAL).linear, pin.getFrameVelocity(model, data, frame_id, pin.LOCAL).angular)
print('world vel', pin.getFrameVelocity(model, data, frame_id, pin.WORLD).linear, pin.getFrameVelocity(model, data, frame_id, pin.WORLD).angular)
print('align vel', pin.getFrameVelocity(model, data, frame_id, pin.LOCAL_WORLD_ALIGNED).linear, pin.getFrameVelocity(model, data, frame_id, pin.LOCAL_WORLD_ALIGNED).angular)

print('local Jacobian', pin.getFrameJacobian(model, data, frame_id, pin.LOCAL))
print('world Jacobian', pin.getFrameJacobian(model, data, frame_id, pin.WORLD))
print('align Jacobian', pin.getFrameJacobian(model, data, frame_id, pin.LOCAL_WORLD_ALIGNED))

R = frame_pose.rotation
ypr = pin.rpy.matrixToRpy(R)
J = pin.getFrameJacobian(model, data, frame_id, pin.LOCAL)
Jw = J[3:6, :]
J_ypr = pin.rpy.Jlog6(ypr) @ Jw



# # Compute the Jacobian at the initial position
# J = pin.computeJointJacobians(model, data, q0)
# print(J)
# # Compute the mass matrix at the initial position
# M = pin.crba(model, data, q0)
# print(M)


