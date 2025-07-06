import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
import numpy as np
import os
from pinocchio import casadi as cpin
import casadi


pin.SE3.__repr__ = pin.SE3.__str__
np.set_printoptions(precision=6, linewidth=300, suppress=True, threshold=1e6)

## basic urdf test

urdfpath = os.path.join(os.path.dirname(__file__), "../g1_model_files/test.urdf")
full_model, full_collision_model, full_visual_model = pin.buildModelsFromUrdf(urdfpath)
robot = RobotWrapper(full_model, full_collision_model, full_visual_model) 
print("robot nq: ", robot.model.nq)
print("robot nv: ", robot.model.nv)

q = pin.neutral(robot.model)

v = np.zeros(robot.model.nv)    

q[0] = np.pi/2
v[0] = 10

pin.framesForwardKinematics(robot.model, robot.data, q)
pin.computeJointJacobians(robot.model, robot.data, q)
pin.computeForwardKinematicsDerivatives(robot.model, robot.data, q, v, np.zeros(robot.model.nv))
pin.computeJointJacobiansTimeVariation(robot.model, robot.data, q, v)


print(robot.data.oMi[0].translation)
print(robot.data.oMi[1].translation)
print(robot.data.oMi[2].translation)



#print joint names: universe joint0 joint1
for i in range(robot.model.njoints):
    print(robot.model.names[i])
    
    


joint1_id = robot.model.getJointId("joint1")
print("joint1_id: ", joint1_id)


dJ = pin.getJointJacobianTimeVariation(robot.model, robot.data, joint1_id, pin.LOCAL_WORLD_ALIGNED)

print(dJ)

pin.forwardKinematics(robot.model, robot.data, q, v, np.zeros(robot.model.nv))
a = pin.getClassicalAcceleration(robot.model, robot.data, joint1_id, pin.LOCAL_WORLD_ALIGNED)

print(a)

mass = pin.computeTotalMass(robot.model)

dJ = pin.computeCentroidalMapTimeVariation(robot.model, robot.data, q, v)/mass
print(dJ)

#dJ*dq
print(dJ.dot(v))

# 
# print(robot.data.acom[0]) #dJ*dq term


pin.centerOfMass(robot.model, robot.data) #assume second order forward kinematics is computed with zero accerleration
print(robot.data.com[0])

print(robot.data.vcom[0])
print(robot.data.acom[0])

Jcom = pin.jacobianCenterOfMass(robot.model, robot.data)
print(Jcom)

# #test cpin
# cmodel = cpin.Model(robot.model)
# cdata = cmodel.createData()


# cq = casadi.SX.sym("q", robot.model.nq, 1)
# cv = casadi.SX.sym("v", robot.model.nv, 1)

# #create zero acceleration
# ca = casadi.SX.zeros(robot.model.nv, 1)

# cpin.forwardKinematics(cmodel, cdata, cq)
# com = cpin.centerOfMass(cmodel, cdata, cq)

# print(com)
