import numpy as np
import time
from pypot.creatures import PoppyHumanoid
from membres import Membres

PI = np.pi

poppy = PoppyHumanoid(simulator='vrep')
poppy.start_sync()

l_arm_group = poppy.l_arm
joints = []
for m in l_arm_group:
    joints.append(m.present_position)
#    print(m.present_position)
joints.append(0)    # 0 for last joint (wrist)
joints = [-1.5,1.5,0,-1.5,0]
lefArmDhParam = np.array([
    [0, 0, joints[0], 0],      
    [-PI/2, 0, joints[1], 0.185],
    [PI/2, 0, joints[2], -0.148],
    [-PI/2, -0.01, joints[3], 0],
    [-PI/2, 0, joints[4], 0.215]])

leftArm = Membres(l_arm_group, lefArmDhParam, poppy)
mgd = leftArm.calc_mgd()
#print(mgd[-1]) # last matrix : 0T5

posPresent = np.mat('[0; 0; 0.0; 0; 0.0]') 
posHello = np.mat('[-1.5; 1.5; 0.0; -1.5; 0.0]')
simulationTime = 20
deltaT= 0.5
timeseries = np.linspace(0, simulationTime, num = int(simulationTime//deltaT))

for t in timeseries:
    newPos = leftArm.gen_traj(posHello, simulationTime, t)
    poppy.l_shoulder_y.goto_position(newPos[0,0]*180/PI, 0.05, wait=False)
    poppy.l_shoulder_x.goto_position(newPos[1,0]*180/PI, 0.05, wait=False)
    poppy.l_arm_z.goto_position(newPos[2,0]*180/PI, 0.05, wait=False)
    poppy.l_elbow_y.goto_position(newPos[3,0]*180/PI, 0.05, wait=True)
    posPresent = newPos
    print(newPos)
    print("*******************************")
poppy.stop_simulation()