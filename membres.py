import math
import numpy as np

# 1. Complétez le fichier membres.py joint à ce document avec les fonctions matrix transfo,
# calc mgd, jacobienne.
# 2. Interprétez et précisez l’objectif des fonctions __init__ et get_min_max.
# 3. Complétez les fonctions : gen traj et eval traj qui permettent de générer et évaluer la
# trajectoire suivie par le membre commandé.

class Membres:
    PI = math.pi
    
    def __init__(self, group, dh, robot):
        self.group = group
        self.dh = dh
        self.robot = robot

    def matrix_transfo(self):
        # get presents joints angles
        """
        joints = []
        for m in self.group:
            joints.append(m.present_position)
        joints.append(0)    # 0 for last joint (wrist)
        """

        matrix_transfo_map = []
        for i in range(len(self.dh)):
            theta = self.dh[i][2]
            alpha = self.dh[i][0]
            d = self.dh[i][1]
            r = self.dh[i][3]
            mx_tf = np.array([[math.cos(theta), -math.sin(theta), 0, d],
                      [math.cos(alpha) * math.sin(theta), math.cos(alpha) * math.cos(theta), -math.sin(alpha),
                       -r * math.sin(alpha)],
                      [math.sin(alpha) * math.sin(theta), math.sin(alpha) * math.cos(theta), math.cos(alpha),
                       r * math.cos(alpha)],
                      [0, 0, 0, 1]])
            matrix_transfo_map.append(mx_tf) 
        return matrix_transfo_map

    def calc_mgd(self):
        mgd = self.matrix_transfo()
        for i in range(len(mgd)):
            if i > 0:
                mgd[i] = np.dot(mgd[i-1],mgd[i])
        return mgd # for last joints position return mgd[-1], 
                   # the hand of Poppy is 0.23m longer along Y axis 

    def jacobienne(self):
        mgd = self.calc_mgd()
        j05 = np.arange(6*len(mgd)) 
        j05.shape = (6, len(mgd))
        for i in range(len(mgd)):
            z = mgd[i][:-1, 2:3] # 3rd col present z vector (rotation vector)
            p = mgd[-1][:-1, -1:] - mgd[i][:-1, -1:] # translation vector
            j05[:,i] = np.vstack((np.cross(z,p),z)) 
        """
        no use for the case that last coordiante is directly on the effector

        a = 0.23  # left forearms(respondable) length
        r05 = mgd[-1][:3,:3]
        mat_d = np.array([
            [0, a*r05[2,1], -a*r05[1,1]], # [2, 0] if the forearme is on X axis of last joint
            [-a*r05[2,1], 0, a*r05[0,1]],
            [a*r05[1,1], -a*r05[0,1], 0]
        ])
        """
        mat_d = np.zeros(9)
        mat_d.shap = (3,3)
        mat_zeros = np.zeros(9)
        mat_zeros.shap = (3,3)
        mat_iden = np.identity(3)
        mat_interm = np.hstack((np.vstack((mat_iden,mat_zeros)),np.vstack((mat_d,mat_iden))))

        J = mat_interm@j05
        mat_jacob = np.linalg.pinv(J)
        #calculuer la matrice Jp^-1
        return mat_jacob, J

    def get_min_max(self, robot_conf): # robot_config : param externe
        #:return angle_limit: Valeur angulaire min et max  pour chaque moteur du group
        angle_limite = []
        for key, value in robot_conf.items():
            if key == 'motors':
                for i in range(len(self.group)):
                    # angle_limite.append(self.group[i]) # debug
                    angle_limite.append(value[self.group[i]]['angle_limit'])
        return angle_limite

    def gen_traj(self, posHello, simulationTime, deltaT):
        #print(posPresent)
        # Polynomial order 5 for the acceleration could be zero when it starts & stops
        # matrixMGD -> position[x, y, z, row, pitch, yaw]
        #deltaT = 0.5
        posInit = np.mat('[-0.0; 0.0; 0.0; 0.0; 0.0]')
        a3 = 10*(posHello-posInit)/(simulationTime**3)
        a4 = -15*(posHello-posInit)/(simulationTime**4)
        a5 = 6*(posHello-posInit)/(simulationTime**5)
        newPos = posInit + a3*deltaT**3 + a4*deltaT**4 + a5*deltaT**5 # be added for each setp
        return newPos
        """
        Polynomial order 3
        posInit = np.mat('[-0.3; 0.3; 0.0; 0.0; 0.0]')
        a2 = (3/math.pow(simulationTime,2)) * (posHello-posInit)
        a3 = (2/math.pow(simulationTime,3)) * (posHello-posInit)
        newPos = posInit + a2*math.pow(deltaT,2) - a3*math.pow(deltaT,3)
        return newPos
        """

    def eval_traj(self):
        # use IK to see if the posHello has a solution //unnecessary in our case
        # if the angles of each joints are in their ranges (limit max min), traj ok
        # 1->traj_ok    0->traj_not_ok
        mgd = self.calc_mgd()
        mcd = self.jacobienne()[1]
        newPos = self.gen_traj()
        angle_limite = self.get_min_max()
        for i in range(len(newPos)):
            if newPos[i] > angle_limite[i][0] and newPos[i] < angle_limite[i][1]:
                print("traj ok")
            else:
                print("traj not ok")
        posOT = newPos@mgd[-1]
        velOT = newPos@mcd
        return posOT, velOT

        """
poppy_head_config = {
    'controllers': {
        'my_dxl_controller': {
            'port': '/dev/ttyACM0',  # Depends on your OS
            'sync_read': False,
            'attached_motors': ['head'],  # You can mix motorgroups or individual motors
        },
    },

    'motorgroups': {
        'head': ['head_y'],
    },

    'motors': {
        'head_y': {
            'id': 1,
            'type': 'AX-12',
            'orientation': 'INdirect',
            'offset': 20.0,
            'angle_limit': (-45, 6),
        },
    },
}
"""

