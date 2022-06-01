import math
import pybullet as p
import modern_robotics as mr
import numpy as np
from virtual_modeling.data_type import to_homogeneous
from scipy.spatial.transform import Rotation as R


def lin_interp(q0, qd, t, T):
    s = t/T
    return q0 + s*(qd-q0)


def cubic_interp(q0, qd, t, T):
    a2 = 3/T**2
    a3 = -2/T**3
    s = a2*t**2 + a3*t**3
    return q0 + s*(qd-q0)


def trap_interp(q0, qd, t, T):
    a = 2*4/T**2
    v = (a*T - math.sqrt(a)*math.sqrt(a*T**2-4))/2
    ta = v/a
    if 0 <= t <= ta:
        s = (a*t**2)/2
    elif ta < t <= (T - ta):
        s = v*t - v**2/(2*a)
    else:
        s = (2*a*v*T - 2*v**2 - a**2*(t-T)**2)/(2*a)
    return q0 + s*(qd-q0)


class TrapInterpWithVel:

    def __init__(self, v1, v2, v3, mdw):
        self.mdv = mdw
        self.t1 = abs(v2-v1)/self.mdv
        self.t3 = abs(v3-v2)/self.mdv
        self.t2 = int((1 - (v2+v1)/2*self.t1 - (v2+v3)/2*self.t3)/v2)
        if self.t2 < 0:
            raise ValueError
        self.v1 = v1
        self.v2 = v2
        self.v3 = v3
        if self.t1 > 0:
            self.a1 = (self.v2-self.v1)/self.t1
        if self.t3 > 0:
            self.a3 = (self.v3-self.v2)/self.t3

    def get(self, ct):
        if ct < self.t1:
            cct = ct
            v = self.v1 + self.a1*cct
            s = self.v1*cct + (self.a1*cct**2)/2
        elif ct < self.t1 + self.t2:
            cct = ct - self.t1
            v = self.v2
            s = (self.v1+self.v2)*self.t1/2 + self.v2*(cct-1)
        elif ct < self.t1 + self.t2 + self.t3:
            cct = ct - self.t1 - self.t2
            v = self.v2 + self.a3 * cct
            s = (self.v1+self.v2)*self.t1/2 + self.v2*self.t2 + self.v2*cct + (self.a3*cct**2)/2
        else:
            cct = ct - self.t1 - self.t2 - self.t3
            v = self.v3
            s = (self.v1+self.v2)*self.t1/2 + self.v2*self.t2 + (self.v2+self.v3)*self.t3/2 + self.v3*cct
        # print(v, s)
        return v, s


def s_curve_interp(q0, qd, t, T, t_3, t_2=None):
    v = 2/(T + t_3)
    if t_2 is None:
        t_2 = (T - t_3)*0.3 # choose
    t_1 = (T - t_3 - 2*t_2)/4
    a = v/(t_1+t_2)
    J = a/t_1
    if 0 <= t <= t_1:
        s = (J*t**3)/6
    elif t_1 < t <= (t_2 + t_1):
        t_d = t - t_1
        s = (a*t_1**2)/6 + a*t_1*t_d/2 + (a*t_d**2)/2
    elif (t_2 + t_1) < t <= (t_2 + 2*t_1):
        t_d = t - (t_2 + t_1)
        s = (a*t_1**2)/6 + a*t_1*t_2/2 + (a*t_2**2)/2 + a*t_1*t_d/2 + a*t_2*t_d + (a*t_d**2)/2 - (J*t_d**3)/6
    elif (t_2 + 2*t_1) < t <= (t_3 + t_2 + 2*t_1):
        t_d = t - (t_2 + 2*t_1)
        s = a*t_1*t_2/2 + v*(t_2+t_1)/2 + (a*t_1**2)/2 + v*t_d
    elif (t_3 + t_2 + 2*t_1) < t <= (t_3 + t_2 + 3*t_1):
        t_d = t - (t_3 + t_2 + 2*t_1)
        s = a*t_1*t_2/2 + v*(t_2+t_1)/2 + (a*t_1**2)/2 + v*t_3 + v*t_d - (J*t_d**3)/6
    elif (t_3 + t_2 + 3*t_1) < t <= (t_3 + 2*t_2 + 3*t_1):
        t_d = t - (t_3 + t_2 + 3*t_1)
        s = a*t_1*t_2/2 + v*(t_2+t_1)/2 + (a*t_1**2)/2 + v*t_3 + v*t_1 - (a*t_1**2)/6 + v*t_d - a*t_1*t_d/2 - (a*t_d**2)/2
    else:
        t_d = t - (t_3 + 2*t_2 + 3*t_1)
        s = v*t_3 + v*t_2 + 2*v*t_1 - (a*t_1**2)/6 + a*t_1*t_d/2 - (a*t_d**2)/2 + (J*t_d**3)/6
    return q0 + s*(qd-q0)


def p2p_s_curve_decoupled(X0, Xd, t, T):
    t_2 = T/8 # choose
    t_3 = T/2 # choose
    t_1 = (T - t_3 - 2*t_2)/4
    v = 2/(T + t_3)
    a = v/(t_1+t_2)
    J = a/t_1
    if 0 <= t <= t_1:
        s = (J*t**3)/6
    elif t_1 < t <= (t_2 + t_1):
        t_d = t - t_1
        s = (a*t_1**2)/6 + a*t_1*t_d/2 + (a*t_d**2)/2
    elif (t_2 + t_1) < t <= (t_2 + 2*t_1):
        t_d = t - (t_2 + t_1)
        s = (a*t_1**2)/6 + a*t_1*t_2/2 + (a*t_2**2)/2 + a*t_1*t_d/2 + a*t_2*t_d + (a*t_d**2)/2 - (J*t_d**3)/6
    elif (t_2 + 2*t_1) < t <= (t_3 + t_2 + 2*t_1):
        t_d = t - (t_2 + 2*t_1)
        s = a*t_1*t_2/2 + v*(t_2+t_1)/2 + (a*t_1**2)/2 + v*t_d
    elif (t_3 + t_2 + 2*t_1) < t <= (t_3 + t_2 + 3*t_1):
        t_d = t - (t_3 + t_2 + 2*t_1)
        s = a*t_1*t_2/2 + v*(t_2+t_1)/2 + (a*t_1**2)/2 + v*t_3 + v*t_d - (J*t_d**3)/6
    elif (t_3 + t_2 + 3*t_1) < t <= (t_3 + 2*t_2 + 3*t_1):
        t_d = t - (t_3 + t_2 + 3*t_1)
        s = a*t_1*t_2/2 + v*(t_2+t_1)/2 + (a*t_1**2)/2 + v*t_3 + v*t_1 - (a*t_1**2)/6 + v*t_d - a*t_1*t_d/2 - (a*t_d**2)/2
    else:
        t_d = t - (t_3 + 2*t_2 + 3*t_1)
        s = v*t_3 + v*t_2 + 2*v*t_1 - (a*t_1**2)/6 + a*t_1*t_d/2 - (a*t_d**2)/2 + (J*t_d**3)/6
    # X = X0 @ mr.MatrixExp6(mr.MatrixLog6(np.linalg.inv(X0)@Xd)*s)
    # print(s)
    pos0 = X0[0:3,3]
    pos1 = Xd[0:3,3]
    pos = pos0 + (pos1-pos0)*s
    print(pos0, pos1, pos)
    mat_rot0 = X0[0:3,0:3]
    # mat_rot1 = Xd[0:3,0:3]
    # mat_rot = mat_rot0 @ mr.MatrixExp3(mr.MatrixLog3(mat_rot0.T@mat_rot1)*s)
    # eul = rot2eul(mat_rot)

    # quat = p.getQuaternionFromEuler(eul)
    print(pos)
    return to_homogeneous(pos, mat_rot0) # todo restore rotation



def rot2eul(R):
    beta = -np.arcsin(R[2,0])
    alpha = np.arctan2(R[2,1]/np.cos(beta),R[2,2]/np.cos(beta))
    gamma = np.arctan2(R[1,0]/np.cos(beta),R[0,0]/np.cos(beta))
    return np.array((alpha, beta, gamma))


# todo update
def p2p_lin_screw(rob, X0, Xd, t, T):
    a2 = 3/T**2
    a3 = -2/T**3
    s = a2*t**2 + a3*t**3
    X = X0 @ mr.MatrixExp6(mr.MatrixLog6(np.linalg.inv(X0)@Xd)*s)

    pos = X[0:3,3]
    mat_rot = X[0:3,0:3]
    eul = rot2eul(mat_rot)

    quat = p.getQuaternionFromEuler(eul)
    return rob.get_world_from_joint(pos, quat)


# todo update
def p2p_lin_decoupled(rob, X0, Xd, t, T):
    a2 = 3/T**2
    a3 = -2/T**3
    s = a2*t**2 + a3*t**3
    # X = X0 @ mr.MatrixExp6(mr.MatrixLog6(np.linalg.inv(X0)@Xd)*s)

    pos0 = X0[0:3,3]
    pos1 = Xd[0:3,3]
    pos = pos0 + (pos1-pos0)*s
    mat_rot0 = X0[0:3,0:3]
    mat_rot1 = Xd[0:3,0:3]
    mat_rot = mat_rot0 @ mr.MatrixExp3(mr.MatrixLog3(mat_rot0.T@mat_rot1)*s)
    eul = rot2eul(mat_rot)

    quat = p.getQuaternionFromEuler(eul)
    return rob.get_world_from_joint(pos, quat)
