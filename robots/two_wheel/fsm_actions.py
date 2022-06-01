import numpy as np
from task_planner.splines import lin_interp, s_curve_interp
from virtual_modeling.data_type import show_plot
from virtual_modeling.data_saver import DataSingleton
from task_planner.splines import TrapInterpWithVel
from task_planner.fsm_action import BaseAction


def cross(v1, v2):
    return v1[0]*v2[1] - v2[0]*v1[1]


class reg:

    def __init__(self):
        self.prev = None
        self.integ = np.array([0., 0.])
        # self.k1 = np.array([1, 4])
        self.k1 = np.array([4, 32])
        self.k2 = np.array([0.0, 0.0])
        self.k3 = np.array([2, 8])
        self.rw = 0.15
        self.dt = 1/240
        self.data = DataSingleton.get().get_ds("1")
        self.data2 = DataSingleton.get().get_ds("2")
        self.data3 = DataSingleton.get().get_ds("3")

    def reg(self, ppos, cpos, cteta):
        dp = ppos - cpos
        ppos_rot = np.array([
            (np.cos(cteta)*dp[0] - np.sin(cteta)*dp[1]),
            (np.sin(cteta)*dp[0] + np.cos(cteta)*dp[1]),
        ])
        self.data3.append((ppos[0], ppos[1], cpos[0], cpos[1]))
        self.data.append(ppos_rot.copy())
        tmp = 1
        if ppos_rot[0] < 0:
            tmp = -1
            # ppos_rot[0] = -ppos_rot[0]
            # ppos_rot[1] = -ppos_rot[1]
        v = np.array([
            np.linalg.norm(ppos_rot)*tmp,
            np.arctan2(ppos_rot[1], ppos_rot[0])
        ])
        if self.prev is not None:
            while abs(v[1] + 2*np.pi - self.prev[1]) < abs(v[1] - self.prev[1]):
                v[1] = v[1] + 2*np.pi
            while abs(v[1] - 2*np.pi - self.prev[1]) < abs(v[1] - self.prev[1]):
                v[1] = v[1] - 2*np.pi
        self.data2.append(v)
        self.integ = self.integ + v
        v2 = np.array([0., 0.])
        if self.prev is not None:
            v2 = v*self.k1 + (v-self.prev)/self.dt*self.k2 + self.integ*self.dt*self.k3
            print(ppos_rot, v, v2, cpos, ppos, ')))))))))))')
        self.prev = v
        if v[0] < 0.001:
            return np.array([0, 0])
        return np.array([v2[0] - self.rw*v2[1], v2[0] + self.rw*v2[1]])


class reg_plan:

    def __init__(self):
        self.prev = None
        self.max_shift = 0.05
        self.integ = np.array([0., 0.])
        # self.k1 = np.array([1, 4])
        self.k1 = np.array([4, 32])
        self.k2 = np.array([0.0, 0.0])
        self.k3 = np.array([2, 8])
        self.rw = 0.15
        self.dt = 1/240
        self.data1 = DataSingleton.get().get_ds("1")
        self.data2 = DataSingleton.get().get_ds("2")
        self.data3 = DataSingleton.get().get_ds("3")
        self.data4 = DataSingleton.get().get_ds("4")
        self.data5 = DataSingleton.get().get_ds("5")

    def reg(self, ppos_l, cpos, cteta):
        if len(ppos_l) == 0:
            return np.array((0, 0))
        ppos = ppos_l[0]
        dp = ppos - cpos
        ppos_rot = np.array([
            (np.cos(cteta)*dp[0] - np.sin(cteta)*dp[1]),
            (np.sin(cteta)*dp[0] + np.cos(cteta)*dp[1]),
        ])
        self.data1.append((ppos[0], ppos[1], cpos[0], cpos[1]))
        # print(ppos_l, ppos_rot)
        self.data2.append(ppos_rot)
        ppos_rot *= 0.05
        if np.linalg.norm(ppos_rot) > self.max_shift:
            ppos_rot *= self.max_shift / np.linalg.norm(ppos_rot)


        if len(ppos_l) > 1:
            vec = ppos_l[1]-ppos_l[0] + ppos_rot
        else:
            vec = np.array([0, 0])

        self.data3.append(vec)
        tmp = 1
        if vec[0] < 0:
            tmp = 1
            # vec = -vec
        v = np.array([
            np.linalg.norm(vec),#*np.cos(np.arctan2(vec[1], vec[0])),
            np.arctan2(vec[1], vec[0])
        ])
        # print(v, '------', np.array([v[0]/self.dt + self.rw*v[1], v[0]/self.dt - self.rw*v[1]]))
        self.data4.append(v)
        self.data5.append(np.array([v[0]/self.dt + self.rw*v[1], v[0]/self.dt - self.rw*v[1]]))

        if abs(v[0]) < 1e-3:
            return np.array((0, 0))

        v[0] *= np.cos(v[1])

        # return np.array((0, 0))
        return np.array([v[0]/self.dt - self.rw*v[1], v[0]/self.dt + self.rw*v[1]])


class CircleMove(BaseAction):

    def __init__(self, center, start, start_v, end, end_v, v):
        super().__init__()
        self.center = center
        self.start = start
        self.start_v = start_v
        self.start_c_teta = np.arctan2((self.start.pos-self.center)[1], (self.start.pos-self.center)[0])
        self.end = end
        self.end_v = end_v
        self.end_c_teta = np.arctan2((self.end.pos-self.center)[1], (self.end.pos-self.center)[0])
        self.r = np.linalg.norm(self.start.pos - self.center)
        self.l = (self.end.teta - self.start.teta) * self.r
        self.v = v
        self.rw = 0.15
        self.rc = 0.1
        self.k1 = 240
        self.mdv = 0.08*self.rc/self.k1
        self.trap_interp = TrapInterpWithVel(self.start_v/self.l, self.v/self.l, self.end_v/self.l, self.mdv)
        self.reg = reg_plan()
        self.iterations = self.trap_interp.t1 + self.trap_interp.t2 + self.trap_interp.t3
        print("it", self.iterations, self.trap_interp.t1, self.trap_interp.t2, self.trap_interp.t3)

        self.positions = list()
        for i in range(int(self.iterations)):
            teta = self.start_c_teta + self.trap_interp.get(i)[1] * (self.end_c_teta - self.start_c_teta)
            self.positions.append(self.center + self.r*np.array([np.cos(teta), np.sin(teta)]))
        self.positions = np.array(self.positions)

        self.c1 = 0.001
        self.c2 = 0.001

    def control(self, pos, t):
        v_orig, s = self.trap_interp.get(t-self.start_time+1)
        alpha = self.start_c_teta + s * (self.end_c_teta - self.start_c_teta)
        ppos = self.center + self.r * np.array([np.cos(alpha), np.sin(alpha)])
        pteta = alpha - np.pi/2
        v = v_orig * self.l
        if abs(v) < 1e-7:
            return np.array([0, 0])

        cpos = pos[:2]
        cteta = pos[2]

        z1 = self.r - np.linalg.norm(cpos-self.center)
        z2 = v*np.sin(cteta-pteta)

        w = v*np.cos(cteta-pteta)/(self.r - z1) - (self.c1*z1 + self.c2*z2)/np.cos(cteta-pteta)/v

        DataSingleton.get().get_ds("1").append((z1, z2, w, np.cos(cteta-pteta)))
        DataSingleton.get().get_ds("2").append((ppos[0], ppos[1], cpos[0], cpos[1]))

        return np.array([v + w/2, v - w/2])/self.rc*self.k1


class CircleMoveGenerator:

    def __init__(self, center, curr, next, v):
        self.curr = curr
        self.center = center
        self.next = next
        self.v = v

    def generate_controller(self, start_time):
        res = CircleMove(self.center, self.curr.jp, self.curr.v, self.next.jp, self.next.v, self.v)
        res.start_time = start_time
        res.next_state = self.next
        return res


class LinMove(BaseAction):

    def __init__(self, start, start_v, end, end_v, v):
        super().__init__()
        self.start = start
        self.start_v = start_v
        self.end = end
        self.end_v = end_v
        self.l = np.linalg.norm(start.pos - end.pos)
        self.direct = (end.pos - start.pos)/self.l
        self.teta = np.arctan2(self.direct[1], self.direct[0])
        self.ort = np.array([self.direct[1], -self.direct[0]])
        self.v = v
        self.rc = 0.1
        self.k1 = 240
        self.mdv = 0.08*self.rc/self.k1
        self.trap_interp = TrapInterpWithVel(self.start_v/self.l, self.v/self.l, self.end_v/self.l, self.mdv)
        self.reg = reg_plan()
        self.iterations = self.trap_interp.t1 + self.trap_interp.t2 + self.trap_interp.t3
        print("it", self.iterations, self.trap_interp.t1, self.trap_interp.t2, self.trap_interp.t3)

        self.positions = list()
        for i in range(int(self.iterations)):
            self.positions.append(self.start.pos + self.trap_interp.get(i)[1]*(self.end.pos - self.start.pos))
        self.positions = np.array(self.positions)

        self.c1 = 0.001
        self.c2 = 0.001

    def control(self, pos, t):
        v_orig, s = self.trap_interp.get(t-self.start_time)
        ppos = self.start.pos + s * (self.end.pos - self.start.pos)
        v = v_orig * self.l
        if abs(v) < 1e-7:
            return np.array([0, 0])

        cpos = pos[:2]
        teta = pos[2]

        z1 = np.sum(self.ort*(cpos - ppos))
        z2 = v*np.sin(teta-self.teta)

        w = -(self.c1*z1 + self.c2*z2)/np.cos(teta-self.teta)/self.v

        DataSingleton.get().get_ds("1").append((z1, z2, w, np.cos(teta-self.teta)))
        DataSingleton.get().get_ds("2").append((ppos[0], ppos[1], cpos[0], cpos[1]))

        return np.array([v + w/2, v - w/2])/self.rc*self.k1


class LinMoveGenerator:

    def __init__(self, curr, next, v):
        self.curr = curr
        self.next = next
        self.v = v

    def generate_controller(self, start_time):
        res = LinMove(self.curr.jp, self.curr.v, self.next.jp, self.next.v, self.v)
        res.start_time = start_time
        res.next_state = self.next
        return res
