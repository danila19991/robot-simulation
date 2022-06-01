from task_planner.splines import lin_interp, s_curve_interp
from virtual_modeling.data_type import to_homogeneous
from task_planner.fsm_state import BaseFsmState
import numpy as np


DEFAULT_ORIENT = np.array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])


class LinMoveWorld:

    def __init__(self):
        self.start_pos = None
        self.end_pos = None
        self.orient = None
        self.start_time = None
        self.iterations = None
        self.next_state = None

    def is_finished(self, t):
        return t > self.start_time + self.iterations

    def control(self, t):
        p = lin_interp(self.start_pos, self.end_pos, t-self.start_time, self.iterations)
        return to_homogeneous(p, self.orient)

    @classmethod
    def is_fsm_state(cls):
        return False


class LinMoveWorldGenerator:

    def __init__(self, iterations, old_state, next_state):
        self.iterations = iterations
        self.next_state = next_state
        self.old_state = old_state

    def generate_controller(self, start_time):
        res = LinMoveWorld()
        res.start_pos = self.old_state.pos
        res.end_pos = self.next_state.pos
        res.orient = self.old_state.orient
        res.start_time = start_time
        res.iterations = self.iterations
        res.next_state = self.next_state
        return res


class SCurveMoveWorld:

    def __init__(self):
        self.start_pos = None
        self.end_pos = None
        self.orient = None
        self.interaction_time = None
        self.start_time = None
        self.end_time = None
        self.ctx = None

    def control(self, t):
        p = s_curve_interp(self.start_pos, self.end_pos, t-self.start_time, self.end_time-self.start_time,
                           self.interaction_time)
        return to_homogeneous(p, self.orient)

    @classmethod
    def is_fsm_state(cls):
        return False


class SCurveMoveWorldGenerator:

    def __init__(self, iterations, t3, old_state, next_state):
        self.iterations = iterations
        self.next_state = next_state
        self.old_state = old_state
        self.t3 = t3
        self.v = np.linalg.norm(old_state.pos - next_state.pos)*2/(iterations + t3)

    def generate_controller(self, start_time):
        res = SCurveMoveWorld()
        res.start_pos = self.old_state.pos
        res.end_pos = self.next_state.pos
        res.orient = self.old_state.orient
        res.start_time = start_time
        res.iterations = self.iterations
        res.next_state = self.next_state
        return res


class ManipulatorState(BaseFsmState):

    def __init__(self, pos, orient=None):
        self.pos = pos
        self.orient = orient
        if self.orient is None:
            self.orient = DEFAULT_ORIENT
        self.next = dict()
        self.default = None

    def get_homogeneous(self):
        return to_homogeneous(self.pos, self.orient)

