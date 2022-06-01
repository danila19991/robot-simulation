import numpy as np
from task_planner.fsm_state import BaseFsmState


class JointPosition:

    def __init__(self, x, y, teta):
        self.x = x
        self.y = y
        self.teta = teta
        self.pos = np.array([self.x, self.y])
        self.a = np.array([np.cos(self.teta), np.sin(self.teta)])
        self.b = np.array([np.sin(self.teta), -np.cos(self.teta)])


class TwoWheelState(BaseFsmState):

    def __init__(self, x, y, teta, v):
        super(TwoWheelState, self).__init__()
        self.jp = JointPosition(x, y, teta)
        self.v = v
