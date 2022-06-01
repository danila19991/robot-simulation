import pybullet as pb
import numpy as np
from scipy.spatial.transform import Rotation as R
from .data_type import to_homogeneous
from task_planner.fsm_state import BaseFsmState


class BaseRobot:

    def __init__(self, ctx):
        self.file_path = ''
        self.start_base_pos = [0, 0, 0]
        self.start_base_orientation = pb.getQuaternionFromEuler([0, 0, 0])
        self.model_id = None
        self.markers = dict()
        self.position_story_detected = list()
        self.position_story_actual = list()
        self.position_story_joint = list()
        self.instrument_id = None
        self.ctx = ctx
        self.state = BaseFsmState()
        self.event_list = list()
        self.resolvers = list()

    def get_start_base_pos(self):
        return self.start_base_pos

    def get_start_base_orientation(self):
        return self.start_base_orientation

    def get_file_path(self):
        return self.file_path

    def generate_urdf(self):
        pass

    def get_instrument_world(self):
        bullet_pos = np.array(pb.getLinkState(self.model_id, self.instrument_id)[4])
        bullet_rot = np.array(pb.getLinkState(self.model_id, self.instrument_id)[5])
        return to_homogeneous(bullet_pos, R.from_quat(bullet_rot).as_matrix())

    def add_position(self):
        self.position_story_actual.append(self.get_instrument_world())

    def simulation_step(self, t):
        raise NotImplementedError

    def set_action(self, act):
        raise NotImplementedError

    def set_state(self, ns):
        raise NotImplementedError

    def check_state(self, t):
        if self.state.is_fsm_state():
            g = None
            while self.event_list:
                e = self.event_list
                self.event_list = self.event_list[1:]
                g = self.state.check_event(e)
                if g is not None:
                    break
            if g is None:
                g = self.state.use_default()
            if g is not None:
                l = g.generate_controller(t)
                self.set_action(l)
        else:
            if self.state.is_finished(t):
                self.set_state(self.state.next_state)
                self.check_state(t)
