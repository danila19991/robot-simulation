from virtual_modeling.basic_model import BaseRobot
import pybullet as pb
import numpy as np
from scipy.spatial.transform import Rotation as R
from virtual_modeling.data_type import to_homogeneous, get_angle_x


class TwoWheelRobot(BaseRobot):

    def __init__(self, ctx):
        super(TwoWheelRobot, self).__init__(ctx)
        self.file_path = "models/2_wheel_robot.urdf"
        self.start_base_pos = [0, 0, 0.4]
        self.start_base_orientation = pb.getQuaternionFromEuler([0, 0, 0])
        self.start_control = np.array([0., 0.])
        self.markers[4] = to_homogeneous(np.array([0, 0, -0.2]), R.from_euler("zyx", np.array([-np.pi/2, 0, 0])).as_matrix())
        self.instrument_id = 3
        self.joint_indices = [0, 1]
        self.control = None
        self.curr_pos = np.array((0., 0., 0.))
        self.rw = 0.15
        self.rc = 0.2
        self.state = None
        self.prev_vel = np.array([0., 0.])
        self.mx_dv = 0.1

    def smooth_control(self, new_w):
        return new_w
        # new_w = np.where(new_w > self.prev_vel + self.mx_dv, self.prev_vel+self.mx_dv, new_w)
        # new_w = np.where(new_w < self.prev_vel - self.mx_dv, self.prev_vel-self.mx_dv, new_w)
        # self.prev_vel = new_w
        # return new_w

    def simulation_step(self, t):
        if self.ctx.skip or self.control is None:
            pb.setJointMotorControlArray(bodyIndex=self.model_id,
                                         jointIndices=self.joint_indices,
                                         targetVelocities=self.smooth_control(self.start_control),
                                         controlMode=pb.VELOCITY_CONTROL)
        else:
            pb.setJointMotorControlArray(bodyIndex=self.model_id,
                                         jointIndices=self.joint_indices,
                                         targetVelocities=self.smooth_control(self.control(self.curr_pos, t)),
                                         controlMode=pb.VELOCITY_CONTROL)

    def get_world_from_joint(self, hom):
        return np.array((
            hom[0, 3],
            hom[1, 3],
            get_angle_x(hom)
        ))

    def add_position(self):
        world_instrument = self.get_instrument_world()
        self.curr_pos = self.get_world_from_joint(world_instrument)
        self.position_story_actual.append(world_instrument)
        self.position_story_joint.append(self.curr_pos)

    def set_action(self, act):
        self.state = act
        self.control = self.state.control

    def set_state(self, ns):
        self.control = None
        print(ns, '-----')
        self.state = ns
