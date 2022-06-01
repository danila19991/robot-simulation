import pybullet

from virtual_modeling.basic_model import BaseRobot
import pybullet as pb
import numpy as np
from scipy.spatial.transform import Rotation as R
from virtual_modeling.data_type import to_homogeneous, get_pos, get_orient_quat


class ManipulatorRobot(BaseRobot):

    def __init__(self, ctx):
        super(ManipulatorRobot, self).__init__(ctx)
        self.file_path = "models/manipulator.urdf"
        self.start_base_pos = [0, 0, 0]
        self.start_orientation = pb.getQuaternionFromEuler([0, 0, 0])
        self.start_world_pos = None
        self.start_world_orient = None
        self.joint_indices = [1, 2, 3, 4, 5, 6]
        self.start_joints = [0, 0, np.pi/2, 0, np.pi/2, 0]
        self.curr_world = None
        self.control_joint_func = None
        self.control_world_func = None
        self.instrument_id = 7

    def get_world_from_joint(self, hom):
        return np.array(pb.calculateInverseKinematics(
            bodyIndex=self.model_id,
            endEffectorLinkIndex=self.instrument_id,
            targetPosition=get_pos(hom),
            targetOrientation=get_orient_quat(hom)
        ))

    def simulation_step(self, t):
        if self.control_world_func and not self.ctx.skip:
            pb.setJointMotorControlArray(bodyIndex=self.model_id,
                                         jointIndices=self.joint_indices,
                                         targetPositions=self.get_world_from_joint(self.control_world_func(t)),
                                         controlMode=pb.POSITION_CONTROL)
        elif self.control_joint_func and not self.ctx.skip:
            pb.setJointMotorControlArray(bodyIndex=self.model_id,
                                         jointIndices=self.joint_indices,
                                         targetPositions=self.control_joint_func(t),
                                         controlMode=pb.POSITION_CONTROL)
        elif self.curr_world is not None:
            pb.setJointMotorControlArray(bodyIndex=self.model_id,
                                         jointIndices=self.joint_indices,
                                         targetPositions=self.get_world_from_joint(self.curr_world),
                                         controlMode=pb.POSITION_CONTROL)
        else:
            pb.setJointMotorControlArray(bodyIndex=self.model_id,
                                         jointIndices=self.joint_indices,
                                         targetPositions=self.start_joints,
                                         controlMode=pb.POSITION_CONTROL)

    def add_position(self):
        self.position_story_actual.append(self.get_instrument_world())
        self.position_story_joint.append(tuple(s[:2] for s in pb.getJointStates(self.model_id, self.joint_indices)))

    def set_action(self, act):
        self.state = act
        self.control_world_func = self.state.control

    def set_state(self, ns):
        self.control_world_func = None
        self.state = ns
        self.curr_world = self.state.get_homogeneous()
