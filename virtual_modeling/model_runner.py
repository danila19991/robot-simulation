from .model_ctx import ModelCtx
import pybullet as pb
import pybullet_data
from camera import Camera
from .aruco_processing import ArucoProcessing
import numpy as np
from scipy.spatial.transform import Rotation as R
from .data_type import to_homogeneous
import time


class ModelRunner:

    def __init__(self, ctx: ModelCtx):
        self.physics_client = None
        self.ctx = ctx
        self.plane_id = None
        self.robots = list()
        self.cameras = list()
        self.aruco_processing = ArucoProcessing()
        self.aruco_positions = list()

    def initialise(self):
        self.aruco_positions = list()
        self.physics_client = pb.connect(pb.GUI if self.ctx.with_gui else pb.DIRECT)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        pb.setGravity(self.ctx.gravity[0], self.ctx.gravity[1], self.ctx.gravity[2])
        self.plane_id = pb.loadURDF("plane.urdf")
        for i, _ in enumerate(self.robots):
            self.robots[i].model_id = pb.loadURDF(self.robots[i].get_file_path(), self.robots[i].get_start_base_pos(),
                                                  self.robots[i].get_start_base_orientation())
        for i, _ in enumerate(self.cameras):
            self.cameras[i].pb_init()

    def run(self, s0, s1, s2=1):
        self.ctx.skip = True
        for i in range(s0):
            pb.stepSimulation()
            for rob in self.robots:
                rob.simulation_step(i)
        self.ctx.skip = False
        for i2, rob in enumerate(self.robots):
            self.robots[i2].add_position()
        for j in range(s1):
            for i in range(s2):
                t = j*s2 + i
                pb.stepSimulation()
                for rob in self.robots:
                    rob.check_state(t)
                    rob.simulation_step(t)
                for i2, rob in enumerate(self.robots):
                    self.robots[i2].add_position()
                if self.ctx.with_gui:
                    time.sleep(self.ctx.dt)

            for cam in self.cameras:
                aruco_positions = self.aruco_processing.process_image(cam)
                self.aruco_positions.append(aruco_positions)

                ctow = cam.get_camera_to_world()
                for i1, pos in aruco_positions.items():
                    for j1, rob in enumerate(self.robots):
                        p = pos
                        if i1 in rob.markers:
                            p = np.matmul(p, rob.markers[i1])
                            p = np.matmul(ctow, p)
                            self.robots[j1].position_story_detected.append(p)
                            # self.robots[j1].curr_pos = self.robots[j1].get_world_from_joint(p)

    def deinitialise(self):
        pb.disconnect()

