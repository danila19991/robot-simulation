import numpy as np


class ModelCtx:

    def __init__(self):
        self.gravity = np.array([0, 0, -9.8])
        self.plane_path = "plane.urdf"
        self.with_gui = False
        self.dt = 1./240
        self.skip = True
