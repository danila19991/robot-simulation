import pybullet as pb
import pybullet_data
import numpy as np
from .data_type import inverse_homogeneous


class Camera:
    TIME_DELTA = 1 / 10
    QUALITY = 80

    default_size = {
        'width': 1000,
        'height': 1000
    }
    up_vector = np.array([0, 1, 0])
    eye_position = np.array([0, 0, 1])
    camera_eye_direction = np.array([0, 0, -1])
    focus_length = 1

    def __init__(self, size=None,
                 camera_eye_position=None,
                 fov=60,
                 camera_target_position=None,
                 up_vector=None,
                 camera_eye_direction=None,
                 focus_length=None):
        if size is None:
            size = Camera.default_size
        self.size = size
        if up_vector is None:
            up_vector = Camera.up_vector
        self.up_vector = up_vector
        if camera_eye_position is None:
            camera_eye_position = Camera.eye_position
        self.camera_pos = np.array(camera_eye_position)
        if camera_target_position is not None:
            self.focus_length = np.linalg.norm(np.array(camera_eye_position) - np.array(camera_target_position))
            self.cameraTargetPosition = camera_target_position
            self.eye_direction = np.array(camera_target_position) - np.array(camera_eye_position)
        elif camera_eye_direction is not None and focus_length is not None:
            self.focus_length = focus_length
            self.cameraTargetPosition = camera_eye_position + camera_eye_direction/np.linalg.norm(camera_eye_direction)*focus_length
            self.eye_direction = camera_eye_direction
        else:
            camera_eye_direction = Camera.camera_eye_direction
            focus_length = Camera.focus_length
            self.focus_length = focus_length
            self.cameraTargetPosition = camera_eye_position + camera_eye_direction/np.linalg.norm(camera_eye_direction)*focus_length
            self.eye_direction = camera_eye_direction
        self.eye_direction = self.eye_direction/np.linalg.norm(self.eye_direction)
        self.up_vector = self.up_vector - self.eye_direction*np.sum(self.eye_direction*self.up_vector)
        self.up_vector = self.up_vector/np.linalg.norm(self.up_vector)
        self.fov = fov
        self.viewMatrix = None
        self.projectionMatrix = None
        self.cam_image_kwargs = None

    def get_focus_length(self):
        return self.focus_length

    def get_pixel_focus_length(self):
        return self.focus_length * self.size['height'] / 2

    def get_camera_pos(self):
        return self.camera_pos

    def get_camera_to_world(self):
        u = np.cross(-self.up_vector, self.eye_direction)
        return np.array([
            [u[0], -self.up_vector[0], self.eye_direction[0], self.camera_pos[0]],
            [u[1], -self.up_vector[1], self.eye_direction[1], self.camera_pos[1]],
            [u[2], -self.up_vector[2], self.eye_direction[2], self.camera_pos[2]],
            [0, 0, 0, 1]
        ])

    def set_new_height(self, h):
        self.__init__(size=self.size, height=h)

    def get_dist_coefs(self):
        return np.zeros([0, 0, 0, 0])

    def get_camera_matrix(self):
        return np.array([[self.get_pixel_focus_length(), 0, self.size['width'] / 2],
                                     [0, self.get_pixel_focus_length(), self.size['height'] / 2],
                                     [0, 0, 1]])

    def pb_init(self):
        self.viewMatrix = pb.computeViewMatrix(
            cameraEyePosition=self.camera_pos,
            cameraTargetPosition=self.cameraTargetPosition,
            cameraUpVector=self.up_vector)
        self.projectionMatrix = pb.computeProjectionMatrixFOV(
            fov=self.fov,
            aspect=1.0,
            nearVal=0.1,
            farVal=100)
        self.focus_length = self.projectionMatrix[0]
        self.cam_image_kwargs = {
            **self.size,
            'viewMatrix': self.viewMatrix,
            'projectionMatrix': self.projectionMatrix,
            'renderer': pb.ER_TINY_RENDERER
        }

    def get_frame(self):
        """
        returns RGBA array of size (x, y, 4)
        """
        return pb.getCameraImage(**self.cam_image_kwargs)[2]
