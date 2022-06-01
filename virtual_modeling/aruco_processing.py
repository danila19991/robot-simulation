import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from .data_type import to_homogeneous


class ArucoProcessing:

    def __init__(self):
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()

    def process_image(self, camera):
        data = camera.get_frame()
        # RGB -> BGR UMat for opencv
        img = np.asarray(data[:, :, [2, 1, 0]], dtype=np.uint8)
        corners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(img, self.dictionary, parameters=self.parameters)
        # print(corners)
        img = cv2.UMat(img)
        cv2.aruco.drawDetectedMarkers(img, corners, markerIds)

        markerLength = 0.15
        distCoeffs = camera.get_dist_coefs()
        cameraMatrix = camera.get_camera_matrix()
        rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, markerLength, cameraMatrix,
                                                                       distCoeffs)

        # print(rvecs, tvecs)
        # img = cv2.aruco.drawAxis(img, cameraMatrix, distCoeffs, rvecs, tvecs, markerLength)
        # res = cv2.resize(img, (int(500), int(500)))
        # cv2.imshow('Marker', res)
        # cv2.waitKey(0)

        res = dict()

        if rvecs is not None and tvecs is not None and markerIds is not None:
            for rvec, tvec, marker_id in zip(rvecs, tvecs, markerIds):
                res[marker_id[0]] = to_homogeneous(tvec[0], cv2.Rodrigues(rvec)[0])

        return res


