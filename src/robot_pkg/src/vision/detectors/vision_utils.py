#!/usr/bin/env python2

import numpy as np
import cv2
import numpy.linalg
import math

DIM=(640, 480)
K=np.array([[312.1901398049754, 0.0, 321.2870436838706], [0.0, 312.3400022092426, 218.27186480894005], [0.0, 0.0, 1.0]])
D=np.array([[-0.04833960724822996], [0.03665149872954583], [-0.05474311939603214], [0.02306394385055088]])
BALANCE=0.5

NEW_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, DIM, np.eye(3), balance=BALANCE)

CAM_HEIGHT = 0.14
CAM_FORWARD = 0.105

def _rotation_matrix(axis, theta):
    axis = np.asarray(axis)
    axis = axis / math.sqrt(np.dot(axis, axis))

    a = math.cos(theta / 2.0)
    b, c, d = -axis * math.sin(theta / 2.0)

    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a*b,  b * d, c * d

    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

class py_detector(object):
    def reset(self):
        raise NotImplementedError()
    def set_mode(self, mode):
        print("ERROR: Mode not implemeted! Please add in to use")
        pass
    def detect(self, images, vision):
        raise NotImplementedError()

class py_vision:
    def __init__(self, state):
        self.ROBOT_STATE = state

        self.IMAGE = 0
        self.DETECTOR = 1

    def pixel_to_global(self, pixel):
        new_pixel = np.array([pixel[0], pixel[1], 1])
        cam_vec = np.matmul(np.linalg.inv(NEW_K), new_pixel)
        print('camera vector: ' + str(cam_vec))
        cam_vec = np.matmul(_rotation_matrix(np.array([1.,0.,0.]), -math.pi/4.), cam_vec)
        print('rotated cam vec: ' + str(cam_vec))
        if cam_vec[1] < 0:
            return np.array([69., 0.])
        local_pos = (CAM_HEIGHT/cam_vec[1]) * cam_vec
        print('local (camera) pos: ' + str(local_pos))
        local_pos[2] += CAM_FORWARD

        local_x = local_pos[2]
        local_y = -local_pos[0]
        global_pos = np.array([
            local_x * math.cos(self.ROBOT_STATE.yaw) + local_y * math.sin(self.ROBOT_STATE.yaw) + self.ROBOT_STATE.pos_x,
            -local_x * math.sin(self.ROBOT_STATE.yaw) + local_y * math.cos(self.ROBOT_STATE.yaw) + self.ROBOT_STATE.pos_y
        ])

        print('global (camera) pos: ' + str(global_pos))

        return global_pos

class py_detection:
    def __init__(self, det_type, position, orientation=0.0, dims=[0.0,0.0,0.0], confidence=1.0):

        self.det_type = int(det_type)
        self.pos_x = position[0]
        self.pos_y = position[1]
        self.orientation = orientation
        self.width = dims[0]
        self.depth = dims[1]
        self.height = dims[2]
        self.confidence = confidence


class py_detection_type:
    INVALID = -1
    DEMO = 0

def treeThreshold(image, classifier):
    shp = image.shape
    pixels = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    pixels = pixels.reshape(-1, 3)
    out = classifier.predict(pixels)
    out = out.reshape(shp[0], shp[1], 1)
    return out