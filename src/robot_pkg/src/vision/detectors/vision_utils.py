#!/usr/bin/env python2

import numpy as np
import cv2
import numpy.linalg
import math

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
    def __init__(self, matrix, dist, state):
        self.CAMERA_MATRIX = matrix
        self.CAMERA_DISTORTION = dist
        self.ROBOT_STATE = state

        self.IMAGE = 0
        self.DETECTOR = 1

    def local_to_pixel(self, local):
        """ Returns the projection of the provided local coordinate using the
            projection matrix of the provided image. """
        # TODO Make sure this works?
        pinhole = np.dot(numpy.linalg.inv(self.CAMERA_MATRIX), local)
        dot_prod = np.dot(_rotation_matrix([0, 1, 0], math.pi / 4.0), pinhole)
        return np.roll(dot_prod, 2)

    def pixel_to_local(self, pixel):
        # TODO This probably doesn't work
        unnormalized = self._pixel_to_local_unnormalized(pixel, image)
        norm = np.linalg.norm(unnormalized)
        return unnormalized / norm
        #rot = _rotation_matrix([0, 1, 0], -math.pi / 4.0)
        #print("rot: ", rot)
        #print("pixel rolled: ", np.roll(pixel, 1))
        #pinhole = np.dot(rot, np.roll(pixel, 1))
        #return numpy.linalg.inv(self.CAMERA_MATRIX[image], pinhole)

    def undistort_image(self, distorted_image):
        # TODO Make sure this works
        cam_matrix = self.CAMERA_MATRIX
        cam_distortion = self.CAMERA_DISTORTION
        return cv2.undistort(distorted_image, cam_matrix, cam_distortion)

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

def vector3d(x, y, z):
    return numpy.array([x, y, z], dtype='f8')