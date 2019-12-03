from __future__ import division, print_function
import numpy as np
import cv2
import math
import os
import sys
import random
from vision_utils import *
from robot_pkg.msg import Detection
import pickle

ISCV3 = cv2.__version__[0]=="3"

classifier = None

class ObstacleDetector(py_detector):

    def __init__(self):
        this_file_dir = os.path.dirname(os.path.abspath(__file__))
        ASSETS_PATH = os.path.join(this_file_dir, "../assets/")
        global classifier
        classifier = pickle.load(open(ASSETS_PATH + 'obstacle.pickle'))

    def reset(self):
        pass

    def detect(self, images, vision):
        print('obstacle detecting')
        global classifier

        img = images[vision.IMAGE]
        binary = treeThreshold(img, classifier)

        kernel = np.ones((3,3),np.uint8)
        binary = cv2.erode(binary,kernel,iterations = 5)
        binary = cv2.dilate(binary,kernel,iterations = 5)

        feedback_image = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)

        _, contours, hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        good_color = (0,255,255) # cyan

        if contours == []:
            images[vision.DETECTOR] = feedback_image
            return []

        cv2.drawContours(feedback_image, contours, -1, good_color, 2)

        pts = []
        for cont in contours:
            for pt in cont:
                pts.append(pt)

        print('number of obstacle pixels: {}', len(pts))

        goals = [vision.pixel_to_global(pt) for pt in pts]

        images[vision.DETECTOR] = feedback_image

        return [py_detection(Detection.OBSTACLE, goals)]
