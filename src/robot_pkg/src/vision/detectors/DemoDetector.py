from __future__ import division, print_function
import numpy as np
import cv2
import math
import os
import sys
import random
from vision_utils import *
from robot_pkg.msg import Detection
from joblib import load

ISCV3 = cv2.__version__[0]=="3"

classifier = None

class DemoDetector(py_detector):

    def __init__(self):
        this_file_dir = os.path.dirname(os.path.abspath(__file__))
        ASSETS_PATH = os.path.join(this_file_dir, "../assets/")
        global classifier
        classifier = load(ASSETS_PATH + 'demo.joblib')

    def reset(self):
        pass

    def detect(self, images, vision):
        print('demo detecting')
        global classifier

        img = images[vision.IMAGE]
        binary = treeThreshold(img, classifier)

        kernel = np.ones((3,3),np.uint8)
        binary = cv2.erode(binary,kernel,iterations = 5)
        binary = cv2.dilate(binary,kernel,iterations = 5)

        feedback_image = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)

        _, contours, hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        good_color = (0,255,0) # green

        if contours == []:
            images[vision.DETECTOR] = feedback_image
            return []

        contours.sort(key=cv2.contourArea, reverse=True)
        cont = contours[0]
        cv2.drawContours(feedback_image, [cont], -1, good_color, 2)

        M = cv2.moments(cont)
        if M['m00'] == 0:
            return []
        cX = M["m10"] / M["m00"]
        cY = M["m01"] / M["m00"]

        print('pixel: ' + str(cX) + ' ' + str(cY))

        goal = vision.pixel_to_global([cX, cY])

        print('global:' + str(goal))

        images[vision.DETECTOR] = feedback_image
        
        return [py_detection(Detection.DEMO, goal)]
