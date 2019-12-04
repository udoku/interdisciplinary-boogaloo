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

# TUNE ME
ROI_VERTICES = np.array([[(110, 150), (150, 330), (490, 330), (530, 150)]], np.int32)

def unscramble_box(box):
    srt = np.argsort(box[:, 1])
    btm1 = box[srt[0]]
    btm2 = box[srt[1]]

    top1 = box[srt[2]]
    top2 = box[srt[3]]

    bc = btm1[0] < btm2[0]
    btm_l = btm1 if bc else btm2
    btm_r = btm2 if bc else btm1

    tc = top1[0] < top2[0]
    top_l = top1 if tc else top2
    top_r = top2 if tc else top1

    return np.array([top_l, top_r, btm_r, btm_l])
    
class PlateauDetector(py_detector):
    def __init__(self):
        this_file_dir = os.path.dirname(os.path.abspath(__file__))
        ASSETS_PATH = os.path.join(this_file_dir, "../assets/")
        global classifier
        classifier = pickle.load(open(ASSETS_PATH + 'ramp.pickle'))
        
        
    def reset(self):
        pass
        
    def detect(self, images, vision):
        global classifier
        img = images[vision.IMAGE]
        binary = treeThreshold(img, classifier)
        
        kernel = np.ones((3,3), np.uint8)
        binary = cv2.erode(binary, kernel, iterations=3)
        binary = cv2.dilate(binary, kernel, iterations=3)
        
        feedback_image = img.copy()
        
        _, contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            images[vision.DETECTOR] = feedback_image
            return []
        
        contour = max(contours, key=cv2.contourArea)
        
        # TODO - if contour is too small (only captured some of tape,
        # and not enough to determine angle, then pretend we saw nothing?
        
        # See: img_2
        
        cv2.drawContours(feedback_image, [contour], -1, (0, 255, 0), 2)
        
        # I really don't know what else to do besides take a bounding
        # and use it to yeet out a predicted angle
        bounding_box = unscramble_box(np.int0(cv2.boxPoints(cv2.minAreaRect(contour))))
        feedback_image = cv2.polylines(feedback_image, np.array([bounding_box]), True, (0, 255, 0))

        # Compute vector from btm_r to top_r
        ret_br = vision.pixel_to_global(bounding_box[2])
        ret_tr = vision.pixel_to_global(bounding_box[1])
        
        dx,dy = ret_tr - ret_br
        
        # TODO - check angle math, idk
        angle = math.atan2(dy,dx)
    
        return [py_detection(Detection.PLATEAU, ret_br, orientation=angle)]