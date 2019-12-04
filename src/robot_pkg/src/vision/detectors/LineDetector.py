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

# TUNE ME
ROI_VERTICES = np.array([[(110, 150), (150, 330), (490, 330), (530, 150)]], np.int32)
# TUNE ME
THRESH_VAL = 40

# If you switch to the DecisionTree thresholding:
# Blur -> decision tree threshold -> contract/dilate?

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
    
def extract_target_points(box):
    # Find short sides, assign corners to variables
    
    # Known point order - one of [top_l, top_r], or [top_l, btm_l] is a long side
    sides = np.array([[box[0], box[1]], [box[0], box[3]]])
    long_side_ind = np.argmax(np.linalg.norm(np.array([box[0] - box[1], box[0] - box[3]]), ord=2, axis=1))
    long_side = sides[long_side_ind,:]
    
    # Find center of rectangle
    center = np.mean(np.array([box[0], box[2]]), axis=0)
    
    # Compute vector for long sides
    long_side_vec = long_side[1] - long_side[0]
    if long_side_vec[1] < 0:
        long_side_vec *= -1
    
    # Return center modified in both directions
    # First point - to move to, larger y coord (lower on screen)
    # Second point - to face, higher y
    return np.array([center + long_side_vec * 1/3, center - long_side_vec * 1/3], np.int32)

class LineDetector(py_detector):

    def __init__(self):
        pass

    def reset(self):
        pass

    def detect(self, images, vision):
        img = images[vision.IMAGE]
        feedback_image = images[vision.DETECTOR]

        # Load and preprocess image
        # preprocess steps: blur or erode/dilate, grayscale, threshold, mask to roi
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
        # blur (or erode/dilate)
        img = cv2.GaussianBlur(img, (7, 7), 0)
        
        # Convert to binary image
        _, img_1 = cv2.threshold(img, THRESH_VAL, 255, cv2.THRESH_BINARY)
        img_2 = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 127, 20)
        img = cv2.bitwise_and(img_1, img_2)
        
        kernel = np.ones((3,3),np.uint8)
        # Order? Necessary?
        img = cv2.erode(img, kernel, iterations=5)
        img = cv2.dilate(img, kernel, iterations=5)
        
        # Ignore all non-ROI points
        mask = 255 * np.ones_like(img, dtype=np.uint8)
        cv2.fillPoly(mask, ROI_VERTICES, 0)
        img = cv2.bitwise_or(img, mask)
        
        # Use contours on negative of image for reasons
        _, contours, _ = cv2.findContours(255-img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            contour = max(contours, key=cv2.contourArea)
        
        # Unlikely null case - no contours?
        else: 
            return []
        
        bounding_box = unscramble_box(np.int0(cv2.boxPoints(cv2.minAreaRect(contour))))
        low_pt, high_pt = extract_target_points(bounding_box)
        
        # Draw original image, contour, crop, etc.
        feedback_image = cv2.drawContours(feedback_image, [contour], -1, (0, 0, 255))
        feedback_image = cv2.polylines(feedback_image, ROI_VERTICES, True, (255, 0, 0))
        feedback_image = cv2.polylines(feedback_image, np.array([bounding_box]), True, (0, 255, 0))
        feedback_image = cv2.arrowedLine(feedback_image, tuple(low_pt), tuple(high_pt), (255,255,255))

        images[vision.DETECTOR] = feedback_image

        ret_low = vision.pixel_to_global(low_pt)
        ret_high = vision.pixel_to_global(high_pt)

        diff = ret_high - ret_low

        angle = math.atan2(diff[1], diff[0])
       
        return [py_detection(Detection.LINE, ret_low, orientation=angle)]
