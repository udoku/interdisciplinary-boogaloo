from __future__ import division, print_function
import numpy as np
import cv2
import math
from vision_utils import *
from robot_pkg.msg import Detection

ISCV3 = cv2.__version__[0]=="3"
# Assumed fixed value
IMAGE_SIZE = (480, 720)

CV2_THRESHOLD = 60

# cv2 masks: 1 = set in output, 0 = leave alone
FISHEYE_MASK = np.load("")

UPPER_CROP = 1./8
LOWER_CROP = 1./2
LEFT_CROP = 1./3
RIGHT_CROP = 1./3

CROP_REGION = np.zeros(IMAGE_SIZE)
CROP_REGION[int(IMAGE_SIZE[0] * UPPER_CROP):int(IMAGE_SIZE[0] * (1-LOWER_CROP)),
    int(IMAGE_SIZE[1] * LEFT_CROP):int(IMAGE_SIZE[1] * (1-RIGHT_CROP))] = 1
    
FULL_MASK = cv2.bitwise_or(FISHEYE_MASK, CROP_REGION)

class LineDetector(py_detector):

    def __init__(self):
        pass

    def reset(self):
        pass

    def detect(self, images, vision):        
        # Extract and copy first image to do work on
        draw_image = images[vision_obj.DETECTOR]
        height, width, _ = draw_image.shape
        
        _, preproc = cv2.GaussianBlur(cv2.cvtColor(
                images[vision_obj.IMAGE].copy(),
                cv2.COLOR_BGR2GRAY))

        # Generate pure white image
        work_image = 255 * np.ones(preproc.shape)
        # To be overwritten by preprocessed image in locations not in mask
        cv2.bitwise_and(preproc, preproc, work_image, mask=FULL_MASK)
                
        work_image = cv2.threshold(work_image, CV2_THRESHOLD, 255)
                        
        contours, _ = cv2.findContours(work_image, 1, cv2.CV_CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            line = max(contours, key=cv2.contourArea)
            center_x = int(line['m10']/line['m00'])
            center_y = int(line['m01']/line['m00'])
            
            # Mark center
            cv2.line(draw_image, (center_x,0), (center_x, height), (255,0,0), 1)
            cv2.line(draw_image, (0,center_y), (width, center_y), (255,0,0), 1)
 
            # Display contour
            cv2.drawContours(draw_image, contours, -1, (0, 255, 0), 1)
                        
            output_global_loc = vision.pixel_to_global(np.array([center_x, center_y]))
            
            # Return pixel to location
            return [py_detection(Detection.LINE,
                [output_global_loc, output_global_loc])]
        
        else:
            # TODO: No line in region - what to do?
            return []