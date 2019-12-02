from __future__ import division, print_function
import numpy as np
import cv2
import math
from vision_utils import *
from robot_pkg.msg import Detection

ISCV3 = cv2.__version__[0]=="3"

CV2_THRESHOLD = 60
FISHEYE_MASK = np.load("")

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
        cv2.bitwise_and(preproc, preproc, work_image, mask=FISHEYE_MASK)
        
        work_image = cv2.threshold(work_image, CV2_THRESHOLD, 255)
        
        # TODO: crop to desired fixed region to check for contour
                
        contours, _ = cv2.findContours(work_image, 1, cv2.CV_CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            line = max(contours, key=cv2.contourArea)
            center_x = int(line['m10']/line['m00'])
            center_y = int(line['m01']/line['m00'])
            
            # TODO: on crop, figure out how to undo crop to draw on og image
            
            # Mark center
            cv2.line(draw_image, (center_x,0), (center_x, height), (255,0,0), 1)
            cv2.line(draw_image, (0,center_y), (width, center_y), (255,0,0), 1)
 
            # Display contour
            cv2.drawContours(draw_image, contours, -1, (0, 255, 0), 1)
            
            # TODO: on crop, transform back to coords in og image
            
            output_global_loc = vision.pixel_to_global(np.array([center_x, center_y]))
            
            # Return pixel to location
            return [py_detection(Detection.LINE,
                [output_global_loc, output_global_loc])]
        
        else:
            # TODO: No line in region - what to do?
            return []