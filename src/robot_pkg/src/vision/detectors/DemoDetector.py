from __future__ import division, print_function
import numpy as np
import cv2
import math
import os
import sys
import random
from vision_utils import *
from robot_pkg.msg import Detection

ISCV3 = cv2.__version__[0]=="3"

class DemoDetector(py_detector):

    def __init__(self):
        pass

    def reset(self):
        pass

    def detect(self, images, vision):

        return [py_detection(Detection.DEMO, [6, 9])]
