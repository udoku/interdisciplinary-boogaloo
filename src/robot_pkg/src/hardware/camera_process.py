#!/usr/bin/env python

from __future__ import division, print_function
import rospy
import cv_bridge
import cv2
import os
import numpy as np

from robot_pkg.msg import Frame, RobotState
from sensor_msgs.msg import Image

current_state = None
cap = None
bridge = None
camera_pub = None

CAMERA_IMAGES_TOPIC = "images/camera"
ROBOT_STATE_TOPIC = "robot_state"

DIM=(640, 480)
K=np.array([[312.1901398049754, 0.0, 321.2870436838706], [0.0, 312.3400022092426, 218.27186480894005], [0.0, 0.0, 1.0]])
D=np.array([[-0.04833960724822996], [0.03665149872954583], [-0.05474311939603214], [0.02306394385055088]])

def main():
    global bridge
    global cap
    global camera_pub
    global CAMERA_IMAGES_TOPIC
    global ROBOT_STATE_TOPIC

    bridge = cv_bridge.CvBridge()
    rospy.init_node('camera_process')

    cap = cv2.VideoCapture(0)

    camera_timer = rospy.Timer(rospy.Duration(1.0/30.0), publish_camera)
    camera_pub = rospy.Publisher(CAMERA_IMAGES_TOPIC, Frame)
    state_sub = rospy.Subscriber(ROBOT_STATE_TOPIC, RobotState, handle_new_state, queue_size=1)

    print('Camera process initialized')

    rospy.spin()

def publish_camera(time):
    global current_state
    global cap
    global camera_pub
    global bridge

    if (current_state == None):
        print('No state received yet')
        return

    _, img = cap.read()
    new_frame = Frame()
    new_frame.state = current_state
    new_frame.image = bridge.cv2_to_imgmsg(undistort(img), 'bgr8')

    camera_pub.publish(new_frame)

def handle_new_state(msg):
    global current_state
    current_state = msg

def undistort(img, balance=0.5):
    global DIM
    global K
    global D

    dim1 = img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort
    assert dim1[0]/dim1[1] == DIM[0]/DIM[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
    dim2 = dim1
    dim3 = dim1

    scaled_K = K * dim1[0] / DIM[0]  # The values of K is to scale with image dimension.
    scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
    # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, dim3, cv2.CV_16SC2)
    return cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

if __name__ == '__main__':
    main()
