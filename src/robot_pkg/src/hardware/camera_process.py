#!/usr/bin/env python

from __future__ import division, print_function
import rospy
import cv_bridge
import cv2
import os

from robot_pkg.msg import Frame, RobotState
from sensor_msgs.msg import Image

current_state = None
cap = None
bridge = None
camera_pub = None

CAMERA_IMAGES_TOPIC = "images/camera"
ROBOT_STATE_TOPIC = "robot_state"

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
    state_sub = rospy.Subscriber(ROBOT_STATE_TOPIC, RobotState, handle_new_state)

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
    new_frame.image = bridge.cv2_to_imgmsg(img, 'bgr8')
    new_frame.state = current_state

    camera_pub.publish(new_frame)

def handle_new_state(msg):
    global current_state
    current_state = msg

if __name__ == '__main__':
    main()
