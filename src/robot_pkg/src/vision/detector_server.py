#!/usr/bin/env python

from __future__ import division, print_function
import rospy
import cv_bridge
import cv2
import os

from detectors.vision_utils import *

from robot_pkg.msg import Detection, Detections, Detector
from robot_pkg.srv import RunDetector, SetupDetector

from detectors.DemoDetector import DemoDetector
from detectors.LineDetector import LineDetector
from detectors.ObstacleDetector import ObstacleDetector

DETECTOR_SETUP_TOPIC = 'detector_setup'
DETECTOR_RUN_TOPIC = 'detector_run'

CAMERA_YAML_PATH = 'calib/camera_data.yml'

VALID_DETECTORS = {
    Detector.DEMO_DETECTOR: DemoDetector,
    Detector.LINE_DETECTOR: LineDetector,
    Detector.OBSTACLE_DETECTOR: ObstacleDetector
}

bridge = None
detector = None

def main():
    global bridge
    bridge = cv_bridge.CvBridge()
    rospy.init_node('detector_server')
    print('Detector server initialized')

    setup_service = rospy.Service(DETECTOR_SETUP_TOPIC, SetupDetector, handle_setup_calls)
    run_service = rospy.Service(DETECTOR_RUN_TOPIC, RunDetector, handle_run_calls)
    rospy.spin()

def handle_setup_calls(req):
    if req.detector.detector_type not in VALID_DETECTORS:
        if (int(req.detector.detector_type) != int(Detector.NO_DETECTOR)):
            print("detector name {} not found".format(req.detector.detector_type))
        return {'success': False}
    print("detector name {} about to setup".format(req.detector.detector_type))
    global detector
    detector = VALID_DETECTORS[req.detector.detector_type]()
    detector.set_mode(req.detector.detector_mode)

    return {
        'success': True,
    }

def handle_run_calls(req):
    global detector
    if req.detector.detector_type not in VALID_DETECTORS:
        if (int(req.detector.detector_type) != int(Detector.NO_DETECTOR)):
            print("detector name {} not found".format(req.detector.detector_type))
        return {'success': False}
    if detector == None:
        print("No detector is set up!")

    vision_obj = py_vision(req.frame.state)

    images = [None, None]
    img = bridge.imgmsg_to_cv2(req.frame.image, 'bgr8')

    images[vision_obj.IMAGE] = img
    images[vision_obj.DETECTOR] = images[vision_obj.IMAGE].copy()

    out = detector.detect(images, vision_obj)

    feedback = bridge.cv2_to_imgmsg(images[vision_obj.DETECTOR], 'bgr8')
    detections = Detections()
    if out is None:
        detections.detections = []
    else:
        detections.detections = [Detection(detection.pos_x,
                                            detection.pos_y,
                                            detection.orientation,
                                            detection.width,
                                            detection.depth,
                                            detection.height,
                                            detection.det_type,
                                            detection.confidence)
                                for detection in out]

    return {'success': True, 'feedback': feedback, 'detections': detections}

if __name__ == '__main__':
    main()
