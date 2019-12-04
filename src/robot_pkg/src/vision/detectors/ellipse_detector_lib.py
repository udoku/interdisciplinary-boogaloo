#!/usr/bin/env python

import os
import sys
import cv2
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy.spatial import ConvexHull

def laplacian_smoothing(c, n=1, l=1.0):
    """
    This function applies laplacian smoothing to a contour
    """
    for j in range(n):
        smooth = np.copy(c)
        for i in range(len(c)):
            x = (l * c[i][0][0] + c[i-1][0][0] + c[(i+1) % len(c)][0][0])/3.0
            y = (l * c[i][0][1] + c[i-1][0][1] + c[(i+1) % len(c)][0][1])/3.0
            smooth[i,0,:] = [x,y]
        c = smooth
    return c

def get_convex_hull(c):
    hull = ConvexHull(np.reshape(c,(-1, 2)))
    return np.reshape(
        c[hull.vertices],
        (-1, 1, 2)
    )

def ellipse_heuristic(c, shape):
    # get_convex_hull(laplacian_smoothing(c, n=16))
    c = get_convex_hull(c)
    ellipse = cv2.fitEllipseAMS(np.reshape(c, (-1,2)))

    e_mask = np.zeros(shape, np.uint8)
    c_mask = np.zeros(shape, np.uint8)
    center_mask = np.zeros(shape, np.uint8)

    e_mask = cv2.ellipse(e_mask, ellipse, 255, -1)
    c_mask = cv2.drawContours(c_mask, [c], -1, 255, -1)
    center_mask = cv2.rectangle(center_mask, (0, 100), (shape[1], shape[0]-100), 255, -1)

    _, (x,y), _ = ellipse

    if y < x:
        x,y=y,x
    
    # print x / float(y)

    if x / float(y) < 0.1:
        return 0

    intersection = e_mask & c_mask & center_mask
    union = (e_mask | c_mask) & center_mask

    us = float(np.sum(union))

    if us == 0:
        return 0

    h = np.sum(intersection) / us

    return h

NUM_ELLIPSE_POINTS = 20
def ellipseScore(cnt):
    """
    Calculates how well an ellipse approximates the contour
    :param cnt: the original contour
    """
    if(len(cnt) < 5):
        return math.inf
    ellipse = cv2.fitEllipse(cnt)
    center, axes, angle = ellipse
    int_center=tuple([int(z) for z in center])
    int_axes = tuple([int(z/2) for z in axes])
    ellipse_cnt=cv2.ellipse2Poly(int_center,int_axes,int(angle),0,360,int(360/NUM_ELLIPSE_POINTS))
    size = axes[0]

    square_error=0.0
    for pt in cnt[0::NUM_ELLIPSE_POINTS]:
        square_error += (cv2.pointPolygonTest(ellipse_cnt,tuple(pt[0]),True) / size)**2
    for pt in ellipse_cnt:
        square_error += (cv2.pointPolygonTest(cnt,tuple(pt),True) / size)**2
    return math.sqrt(square_error)

def detect_ellipse(img):
    """
    This function computes an image with a pixel set at the largest "wye" detected in the image
    The function returns the binary wye detection image, the threshold image, the smoothed contour, 
    and the skeleton images as a tuple
    """

    # apply a slight blur to help thresholding
    img = cv2.medianBlur(img, 5)

    h,w = np.shape(img)

    # apply adaptive thresholding to find lines
    thresh = 255-cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 63, 15)

    # mask out some areas we wish to ignore (eg robot and horizon)
    thresh = cv2.ellipse(thresh, (w/2, h), (400, 100), 0, 0, 360, (0,0,0), -1)
    thresh = cv2.rectangle(thresh, (0,0), (w, 60), (0,0,0), -1)
    # thresh = cv2.rectangle(thresh, (0, 3*h/4), (w/4, h), (0,0,0), -1)
    # thresh = cv2.rectangle(thresh, (3*w/4, 3*h/4), (w, h), (0,0,0), -1)

    detect = np.zeros(thresh.shape, np.uint8)
    closed = np.zeros(thresh.shape, np.uint8)

    contours, heirarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    contours = filter(lambda c: len(c) > 50, contours)
    # contours = map(lambda c: get_convex_hull(c), contours)

    contours = filter(lambda c: cv2.contourArea(get_convex_hull(c)) > 10000, contours)
    contours = filter(lambda c: ellipse_heuristic(c, img.shape) > 0.7, contours)

    ellipse = None

    if len(contours) > 0:

        # for c in contours:
        #     print cv2.contourArea(get_convex_hull(c)), ellipse_heuristic(c, img.shape)

        # largest = max(contours, key = lambda c: ellipseScore(c))
        largest = max(contours, key = lambda c: math.sqrt(cv2.contourArea(get_convex_hull(c))) * ellipse_heuristic(c, img.shape))
        # largest = max(contours, key = lambda c: cv2.contourArea(c))
        # largest = contours[]


        largest = laplacian_smoothing(largest, n=16)
        largest = get_convex_hull(largest)

        ellipse = cv2.fitEllipseAMS(np.reshape(largest, (-1,2)))

        detect = cv2.ellipse(detect, ellipse, (255,255,255), -1)

        closed = cv2.drawContours(closed, [largest], -1, (255,255,255), -1)

    return ellipse, detect, thresh, closed


if __name__ == "__main__":
    filename = sys.argv[1]
    img = cv2.imread(filename)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ellipse, detect, thresh, closed = detect_ellipse(img)

    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    # img2 = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
    thresh = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)

    img = cv2.ellipse(img, ellipse, (255,0,0), 3)

    # img = cv2.drawContours(img, contours, -1, (0,0,255), 3)
    # img = cv2.drawContours(img, [largest], -1, (0,255,0), 3)

    plt.subplot(2,3,1)
    plt.imshow(img)
    plt.subplot(2,3,2)
    plt.imshow(thresh)
    plt.subplot(2,3,3)
    plt.imshow(closed)
    plt.subplot(2,3,5)
    plt.imshow(detect)
    plt.show()