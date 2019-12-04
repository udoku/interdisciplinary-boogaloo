#!/usr/bin/env python

import os
import sys
import cv2
import numpy as np
from matplotlib import pyplot as plt

def m_syms(m):
    """
    returns all the square symmetries of a matrix
    """
    mt = np.transpose(m)
    ret = [m]
    for i in range(3):
        m = np.rot90(m)
        ret += [m]
    ret += [mt]
    for i in range(3):
        mt = np.rot90(mt)
        ret += [mt]

    return ret

def detect_junctions(img):
    """
    This function detects junctions in skeletonized binary images
    """
    kernels = [
        # .+.
        # .++
        # .+.
        m_syms(np.array([
            [-1, 1,-1],
            [-1, 1, 1],
            [-1, 1,-1]
        ], np.int8)),
        # ..+
        # .+.
        # +.+
        m_syms(np.array([
            [-1,-1, 1],
            [-1, 1,-1],
            [ 1,-1, 1]
        ], np.int8)),
        # .+.
        # .++
        # +..
        m_syms(np.array([
            [-1, 1,-1],
            [-1, 1, 1],
            [ 1,-1,-1]
        ], np.int8)),
        # +..
        # .++
        # +..
        m_syms(np.array([
            [ 1,-1,-1],
            [-1, 1, 1],
            [ 1,-1,-1]
        ], np.int8)),
    ]
    
    flat_kernels = [x for y in kernels for x in y]

    found = np.zeros(img.shape, np.uint8)
    for k in flat_kernels:
        found = found | cv2.morphologyEx(img, cv2.MORPH_HITMISS, k)
    return found

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

def detect_wye(img):
    """
    This function computes an image with a pixel set at the largest "wye" detected in the image
    The function returns the binary wye detection image, the threshold image, the smoothed contour, 
    and the skeleton images as a tuple
    """
    img = cv2.medianBlur(img, 5)

    h,w = np.shape(img)

    thresh = 255-cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 127, 20)
    thresh = cv2.ellipse(thresh, (w/2, h), (400, 100), 0, 0, 360, (0,0,0), -1)
    thresh = cv2.rectangle(thresh, (0,0), (w, 100), (0,0,0), -1)

    contours, heirarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    contours = filter(lambda c: len(c) > 50, contours)
    largest = max(contours, key = lambda c: cv2.contourArea(c))

    largest = laplacian_smoothing(largest, n=16)

    skel = np.zeros(img.shape, np.uint8)
    skel = cv2.drawContours(skel, [largest],-1, (255,255,255), -1)
    closed = np.copy(skel)
    skel = cv2.ximgproc.thinning(skel)
    mask = np.zeros(skel.shape, np.uint8)
    mask = cv2.rectangle(mask, (32, 32), (mask.shape[1]-32, mask.shape[0]-32), (255,255,255), -1)
    skel = skel & mask
    # skel_full = np.copy(skel)
    detect = detect_junctions(skel)

    return detect, thresh, closed, skel


if __name__ == "__main__":
    filename = sys.argv[1]
    img = cv2.imread(filename)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    detect, thresh, closed, skel = detect_wye(img)

    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    # img2 = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
    thresh = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)

    # img = cv2.drawContours(img, contours, -1, (0,0,255), 3)
    # img = cv2.drawContours(img, [largest], -1, (0,255,0), 3)

    plt.subplot(2,3,1)
    plt.imshow(img)
    plt.subplot(2,3,2)
    plt.imshow(thresh)
    plt.subplot(2,3,3)
    plt.imshow(closed)
    plt.subplot(2,3,4)
    plt.imshow(skel)
    plt.subplot(2,3,5)
    plt.imshow(detect)
    plt.show()