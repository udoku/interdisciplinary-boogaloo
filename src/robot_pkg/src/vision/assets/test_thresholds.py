import cv2
import numpy as np
from sklearn.tree import DecisionTreeClassifier
import pickle
import sys

def treeThreshold(image, classifier):
    shp = image.shape
    pixels = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    pixels = pixels.reshape(-1, 3)
    out = classifier.predict(pixels)
    out = out.reshape(shp[0], shp[1], 1)
    return out

def main():
    if len(sys.argv) != 3:
        print(str(sys.argv[0]) + ' [pickleFile] [testImage]')
        sys.exit(0)

    classifier = pickle.load(open(sys.argv[1]))

    cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('image', 1000, 1000)
    cv2.namedWindow('threshold', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('threshold', 1000, 1000)

    test = cv2.imread(sys.argv[2])
    cv2.imshow('image', test)

    out = treeThreshold(test, classifier)

    cv2.imshow('threshold', out)
    cv2.waitKey(0)


if __name__ == '__main__':
    main()
