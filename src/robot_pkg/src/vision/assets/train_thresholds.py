import cv2
import numpy as np
from sklearn.tree import DecisionTreeClassifier
import pickle
import sys
import os

def showRanges(classifier):
    x, y, z = np.indices((255,255,255), dtype=np.uint8)
    bgrColorSpace = np.stack([x, y, z], axis=3)
    x, y, z = np.indices((180,255,255), dtype=np.uint8)
    hsvColorSpace = np.stack([x, y, z], axis=3)

    cv2.namedWindow('bgr', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('bgr', 1000, 1000)
    cv2.namedWindow('hsv', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('hsv', 1000, 1000)

    for i in range(0, 256, 8):
        bgr = bgrColorSpace[i,:,:,:]
        shp = bgr.shape
        bgrMask = classifier.predict(cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV).reshape(-1, 3)).astype(np.uint8)
        bgrMask = bgrMask.reshape(shp[0], shp[1], 1)
        bgrMask = np.repeat(bgrMask, 3, axis=2)
        bgr = cv2.bitwise_and(bgr, bgrMask)
        bgr = bgr.reshape(shp)

        hsv = hsvColorSpace[:,:,i,:]
        shp = hsv.shape
        hsvMask = classifier.predict(hsv.reshape(-1, 3)).astype(np.uint8)
        hsvMask = hsvMask.reshape(shp[0], shp[1], 1)
        hsvMask = np.repeat(hsvMask, 3, axis=2)
        hsv = cv2.bitwise_and(hsv, hsvMask)
        hsv = hsv.reshape(shp)
        hsv = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

        cv2.imshow('bgr', bgr)
        cv2.imshow('hsv', hsv)
        cv2.waitKey(0)

    cv2.destroyWindow('bgr')
    cv2.destroyWindow('hsv')


def main():

    if len(sys.argv) != 3:
        print(str(sys.argv[0]) + ' [dataFolder] [pickleName]')
        sys.exit(0)

    folder = sys.argv[1]

    files = os.listdir(folder)

    images = []
    filters = []

    print('Files in directory: ' + str(files))

    i = 0
    while True:
        imgFile = 'img_' + str(i) + '.png'
        maskFile = 'mask_' + str(i) + '.png'
        if imgFile in files and maskFile in files:
            images.append(os.path.join(folder, imgFile))
            filters.append(os.path.join(folder, maskFile))
        else:
            break
        i += 1

    fileName = sys.argv[2]

    print('Using images: ' + str(images))
    print('Using masks: ' + str(filters))
    print('Save point: ' + fileName)

    image = np.concatenate([cv2.imread(name) for name in images], axis=0)
    filter = np.concatenate([cv2.imread(name) for name in filters], axis=0)

    filter = filter - image
    filter = cv2.cvtColor(filter, cv2.COLOR_BGR2GRAY)

    _,filter = cv2.threshold(filter,0,255,cv2.THRESH_BINARY)

    print('Loaded training images.')

    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    image = image.reshape(-1, 3)
    filter = filter.reshape(-1, 1)

    sparseNegatives = np.array([np.array([h, s, v]) for h in range(0,180,10)
                                           for s in range(0,256,16)
                                           for v in range(0,256,16)], dtype=np.uint8)

    zeros = np.array([np.array([0]) for _ in range(sparseNegatives.shape[0])], dtype=np.uint8)

    image = np.concatenate([image, sparseNegatives], axis=0)
    filter = np.concatenate([filter, zeros], axis=0)


    classifier = DecisionTreeClassifier(max_depth=20, min_samples_leaf=100)

    print('Initialized classifier. Training...')

    classifier.fit(image, filter)

    print('Trained! Saving to: ' + fileName)

    pickle.dump(classifier, open(fileName, 'w'))

    print('In-sample score: ' + str(classifier.score(image, filter)))

    showRanges(classifier)


if __name__ == '__main__':
    main()
