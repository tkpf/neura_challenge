import numpy as np
import cv2 as cv

def getEdges(img):
    # # Apply Gaussian Smoothing
    # img =  cv.GaussianBlur(img, (5, 5), 0)
    # Apply Bilateral smoothing
    img = cv.bilateralFilter(img,5,100,100) # experiment more with parameteres, default values were 9, 75, 75

    gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)

    gray = np.float32(gray)
    dst = cv.cornerHarris(gray,3,5,0.04) # experiment with parameters, default values 2, 3, 0.04
    print(np.shape(dst))
    #print(dst>0.12*dst.max())
    count = np.count_nonzero(dst>0.12*dst.max())
    print(count)
    #result is dilated for marking the corners, not important
    dst = cv.dilate(dst,None)

    # Threshold for an optimal value, 0.1-0.2 seems to be a code choice, default value was 0.01
    img[dst>0.12*dst.max()]=[0,0,255]
    return img