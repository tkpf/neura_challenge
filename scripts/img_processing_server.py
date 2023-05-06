# !/usr/bin/env python

# some snippets may originally come from https://github.com/lucasw/image_manip/blob/master/image_manip/scripts/image_folder_publisher.py
import rospy
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from vision_processing.srv import FindEdgesInImage, FindEdgesInImageResponse



def edge_detection_service():
    rospy.init_node("edge_detection_service")
    s = rospy.Service("vision_processing_srv", FindEdgesInImage, detectEdges)
    print("Ready to process Image...")
    rospy.spin()

def detectEdges(req_img):
    bridge = CvBridge()
    img_in_cvformat = bridge.imgmsg_to_cv2(req_img)
    processed_img_in_cvformat = getEdges(img_in_cvformat)
    return FindEdgesInImageResponse(bridge.cv2_to_imgmsg(processed_img_in_cvformat))#, encoding='bgr8'))

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

if __name__ == '__main__':
    print("Main called.")
    try:
        edge_detection_service()
    except rospy.ROSInterruptException:
        print("Error occured. Abort...")
        rospy.logerr("Error")
