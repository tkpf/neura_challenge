# !/usr/bin/env python

import rospy
import cv2 as cv
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from edge_detection.srv import EdgeDetection, EdgeDetectionResponse


class Edge_Detection_Service:
    # init and start up service
    def __init__(self):
        self.cv_bridge = CvBridge()
        self.startup()

    def startup(self):
        rospy.init_node("edge_detection_service")
        rospy.loginfo("Starting Edge Detection Service...")
        s = rospy.Service("edge_detection", EdgeDetection, self.detection_service_callback)
        rospy.loginfo("Edge Detection Service up. Ready to process Image...")
        rospy.spin()

    def detection_service_callback(self, req):
        rospy.logdebug("New edge detection callback received.")
        img_in_cvformat = self.cv_bridge.imgmsg_to_cv2(req.img)
        processed_img_in_cvformat = self.detect_edges_canny(img_in_cvformat)
        processed_imgmsg = self.cv_bridge.cv2_to_imgmsg(processed_img_in_cvformat, encoding='rgb8')
        # set header
        processed_imgmsg.header = req.img.header
        return EdgeDetectionResponse(processed_imgmsg)

    def detect_corners(self, img, gaussian_smoothing = False, bilateral_smoothing = True, dilate = True):
        if gaussian_smoothing:
            img =  cv.GaussianBlur(img, (5, 5), 0)
        if bilateral_smoothing:
            img = cv.bilateralFilter(img,5,100,100) # experiment more with parameteres

        gray = cv.cvtColor(img,cv.COLOR_RGB2GRAY)
        gray = np.float32(gray)

        dst = cv.cornerHarris(gray,3,5,0.04) # experiment with parameters
        count = np.count_nonzero(dst>0.12*dst.max())
        rospy.logdebug("Corners found in " + count +  " pixels.")

        #result is dilated for marking the corners better, can be disregarded
        if dilate:
            dst = cv.dilate(dst,None)

        # Threshold for an optimal value, 0.1-0.2 seems to be a code choice, default value was 0.01
        img[dst>0.12*dst.max()]=[0,0,255]
        return img

    def detect_edges_canny(self, img):
        # Convert to grayscale
        gray = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
        # Detect edges using Canny algorithm
        edges = cv.Canny(gray, 100, 200)
        # dilate to see
        cv.dilate(edges,None)
        # img is not writable, see img.flags e.g. by printing
        img_writable = np.copy(img)
        img_writable[edges > 0.2] = [0, 255, 0]

        return img_writable


if __name__ == '__main__':
    try:
        Edge_Detection_Service()
    except rospy.ROSInterruptException:
        rospy.logerr("Error occured. Abort...")
