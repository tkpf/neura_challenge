# !/usr/bin/env python

import sys
import rospy
import cv2 as cv
import numpy as np

from cv_bridge import CvBridge
from edge_detection.srv import EdgeDetection, EdgeDetectionResponse

def detect_edges_in_image(img_in_cvformat):
    bridge = CvBridge()
    imgmsg = bridge.cv2_to_imgmsg(img_in_cvformat, encoding="bgr8")
    print("File converted to cv format. Waiting for Service.")
    rospy.wait_for_service("edge_detection")
    print("Service found.")
    try:
        vision_processing_srv = rospy.ServiceProxy('edge_detection', EdgeDetection)
        print("Calling Service...")
        resp = vision_processing_srv(imgmsg)
        print("Service response received.")
        img_analyzed = bridge.imgmsg_to_cv2(resp.img)
        return img_analyzed
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



if __name__ == "__main__":
    try:
        filepath = sys.argv[2]
        print("Search for file at location %s.")
        img = cv.imread(filepath)
        print("File found and read!")
    except Exception as e:
        print("There was something wrong with the arguments provided. Imagefile could not be read.\n" %e)
        print("Aborting...")
        sys.exit()
            
    img = cv.imread(filepath)
    print("File found and read!")
    img_enriched = detect_edges_in_image(img)

    print("Show Image...")
    cv.imshow('Analyzed_img',img_enriched)
    cv.waitKey(3000) # delay for 5000 ms (5 seconds)
    cv.destroyAllWindows()
    # cv.imshow('Analyzed_Img',img)
    # if cv.waitKey(0) & 0xff == 27:
    #     cv.destroyAllWindows()
