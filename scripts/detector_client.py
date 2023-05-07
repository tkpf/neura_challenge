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
    if len(sys.argv) == 3:
        filepath = sys.argv[2]
        print("Search for file at location %s.")
    else:
        # Note: If script is run via rosrun command (and not roslaunch which is taken as default, the correct number of args is 2. then filepath= sys.argv[1]
        print("Wrong number of arguments provided. Abort...")
        print("Note: If script is run via rosrun command, you may need to check for only 2 arguments. Default is roslaunch with 3 arguments.")
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
