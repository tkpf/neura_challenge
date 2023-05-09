# !/usr/bin/env python

import sys
import rospy
import cv2 as cv
import numpy as np

from cv_bridge import CvBridge
from edge_detection.srv import EdgeDetection, EdgeDetectionResponse

def detect_edges_in_image(img_in_cvformat):
    bridge = CvBridge()
    imgmsg = bridge.cv2_to_imgmsg(img_in_cvformat, encoding="rgb8")
    rospy.loginfo("File converted to cv format. Waiting for Service.")
    rospy.wait_for_service("edge_detection")
    rospy.loginfo("Service found.")
    try:
        vision_processing_srv = rospy.ServiceProxy('edge_detection', EdgeDetection)
        rospy.loginfo("Calling Service...")
        resp = vision_processing_srv(imgmsg)
        rospy.loginfo("Service response received.")
        img_analyzed = bridge.imgmsg_to_cv2(resp.img)
        return img_analyzed
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)



if __name__ == "__main__":
    try:
        filepath = sys.argv[1]
        rospy.loginfo("Search for file at location %s.", filepath)
        img = cv.imread(filepath)
        rospy.loginfo("File found and read!")
    except Exception as e:
        rospy.logerr("There was something wrong with the arguments provided. Imagefile could not be read.\n" %e)
        rospy.logerr("Aborting...")
        sys.exit()

    img_enriched = detect_edges_in_image(img)

    rospy.loginfont("Show Image...")
    cv.imshow('Analyzed_Img',img_enriched)
    if cv.waitKey(0) & 0xff == 27:
        cv.destroyAllWindows()
