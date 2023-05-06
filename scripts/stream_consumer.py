#!/usr/bin/env python

import rospy
import cv2 as cv
from sensor_msgs.msg import Image

from edge_detection.detector_client import detect_edges_in_image

IMAGE_STREAM_TOPIC = '/camera/color/image_raw'

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    enriched_img = detect_edges_in_image(data.data)
    cv.imshow('dst',enriched_img)
    cv.waitKey(2000) # delay for 5000 ms (5 seconds)
    cv.destroyAllWindows()

    
def listener():
    # init and subscribe
    rospy.init_node('consumer')
    rospy.Subscriber(IMAGE_STREAM_TOPIC, Image, callback)
    # live on until node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()