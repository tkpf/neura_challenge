#!/usr/bin/env python

from cv_bridge import CvBridge
from edge_detection.srv import EdgeDetection, EdgeDetectionResponse

def detect_edges_in_image(img_in_cvformat):
    bridge = CvBridge()
    imgmsg = bridge.cv2_to_imgmsg(img_in_cvformat, encoding="mono8")
    print("File converted to cv format. Waiting for Service.")
    rospy.wait_for_service("edge_detection")
    print("Service found.")
    try:
        vision_processing_srv = rospy.ServiceProxy('edge_detection', EdgeDetection)
        print("Calling Service...")
        print(imgmsg)
        #resp = vision_processing_srv(imgmsg)
        print("Service response received.")
        #img_analyzed = bridge.imgmsg_to_cv2(resp.img)
        #return img_analyzed
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)








import rospy
import cv2 as cv
from sensor_msgs.msg import Image
import numpy as np

# from detector_client import detect_edges_in_image

IMAGE_STREAM_TOPIC = '/camera/color/image_raw'

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print(data)
    img = np.frombuffer(data.data, dtype=np.uint8)
    enriched_img = detect_edges_in_image(img)
    cv.imshow('Analyzed Image',enriched_img)
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

