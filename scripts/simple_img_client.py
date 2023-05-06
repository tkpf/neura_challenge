import sys
import rospy
import cv2 as cv
from bridge import CvBridge
import numpy as np
from vision_processing.srv import FindEdgesInImage, FindEdgesInImageResponse



def processImage(filepath):
    img_in_cvformat = cv.imread(filepath)
    print("File found!")
    bridge = CvBridge()
    #dtype, n_channels = bridge.encoding_as_cvtype2('8UC3')    
    #img_in_cvformat = np.ndarray(shape=(480, 640, n_channels), dtype=dtype)
    img_in_matformat = bridge.cv2_to_imgmsg(img_in_cvformat, encoding="bgr8")
    print("File converted to cv format. Waiting for Service.")
    rospy.wait_for_service("vision_processing_srv")
    print("Service found.")
    try:
        vision_processing_srv = rospy.ServiceProxy('vision_processing_srv', FindEdgesInImage)
        print("Calling Service...")
        resp = vision_processing_srv(img_in_matformat)
        print("Service response received.")
        return bridge.imgmsg_to_cv2(resp.imgOutssss)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



if __name__ == "__main__":
    if len(sys.argv) == 2:
        filepath = sys.argv[1]
        print("Search for file at location %s.")
    else:
        print("To many arguments provided. Abort...")
        sys.exit()
    img = processImage(filepath)
    print("Show Image...")
    cv.imshow('dst',img)
    if cv.waitKey(0) & 0xff == 27:
        cv.destroyAllWindows()