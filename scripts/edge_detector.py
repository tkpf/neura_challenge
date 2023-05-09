# !/usr/bin/env python

# some snippets may originally come from https://github.com/lucasw/image_manip/blob/master/image_manip/scripts/image_folder_publisher.py
import rospy
import cv2 as cv
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from edge_detection.srv import EdgeDetection, EdgeDetectionResponse



def edge_detection_service():
    rospy.init_node("edge_detection_service")
    s = rospy.Service("edge_detection", EdgeDetection, detection_service_callback)
    print("Ready to process Image...")
    rospy.spin()

def detection_service_callback(req):
    bridge = CvBridge()
    img_in_cvformat = bridge.imgmsg_to_cv2(req.img)
    processed_img_in_cvformat = detect_edges_canny(img_in_cvformat)
    processed_imgmsg = bridge.cv2_to_imgmsg(processed_img_in_cvformat, encoding='rgb8')
    # set header
    print(f"This is the image header:\n{req.img.header}")
    processed_imgmsg.header = req.img.header
    return EdgeDetectionResponse(processed_imgmsg)

def detect_corners(img):
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
    print(img)
    img[dst>0.12*dst.max()]=[0,0,255]
    return img


def detect_edges_canny(img):
    # Convert to grayscale
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Detect edges using Canny algorithm
    edges = cv.Canny(gray, 100, 200)
    # dilate to see
    cv.dilate(edges,None)
    # img is not writable, see img.flags e.g. by printing
    img_writable = np.copy(img)
    img_writable[edges > 0.2] = [0, 255, 0]

    # Find contours of edges
    contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    # Draw contours on original image in green color
    #cv.drawContours(img, contours, -1, (0, 255, 0), 2)

    

    # # Iterate over contours and extract start and end points
    # for contour in contours:
    #     # Get the first and last point in the contour
    #     start_point = tuple(contour[0][0])
    #     end_point = tuple(contour[-1][0])
    #     print(f"Start at {start_point}, End at {end_point}")

    return img_writable


if __name__ == '__main__':
    print("Main called.")
    try:
        edge_detection_service()
    except rospy.ROSInterruptException:
        print("Error occured. Abort...")
        rospy.logerr("Error")
