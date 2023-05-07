#!/usr/bin/env python

from cv_bridge import CvBridge
from edge_detection.srv import EdgeDetection, EdgeDetectionResponse

def detect_edges_in_image(img_in_cvformat):
    bridge = CvBridge()
    imgmsg = bridge.cv2_to_imgmsg(img_in_cvformat, encoding="rgb8")
    print("File converted to cv format. Waiting for Service.")
    rospy.wait_for_service("edge_detection")
    print("Service found.")
    try:
        vision_processing_srv = rospy.ServiceProxy('edge_detection', EdgeDetection)
        print("Calling Service...")
        #resp = vision_processing_srv(imgmsg)
        print("Service response received.")
        #img_analyzed = bridge.imgmsg_to_cv2(resp.img)
        #return img_analyzed
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def broker(imgmsg):
    print("Waiting for Service.")
    rospy.wait_for_service("edge_detection")
    print("Service found.")
    try:
        vision_processing_srv = rospy.ServiceProxy('edge_detection', EdgeDetection)
        print("Calling Service...")
        resp = vision_processing_srv(imgmsg)
        print("Service response received.")
        # publish on new topic
        print(f"Image with detected edges published to topic {IMAGE_EDGED_TOPIC}")
        edged_image_publisher.publish(resp.img)
        # TODO not needed anymore if image dont get printed
        # img_analyzed = bridge.imgmsg_to_cv2(resp.img)
        # return img_analyzed
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# TODO eg starting analyzing of 3d points in new thread to be more efficient





import rospy
import cv2 as cv
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
import numpy as np

# from detector_client import detect_edges_in_image

IMAGE_COLOR_STREAM_TOPIC = '/camera/color/image_raw'
IMAGE_DEPTH_STREAM_TOPIC = '/camera/depth/image_rect_raw'
IMAGE_COLOR_CAMERA_INFO_TOPIC = '/camera/color/camera_info'
IMAGE_EDGED_TOPIC = '/edge_detection/edged_image'
EDGE_POINTS_3D_TOPIC = '/edge_detection/edge_points'

#TODO camera info of depth image necessary? or rectified because of topic name?
# TODO check if high and width equals resolution etc

cur_depth_image = None
calibration_matrix_color = None
calibration_matrix_depth = None

edged_image_publisher = rospy.Publisher(IMAGE_EDGED_TOPIC, Image, queue_size =5)
point_3d_publisher = rospy.Publisher(EDGE_POINTS_3D_TOPIC, Point, queue_size =3)

#TODO more efficient by passing whole image array at once, due to matrix operations this remains the same
def construct_3d_points(points_2d, img_depth, K_color, K_depth):
    # calibration matrices are different for color and depth camera 
    # thus project first to image plane, then check for correspondent depth
    # TODO what is depth, which values does it take?
    print(points_2d.shape)
    print(img_depth.shape)
    assert(points_2d.shape[1] == 2)
    assert(K_color.shape == (3,3) and K_depth.shape == (3,3))
    h_points_2d_color = points_2d.T + np.ones(points_2d.shape[0])
    assert(h_points_2d_color.shape[0] == 3)
    # undistort by calibration matrix
    h_points_2d_color_undistorted = np.linalg.inv(K_color) @ h_points_2d_color
    h_points_2d_color_undistorted = h_points_2d_color_undistorted / h_points_2d_color_undistorted[-1, :]
    # get correspondent point in depth camera
    h_points_2d_depth_corespondence = K_depth @ h_points_2d_color_undistorted
    h_points_2d_depth_corespondence = h_points_2d_depth_corespondence / h_points_2d_depth_corespondence[-1, :]
    h_points_2d_depth_corespondence = np.round(h_points_2d_depth_corespondence).astype(int)
    depth = [img_depth[tuple(i)] for i in h_points_2d_depth_corespondence[:2,:].T]
    # corresponds to homogenous 3D point on projection plan given inverse depth as 4th entry
    points_3d = h_points_2d_color_undistorted * depth
    assert(points_3d.shape[0] == 3)
    return points_3d.T

# TODO class so that publisher pub, does not need to be in global scope but can be passed to class
# better data ja
def color_image_callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    enriched_img = broker(data)

    # cv.imshow('Analyzed Image',enriched_img)
    # cv.waitKey(2000) # delay for 5000 ms (5 seconds)
    # cv.destroyAllWindows()

def edged_image_callback(data):
    # extract edge point through "greenness"
    img_in_cvformat = CvBridge().imgmsg_to_cv2(data)
    edges_coordinates = np.array(np.where(img_in_cvformat == [0, 255, 0]))
    #edges_coordinates = [list(i) for i in edges_coordinates]

    points_3d = construct_3d_points(edges_coordinates, cur_depth_image, calibration_matrix_color, calibration_matrix_depth)


# TODO threading, wait for image data to drop in
def depth_image_callback(data):
    cur_depth_image = data

def camera_color_intrinistics_callback(data):
    # By manually inspection it is clear, that all intrinistic parameters of the camera are given with K
    calibration_matrix_color = data.K
    # calibration parameters are persistent, thus unregister after first retrievment
    # TODO rospy.Subscriber.unregister("camera_color_intrinistics")

def camera_depth_intrinistics_callback(data):
    # By manually inspection it is clear, that all intrinistic parameters of the camera are given with K
    calibration_matrix_depth = data.K
    # calibration parameters are persistent, thus unregister after first retrievment
    # TODO rospy.Subscriber.unregister("camera_depth_intrinistics")

def color_image_consumer():
    # init and subscribe
    rospy.Subscriber(IMAGE_COLOR_STREAM_TOPIC, Image, color_image_callback)

def depth_image_consumer():
    # init and subscribe
    rospy.Subscriber(IMAGE_DEPTH_STREAM_TOPIC, Image, depth_image_callback)

def camera_color_intrinistics_listener():
    # init and subscribe
    rospy.Subscriber(IMAGE_COLOR_CAMERA_INFO_TOPIC, CameraInfo, camera_color_intrinistics_callback)

def camera_depth_intrinistics_listener():
    # init and subscribe
    rospy.Subscriber(IMAGE_COLOR_CAMERA_INFO_TOPIC, CameraInfo, camera_depth_intrinistics_callback)

def edged_image_consumer():
    rospy.Subscriber(IMAGE_EDGED_TOPIC, Image, edged_image_callback)

# TODO only spin camera_intrinistics_callback until data is fetched once
# TODO compare timestamps with depth_image and color_image

# TODO one global cvBridge, declared in main instead of several instances
if __name__ == '__main__':
    rospy.init_node('stream_broker')
    camera_color_intrinistics_listener()
    camera_depth_intrinistics_listener()
    color_image_consumer()
    depth_image_consumer()
    edged_image_consumer()
    print("Keep on spinning...")
    rospy.spin()
    

