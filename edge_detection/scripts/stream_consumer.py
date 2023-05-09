#!/usr/bin/env python

from cv_bridge import CvBridge
from edge_detection.srv import EdgeDetection, EdgeDetectionResponse

import rospy
import cv2 as cv
from sensor_msgs.msg import Image, CameraInfo, PointCloud
from std_msgs.msg import Header
from geometry_msgs.msg import Point32
import numpy as np
from queue import Queue

IMAGE_COLOR_STREAM_TOPIC = '/camera/color/image_raw'
IMAGE_DEPTH_STREAM_TOPIC = '/camera/depth/image_rect_raw'
IMAGE_COLOR_CAMERA_INFO_TOPIC = '/camera/color/camera_info'
IMAGE_DEPTH_CAMERA_INFO_TOPIC ='/camera/depth/camera_info'
IMAGE_EDGED_TOPIC = '/edge_detection/edged_image'
EDGE_POINTS_3D_TOPIC = '/edge_detection/edge_points'

# TODO make bag file a variable in launch file

#TODO camera info of depth image necessary? or rectified because of topic name?
# TODO check if high and width equals resolution etc
# TODO what does depth value mean??

# TODO frame id, where is the sensor mounted? transform!!


def construct_3d_points(points_2d, img_depth, K_color, K_depth):
    """
    Constructs 3d points given 2d point image coordinates, a depth image and calibration matrices for both cameras

    Args:
        points_2d (np.array): Indices of image pixels of which 3d points should be reconstructed
        img_depth (np.array): Image as numpy array. Pixels representing depth values.
        K_color (3x3 np.array): Calibration matrix for color camera used to capture points_2d.
        K_depth (3x3 np.array): Calibration matrix for depth camera used to capture img_depth. 
    """
    temp = points_2d
    rospy.logdebug("In construct_3d_points() method")
    rospy.logdebug("Max value first axis:" + str(np.max(temp[:,0].flatten())))
    rospy.logdebug("Max value 2nd axis:" + str(np.max(temp[:,1].flatten())))
    # calibration matrices are different for color and depth camera 
    # thus project first to image plane, then check for correspondent depth
    # TODO what is depth, which values does it take?
    rospy.logdebug(f"2dshape : {points_2d.shape}")
    rospy.logdebug(f"img_depth : {img_depth.shape}")
    assert(points_2d.shape[1] == 2)
    assert(K_color.shape == (3,3) and K_depth.shape == (3,3))
    h_points_2d_color = np.concatenate((points_2d, np.full((points_2d.shape[0], 1), 1)), axis=1).T
    rospy.logdebug(f"h_2dshape : {h_points_2d_color.shape}")
    assert(h_points_2d_color.shape[0] == 3)
    # undistort by calibration matrix
    h_points_2d_color_undistorted = np.linalg.inv(K_color) @ h_points_2d_color
    h_points_2d_color_undistorted = h_points_2d_color_undistorted / h_points_2d_color_undistorted[-1, :]
    # get correspondent point in depth camera
    h_points_2d_depth_corespondence = K_depth @ h_points_2d_color_undistorted
    h_points_2d_depth_corespondence = h_points_2d_depth_corespondence / h_points_2d_depth_corespondence[-1, :]
    h_points_2d_depth_corespondence = np.round(h_points_2d_depth_corespondence).astype(int)
    # cut off indices which are out off bounds due to transformation
    correspondence_points_cut_off = []
    final_points_2d = []
    (max_h, max_w) = img_depth.shape
    for  (d_p, c_p) in zip(h_points_2d_depth_corespondence[:2,:].T, h_points_2d_color_undistorted.T):
            if 0 <= d_p[0] < max_h and 0 <= d_p[1] < max_w:
                correspondence_points_cut_off.append(d_p)
                final_points_2d.append(c_p)

    rospy.logdebug("Points before cut off: %s\nPoints after cut off: %s", points_2d.shape[0], len(correspondence_points_cut_off))
    depth = [img_depth[tuple(i)] for i in correspondence_points_cut_off]
    # corresponds to homogenous 3D point on projection plan given inverse depth as 4th entry
    depth = np.array(depth).reshape(-1, 1)
    c_p = np.array(final_points_2d)
    points_3d = final_points_2d * depth
    assert(points_3d.shape[1] == 3)
    return points_3d

class Stream_Consumer:
    '''
    Bundles all Subscribers and Publishers active in edge detection with the given bag file. 
    '''

    # init
    def __init__(self):
        rospy.Subscriber(IMAGE_COLOR_STREAM_TOPIC, Image, self.color_image_callback)
        rospy.Subscriber(IMAGE_DEPTH_STREAM_TOPIC, Image, self.depth_image_callback)
        rospy.Subscriber(IMAGE_COLOR_CAMERA_INFO_TOPIC, CameraInfo, self.camera_color_intrinistics_callback)
        rospy.Subscriber(IMAGE_DEPTH_CAMERA_INFO_TOPIC, CameraInfo, self.camera_depth_intrinistics_callback)
        rospy.Subscriber(IMAGE_EDGED_TOPIC, Image, self.edged_image_callback)

        self.calibration_matrix_color = None
        self.calibration_matrix_depth = None

        self.bridge = CvBridge()
        self.depth_imgmsg_queue = Queue()

        self.edged_image_publisher = rospy.Publisher(IMAGE_EDGED_TOPIC, Image, queue_size =5)
        self.point_3d_publisher = rospy.Publisher(EDGE_POINTS_3D_TOPIC, PointCloud, queue_size =3)


    def broker(self, imgmsg):
        rospy.logdebug("Waiting for Service.")
        rospy.wait_for_service("edge_detection")
        rospy.logdebug("Service found.")
        try:
            vision_processing_srv = rospy.ServiceProxy('edge_detection', EdgeDetection)
            rospy.logdebug("Calling Service...")
            resp = vision_processing_srv(imgmsg)
            rospy.logdebug("Service response received.")
            # publish on new topic
            rospy.logdebug(f"Image with detected edges published to topic {IMAGE_EDGED_TOPIC}")
            self.edged_image_publisher.publish(resp.img)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)
    
    def color_image_callback(self, data):
        rospy.logdebug(rospy.get_caller_id() + "Color image received")
        enriched_img = self.broker(data)

    def edged_image_callback(self, data):
        # extract edge point through "greenness"
        img_in_cvformat = self.bridge.imgmsg_to_cv2(data)
        edges_coordinates = (img_in_cvformat == [0, 255, 0])
        edges_coordinates = np.array(np.argwhere(edges_coordinates[:,:,0]))
        rospy.logdebug("Edge coordinates found")#: \n" + edges_coordinates)
        if not self.depth_imgmsg_queue.empty():
            a = rospy.Duration.from_sec(0.1)
            cur_depth_imgmsg = self.depth_imgmsg_queue.get()
            timegab = cur_depth_imgmsg.header.stamp - data.header.stamp
            # compare timestamps
            while timegab < - a and not self.depth_imgmsg_queue.empty():
                # depth images too old
                rospy.logwarn("Depth image too old. Skipping.")

            if abs(timegab) <= a:
                #
                # extract translations in between sensors
                # depth_camera is bound to 'camera_depth_optical_frame'
                # color_camera to 'camera_color_optical_frame'
                # via cmd 'rostopic echo /tf_static | grep camera_color_optical_frame' no information was found!
                if cur_depth_imgmsg.header.frame_id == "camera_depth_optical_frame":
                    rospy.logwarn("Frame id %s could not be find in topic /tf or /tf_static by manual inspeciton.", cur_depth_imgmsg.header.frame_id)
                    rospy.logwarn("It is assumed that the depth camera is mounted exactly where the color camera is.")
                else:
                    rospy.logwarn("Exact position of depth camera is not considered. Camera is at frame_id: %s", cur_depth_imgmsg.header.frame_id)
                cur_depth_cvimg = self.bridge.imgmsg_to_cv2(cur_depth_imgmsg)
                points_3d = construct_3d_points(edges_coordinates, cur_depth_cvimg, self.calibration_matrix_color, self.calibration_matrix_depth)
                point_cloud = PointCloud()
                #set header
                point_cloud.header =data.header
                # convert to point cloud points using lambda function
                convert_points = lambda p: Point32(*p)
                point_cloud.points = list(map(convert_points, points_3d))
                rospy.logdebug("Publishing new Point cloud...")
                self.point_3d_publisher.publish(point_cloud)
            else: rospy.logwarn("Timegab too high between images: %s second(s)", str(timegab.to_sec()))
        else:
            rospy.logwarn("No depth image in queue waiting. Skipping.")

    def depth_image_callback(self, data):
        self.depth_imgmsg_queue.put(data)

    def camera_color_intrinistics_callback(self, data):
        # By manually inspection it is clear, that all intrinistic parameters of the camera are given with K
        self.calibration_matrix_color = np.array(data.K).reshape(3,3)
        # calibration parameters are persistent, thus unregister after first retrievment
        # TODO rospy.Subscriber.unregister("camera_color_intrinistics")

    def camera_depth_intrinistics_callback(self, data):
        # By manually inspection it is clear, that all intrinistic parameters of the camera are given with K
        self.calibration_matrix_depth = np.array(data.K).reshape(3,3)
        # calibration parameters are persistent, thus unregister after first retrievment
        # TODO rospy.Subscriber.unregister("camera_depth_intrinistics")


if __name__ == '__main__':
    rospy.init_node('stream_broker')
    try:
        Stream_Consumer()
    except Exception as e:
        rospy.logerr("Error occured when initializing Stream_Consumer:%s\n Aborting...", e)

    rospy.logdebug("Keep on spinning...")
    rospy.spin()
    

