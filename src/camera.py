#!/usr/bin/env python
from __future__ import print_function

import roslib

roslib.load_manifest('camera_node')
import sys
import rospy
import cv2
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from collections import deque
import numpy as np
import argparse
import imutils
import time
# import pyrealsense2
from geometry_msgs.msg import Pose


class Intrinsics:
    ppx = None
    ppy = None
    fx = None
    fy = None

    def __init__(self, ppx, ppy, fx, fy):
        self.ppx = ppx
        self.ppy = ppy
        self.fx = fx
        self.fy = fy


class ImageConverter:

    previous_x = None
    previous_y = None
    previous_r = None

    robot_x = None
    robot_y = None
    robot_z = None

    def __init__(self):
        self.bridge = CvBridge()
        self.auth_camera()
        self.are_camera_topics_present()
        image_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
        info_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo)
        self.ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub, info_sub], 10, 0.5)
        self.ts.registerCallback(self.callback)

    def are_camera_topics_present(self):
        topics = rospy.get_published_topics()
        if any("/camera/color/image_raw" in x for x in topics) and any(
                "/camera/aligned_depth_to_color/image_raw" in x for x in topics) and any(
                "/camera/aligned_depth_to_color/camera_info" in x for x in topics):
            print("All topics present - continuing!")
        else:
            raise Exception("Camera topics not found!")

    def auth_camera(self):
        # TODO: find out if there is any way of authenticating the camera
        pass

    def update_previous_position(self, center, radius):
        ImageConverter.previous_x = center[0]
        ImageConverter.previous_y = center[1]
        ImageConverter.previous_r = radius

    def callback(self, rgb_data, depth_data, camera_info):
        try:
            color_image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
        except CvBridgeError as e:
            print(e)

        mask_lower = (0, 165, 143)
        mask_upper = (179, 255, 255)

        if color_image is None:
            return

        # get the position of a robot
        test_circle = (315, 55)
        cv2.circle(color_image, test_circle, 5, (0, 0, 255), -1)
        if not ImageConverter.robot_x or not ImageConverter.robot_y or not ImageConverter.robot_z:
            # ImageConverter.robot_x, ImageConverter.robot_y, ImageConverter.robot_z = convert_depth_to_phys_coord_using_realsense(
            #         test_circle[0], test_circle[1], depth_image[test_circle[1]][test_circle[0]], camera_info)
            ImageConverter.robot_x, ImageConverter.robot_y, ImageConverter.robot_z = convert_depth_to_phys_coord(
                test_circle[0], test_circle[1], depth_image[test_circle[1]][test_circle[0]], camera_info)

        # detect the ball
        blurred = cv2.GaussianBlur(color_image, (1, 1), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, mask_lower, mask_upper)
        mask = cv2.erode(mask, None, iterations=4)
        mask = cv2.dilate(mask, None, iterations=4)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        # if one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask and use it to compute the minimum enclosing circle
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # check if the position of the ball should be updated
            if not ImageConverter.previous_x or not ImageConverter.previous_y or not ImageConverter.previous_r:
                self.update_previous_position(center, radius)

            if abs(center[0] - ImageConverter.previous_x) > 2 or abs(center[1] - ImageConverter.previous_y) > 2 or abs(radius - ImageConverter.previous_r) > 0:
                self.update_previous_position(center, radius)
            else:
                center = (ImageConverter.previous_x, ImageConverter.previous_y)
                radius = ImageConverter.previous_r

            # print the circles(centre and ball contours) on the frame
            cv2.circle(color_image, center, int(radius), (0, 255, 255), 2)
            cv2.circle(color_image, center, 5, (0, 0, 255), -1)

            # get the coordinates and publish them
            # x, y, z = convert_depth_to_phys_coord_using_realsense(center[0], center[1], depth_image[center[1]][center[0]], camera_info)
            x, y, z = convert_depth_to_phys_coord(center[0], center[1], depth_image[center[1]][center[0]], camera_info)

            rospy.loginfo('Distance of a ball to robot %s, %s, %s', -(y - ImageConverter.robot_y - 0.105), (x - ImageConverter.robot_x), -(z - ImageConverter.robot_z))
            publish_3d_point_coordinates(-(y - ImageConverter.robot_y - 0.105), (x - ImageConverter.robot_x), -(z - ImageConverter.robot_z))

        cv2.imshow("Ball tracking", color_image)
        cv2.waitKey(3)


# def convert_depth_to_phys_coord_using_realsense(x, y, depth, cameraInfo):
#     _intrinsics = pyrealsense2.intrinsics()
#     _intrinsics.width = cameraInfo.width
#     _intrinsics.height = cameraInfo.height
#     _intrinsics.ppx = cameraInfo.K[2]
#     _intrinsics.ppy = cameraInfo.K[5]
#     _intrinsics.fx = cameraInfo.K[0]
#     _intrinsics.fy = cameraInfo.K[4]
#     # _intrinsics.model = cameraInfo.distortion_model
#     _intrinsics.model = pyrealsense2.distortion.none
#     _intrinsics.coeffs = [i for i in cameraInfo.D]
#     # depth unit is set to 0.001 m, and the function requires distance in m, so thats why I'm dividing it by 1000
#     result = pyrealsense2.rs2_deproject_pixel_to_point(_intrinsics, [x, y], depth / 1000)  # result[0]: right, result[1]: down, result[2]: forward
#
#     return round(result[0], 3), round(-result[1], 3), round(result[2], 3)


def convert_depth_to_phys_coord(x_pixel, y_pixel, depth, cameraInfo):

    # converting to right unit (0.001 m)
    depth = depth / 1000
    intrinsics = Intrinsics(cameraInfo.K[2], cameraInfo.K[5], cameraInfo.K[0], cameraInfo.K[4])

    x_coordinate = depth * ((x_pixel - intrinsics.ppx) / intrinsics.fx)
    y_coordinate = depth * ((y_pixel - intrinsics.ppy) / intrinsics.fy)

    return round(x_coordinate, 3), round(-y_coordinate, 3), round(depth, 3)


def publish_3d_point_coordinates(x, y, z):
    pub = rospy.Publisher('ball_coordinates', Pose, queue_size=1)

    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation.x = 0.0
    p.orientation.x = 0.0
    p.orientation.x = 0.0
    p.orientation.w = 0.0
    pub.publish(p)


if __name__ == '__main__':
    rospy.loginfo("Start")
    rospy.init_node('image_converter', anonymous=True)
    ImageConverter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
