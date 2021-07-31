#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.clock import Clock

import message_filters
from sensor_msgs.msg import Image, Imu, CameraInfo
from std_msgs.msg import Header

from cv_bridge import CvBridge

import numpy as np
import cv2

class MynteyeRectification(Node):

    def __init__(self):
        super().__init__('mynteye_rectification')

        self.declare_parameters(namespace='',
                                parameters=[('qos_length', None),
                                            ('camera_info.left.D', None),
                                            ('camera_info.left.K', None),
                                            ('camera_info.left.R', None),
                                            ('camera_info.left.P', None),
                                            ('camera_info.right.D', None),
                                            ('camera_info.right.K', None),
                                            ('camera_info.right.R', None),
                                            ('camera_info.right.P', None),
                                            ('on_display', None)])

        qos_length = self.get_parameter('qos_length').get_parameter_value().integer_value
        qos_profile = QoSProfile(depth=qos_length,
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 reliability=QoSReliabilityPolicy.RELIABLE)

        # Create Subscribers
        self.left_sub = message_filters.Subscriber(self, Image, 'left/image_raw', qos_profile=qos_profile)
        self.right_sub = message_filters.Subscriber(self, Image, 'right/image_raw', qos_profile=qos_profile)

        # Apply message filter
        self.timestamp_sync = message_filters.ApproximateTimeSynchronizer([self.left_sub, self.right_sub], queue_size = 2, slop = 0.0005)
        self.timestamp_sync.registerCallback(self.imageCallback)

        # Create Publishers
        self.left_pub = self.create_publisher(Image, 'left/image_rect', qos_profile)
        self.camera_info_left_pub = self.create_publisher(CameraInfo, 'left/camera_info', qos_profile)
        self.right_pub = self.create_publisher(Image, 'right/image_rect', qos_profile)
        self.camera_info_right_pub = self.create_publisher(CameraInfo, 'right/camera_info', qos_profile)

        # Load cv_bridge
        self.bridge = CvBridge()

        self.do_once = False
        self.on_display = self.get_parameter('on_display').get_parameter_value().bool_value

    def imageCallback(self, left_msg, right_msg):
        left_img_raw = self.bridge.imgmsg_to_cv2(left_msg)
        right_img_raw = self.bridge.imgmsg_to_cv2(right_msg)
        img_size = left_img_raw.shape[::-1]

        if self.do_once == False:

            if img_size[0] == 376:
                scaleby = 2
            else:
                scaleby = 1
            D1 = np.array(self.get_parameter('camera_info.left.D').get_parameter_value().double_array_value, dtype=np.float64)
            K1 = np.array(self.get_parameter('camera_info.left.K').get_parameter_value().double_array_value, dtype=np.float64) / scaleby
            R1 = np.array(self.get_parameter('camera_info.left.R').get_parameter_value().double_array_value, dtype=np.float64)
            P1 = np.array(self.get_parameter('camera_info.left.P').get_parameter_value().double_array_value, dtype=np.float64) / scaleby

            D2 = np.array(self.get_parameter('camera_info.right.D').get_parameter_value().double_array_value, dtype=np.float64)
            K2 = np.array(self.get_parameter('camera_info.right.K').get_parameter_value().double_array_value, dtype=np.float64) / scaleby
            R2 = np.array(self.get_parameter('camera_info.right.R').get_parameter_value().double_array_value, dtype=np.float64)
            P2 = np.array(self.get_parameter('camera_info.right.P').get_parameter_value().double_array_value, dtype=np.float64) / scaleby

            self.mapx_left, self.mapy_left = cv2.initUndistortRectifyMap(K1.reshape(3, 3), D1.reshape(1,5), R1.reshape(3, 3), P1.reshape(3, 4), img_size, cv2.CV_16SC2)
            self.mapx_right, self.mapy_right = cv2.initUndistortRectifyMap(K2.reshape(3, 3), D2.reshape(1,5), R2.reshape(3, 3), P2.reshape(3, 4), img_size, cv2.CV_16SC2)

            self.camera_info_left = CameraInfo()
            self.camera_info_left.d = D1.flatten().tolist()
            self.camera_info_left.k = K1.flatten().tolist()
            self.camera_info_left.r = R1.flatten().tolist()
            self.camera_info_left.p = P1.flatten().tolist()
            self.camera_info_left.header.frame_id = 'left_frame_id'

            self.camera_info_right = CameraInfo()
            self.camera_info_right.d = D2.flatten().tolist()
            self.camera_info_right.k = K2.flatten().tolist()
            self.camera_info_right.r = R2.flatten().tolist()
            self.camera_info_right.p = P2.flatten().tolist()
            self.camera_info_left.header.frame_id = 'right_frame_id'

            self.do_once = True

        left_img_rect = cv2.remap(left_img_raw, self.mapx_left, self.mapy_left, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)
        right_img_rect = cv2.remap(right_img_raw, self.mapx_right, self.mapy_right, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)

        if self.on_display == True:
            concat_img = cv2.hconcat([left_img_rect, right_img_rect])
            cv2.imshow("img", concat_img)
            cv2.waitKey(1)

        clock = Clock().now()
        left_img_rect_msg = self.bridge.cv2_to_imgmsg(left_img_rect, 'mono8')
        left_img_rect_msg.header.frame_id = 'left_frame_id'
        right_img_rect_msg = self.bridge.cv2_to_imgmsg(right_img_rect, 'mono8')
        right_img_rect_msg.header.frame_id = 'right_frame_id'
        left_img_rect_msg.header.stamp = right_img_rect_msg.header.stamp = clock.to_msg()

        self.camera_info_right.header.stamp = self.camera_info_left.header.stamp = clock.to_msg()

        self.left_pub.publish(left_img_rect_msg)
        self.camera_info_left_pub.publish(self.camera_info_left)
        self.right_pub.publish(right_img_rect_msg)
        self.camera_info_right_pub.publish(self.camera_info_right)

def main(args=None):
    rclpy.init(args=args)

    rect_node = MynteyeRectification()

    rclpy.spin(rect_node)

    rect_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



