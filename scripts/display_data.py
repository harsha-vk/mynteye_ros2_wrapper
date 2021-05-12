#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.qos_event import SubscriptionEventCallbacks
import cv2

class DisplayData(Node):

    def __init__(self, qos_profile):
        super().__init__('display_data')
        self.left_sub = self.create_subscription(
                            Image, 'left/image_raw',
                            self._left_callback,
                            qos_profile)
        self.right_sub = self.create_subscription(
                            Image, 'right/image_raw',
                           self._right_callback,
                           qos_profile)  
        self.bridge = CvBridge() 

    def _left_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg)
        cv2.imshow('left', frame)
        cv2.waitKey(1)

    def _right_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg)
        cv2.imshow('right', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    custom_qos_profile = QoSProfile(depth=7, history=QoSHistoryPolicy.KEEP_LAST, reliability=QoSReliabilityPolicy.RELIABLE)
    display_data_sub = DisplayData(custom_qos_profile)
    rclpy.spin(display_data_sub)
    display_data_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
