#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

import message_filters
from sensor_msgs.msg import Image, Imu

from cv_bridge import CvBridge

import numpy as np
import cv2
import io, os, yaml

class MynteyeCalibration(Node):

    def __init__(self):
        super().__init__('mynteye_calibration')

        # Declare ROS parameters
        self.declare_parameters(namespace='',
                               parameters=[('qos_length', None),
                                           ('to_calibrate', None),
                                           ('corners_x', None),
                                           ('corners_y', None),
                                           ('samples_count', None),
                                           ('window_size',None)])

        qos_length = self.get_parameter('qos_length').get_parameter_value().integer_value
        qos_profile = QoSProfile(depth=qos_length,
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 reliability=QoSReliabilityPolicy.RELIABLE)

        to_calibrate = self.get_parameter('to_calibrate').get_parameter_value().string_value
        if to_calibrate == 'stereo_images':
            # Declare parameters
            self.imgCalibParams()

            # Create Subscribers
            self.left_sub = message_filters.Subscriber(self, Image, 'left/image_raw', qos_profile=qos_profile)
            self.right_sub = message_filters.Subscriber(self, Image, 'right/image_raw', qos_profile=qos_profile)

            # Apply message filter
            self.timestamp_sync = message_filters.ApproximateTimeSynchronizer([self.left_sub, self.right_sub], queue_size = 2, slop = 0.0005)
            self.timestamp_sync.registerCallback(self.imgCallback)

            # Load cv_bridge
            self.bridge = CvBridge()
        elif to_calibrate == 'imu_data':
            self.imu_sub = self.create_subscription(Imu, 'imu/data_raw', self.imuCallback, qos_profile)

    def imgCalibParams(self):
        self.nx = self.get_parameter('corners_x').get_parameter_value().integer_value
        self.ny = self.get_parameter('corners_y').get_parameter_value().integer_value
        self.samples_count = self.get_parameter('samples_count').get_parameter_value().integer_value
        self.win_size = self.get_parameter('window_size').get_parameter_value().integer_value

        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
        self.corners_criteria = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE

        self.objp = np.zeros((self.nx*self.ny, 1, 3), np.float32)
        self.objp[:, 0, :2] = np.mgrid[0:self.nx, 0:self.ny].T.reshape(-1, 2)

        self.objpoints = []
        self.imgpoints_left = []
        self.imgpoints_right = []

        self.do_once = False

    def imgCallback(self, left_msg, right_msg):
        left_img_raw = self.bridge.imgmsg_to_cv2(left_msg)
        right_img_raw = self.bridge.imgmsg_to_cv2(right_msg)

        concat_img = cv2.hconcat([left_img_raw, right_img_raw])
        cv2.imshow("press 's' to proceed", concat_img)
 
        if cv2.waitKey(1) == ord('s') and self.samples_count > 0:
            ret_left, corners_left = cv2.findChessboardCorners(left_img_raw, (self.nx, self.ny), self.corners_criteria)
            ret_right, corners_right = cv2.findChessboardCorners(right_img_raw, (self.nx, self.ny), self.corners_criteria)

            if ret_left == True and ret_right == True:

                corners_left = cv2.cornerSubPix(left_img_raw,
                                                corners_left,
                                                (self.win_size, self.win_size),
                                                (-1, -1),
                                                self.criteria)
                corners_right = cv2.cornerSubPix(right_img_raw,
                                                 corners_right,
                                                 (self.win_size, self.win_size),
                                                 (-1, -1),
                                                 self.criteria)

                # Draw and display the corners
                cv2.drawChessboardCorners(left_img_raw, (self.nx, self.ny), corners_left, ret_left)
                cv2.drawChessboardCorners(right_img_raw, (self.nx, self.ny), corners_right, ret_right)
                concat_img = cv2.hconcat([left_img_raw, right_img_raw])
                cv2.imshow("press 'y'/'n' to accept/decline", concat_img)
                if cv2.waitKey(0) == ord('y'):
                    self.objpoints.append(self.objp)
                    self.imgpoints_left.append(corners_left)
                    self.imgpoints_right.append(corners_right)
                    self.samples_count -= 1
        elif self.samples_count == 0 and self.do_once == False:
            img_size = left_img_raw.shape[::-1]
            _, K1, D1, _, _ = cv2.calibrateCamera(self.objpoints, self.imgpoints_left, img_size, None, None)
            _, K2, D2, _, _ = cv2.calibrateCamera(self.objpoints, self.imgpoints_right, img_size, None, None)

            flag = cv2.CALIB_FIX_INTRINSIC
            ret, K1, D1, K2, D2, R, T, _, _ = cv2.stereoCalibrate(self.objpoints, self.imgpoints_left, self.imgpoints_right,
                                                                 K1, D1, K2, D2, img_size,
                                                                 self.criteria, flag)
            flag = cv2.CALIB_ZERO_DISPARITY
            R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(K1, D1, K2, D2, img_size, R, T, flags = flag, alpha=0)

            self.get_logger().info('Reprojection error = %f' % ret)
            if ret > 0.0 and ret < 1.0:
                param_dict = {"left": {"D" : D1.flatten().tolist(), "K" : K1.flatten().tolist(),
                                       "R" : R1.flatten().tolist(), "P" : P1.flatten().tolist()},
                             "right" : {"D" : D2.flatten().tolist(), "K" : K2.flatten().tolist(),
                                        "R" : R2.flatten().tolist(), "P" : P2.flatten().tolist()}}

                with open(os.path.join(os.getcwd(), "src/mynteye_ros2_wrapper/config/params.yaml"), "r") as file_in:
                    load_data = yaml.safe_load(file_in)
                
                load_data["mynteye_rectification"]["ros__parameters"]["camera_info"] = param_dict
                
                with io.open(os.path.join(os.getcwd(), "src/mynteye_ros2_wrapper/config/params.yaml"), "w", encoding="utf8") as file_out:
                    yaml.dump(load_data, file_out, default_flow_style=False, allow_unicode=True)
                
                self.get_logger().info("Data written to 'mynteye_ros2_wrapper/config/params.yaml' file")
                self.get_logger().info("Press 'ctrl+c' to close")
            else:
                self.get_logger().info("Reprojection error is not valid. Recalibration is required")

            self.do_once = True
            

    def imuCallback(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)

    calib_node = MynteyeCalibration()

    rclpy.spin(calib_node)

    calib_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
