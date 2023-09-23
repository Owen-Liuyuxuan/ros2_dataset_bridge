#!/usr/bin/env python3
import numpy as np

import rclpy
import cv2
from .utils import kitti360_utils
from .utils.ros_util import ROSInterface
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Int32, Bool

class Kitti360VisualizeNode(object):
    """ Main node for data visualization. Core logic lies in publish_callback.
    """
    def __init__(self):
        self.ros_interface = ROSInterface("Kitti360VisualizeNode")

        self.ros_interface.create_publisher(Image, "/kitti360/left_camera/image", 10)
        self.ros_interface.create_publisher(Image, "/kitti360/right_camera/image", 10)
        self.ros_interface.create_publisher(Image, "/kitti360/left_fisheye_camera/image", 10)
        self.ros_interface.create_publisher(Image, "/kitti360/right_fisheye_camera/image", 10)
        self.ros_interface.create_publisher(CameraInfo, "/kitti360/left_camera/camera_info", 10)
        self.ros_interface.create_publisher(CameraInfo, "/kitti360/right_camera/camera_info", 10)
        self.ros_interface.create_publisher(CameraInfo, "/kitti360/left_fisheye_camera/camera_info", 10)
        self.ros_interface.create_publisher(CameraInfo, "/kitti360/right_fisheye_camera/camera_info", 10)
        self.ros_interface.create_publisher(PointCloud2, "/kitti360/lidar", 10)

        self.ros_interface.declare_parameter("KITTI360_RAW_DIR", "")
        self.ros_interface.declare_parameter("Image_PointCloud_Depth", 5.0)
        self.ros_interface.declare_parameter("UPDATE_FREQUENCY", 8.0)

        self.KITTI360_raw_dir = self.ros_interface.get_parameter("KITTI360_RAW_DIR").get_parameter_value().string_value
        self.image_pc_depth   = self.ros_interface.get_parameter("Image_PointCloud_Depth").get_parameter_value().double_value
        self.update_frequency = self.ros_interface.get_parameter("UPDATE_FREQUENCY").get_parameter_value().double_value
        print(f"update_frequency={self.update_frequency}")
        self.index = 0
        self.published = False
        self.sequence_index = 0
        self.pause = False
        self.stop = True

        self.meta_dict = kitti360_utils.get_files(self.KITTI360_raw_dir, self.index)

        self.ros_interface.create_timer(1.0 / self.update_frequency, self.publish_callback)

        self.ros_interface.create_subscription(Int32, "/ui_controller/control/index", self.index_callback, 10)
        self.ros_interface.create_subscription(Bool, "/ui_controller/control/stop", self.stop_callback, 10)
        self.ros_interface.create_subscription(Bool, "/ui_controller/control/pause", self.pause_callback, 10)

        self.ros_interface.spin()

    def stop_callback(self, msg):
        self.published=False
        self.stop = msg.data
        self.sequence_index = 0
        self.publish_callback()

    def pause_callback(self, msg):
        self.pause = msg.data
        self.published=False

    def index_callback(self, msg):
        self.index = msg.data
        self.sequence_index = 0
        self.meta_dict = kitti360_utils.get_files(self.KITTI360_raw_dir, self.index)
        self.publish_callback()

    def publish_callback(self):
        if self.stop: # if stopped, falls back to an empty loop
            return

        meta_dict = self.meta_dict
        if meta_dict is None:
            self.ros_interface.get_logger().warn("meta_dict from kitti360_utils.get_files is None, current_arguments {}"\
                .format([self.KITTI360_raw_dir, self.index]))
            return

        length = min([len(meta_dict['key_frames'])])
        if length == 0:
            self.ros_interface.get_logger().warn("No sequence found at {} index {}".format(self.KITTI360_raw_dir, self.index))
            return
        self.sequence_index = (self.sequence_index) % length

        P0 = meta_dict["calib"]["P0"]
        P1 = meta_dict["calib"]["P1"]
        R0_rect = meta_dict["calib"]["T_rect02cam0"]
        R1_rect = meta_dict["calib"]["T_rect12cam1"]
        T_cam2velo = meta_dict["calib"]["T_cam2velo"]
        T_image0 = meta_dict["calib"]["cam_to_pose"]["T_image0"]
        T_image1 = meta_dict["calib"]["cam_to_pose"]["T_image1"]
        self.ros_interface.publish_transformation(np.linalg.inv(T_cam2velo), 'left_camera', 'lidar')
        self.ros_interface.publish_transformation(T_image0, 'base_link', 'left_camera')
        self.ros_interface.publish_transformation(R0_rect,  'left_camera', 'left_rect')
        self.ros_interface.publish_transformation(T_image1, 'base_link', 'right_camera')
        self.ros_interface.publish_transformation(R1_rect,  'right_camera', 'right_rect')
        self.ros_interface.publish_transformation(meta_dict["calib"]["cam_to_pose"]["T_image2"], 'base_link', 'left_fisheye_camera')
        self.ros_interface.publish_transformation(meta_dict["calib"]["cam_to_pose"]["T_image3"], 'base_link', 'right_fisheye_camera')
        self.ros_interface.publish_transformation(meta_dict["poses"][self.sequence_index], "odom", "base_link")

        if self.pause: # if paused, all data will freeze
            return

        
        frame_idx = meta_dict['key_frames'][self.sequence_index]
        if meta_dict['left_image'] is not None:
            left_image = cv2.imread(meta_dict["left_image"][frame_idx])
            self.ros_interface.publish_image(left_image, P0, "/kitti360/left_camera/image", frame_id="left_camera")
        if meta_dict['right_image'] is not None:
            right_image = cv2.imread(meta_dict["right_image"][frame_idx])
            self.ros_interface.publish_image(right_image, P1, "/kitti360/right_camera/image", frame_id="right_camera")

        if meta_dict['fisheye2_image'] is not None:
            fisheye2_image = cv2.imread(meta_dict["fisheye2_image"][frame_idx])
            self.ros_interface.publish_fisheye_image(fisheye2_image, meta_dict["calib"]['calib2'],
                                                      "/kitti360/left_fisheye_camera/image", frame_id='left_fisheye_camera')

        if meta_dict['fisheye3_image'] is not None:
            fisheye3_image = cv2.imread(meta_dict["fisheye3_image"][frame_idx])
            self.ros_interface.publish_fisheye_image(fisheye3_image, meta_dict["calib"]['calib3'],
                                                      "/kitti360/right_fisheye_camera/image", frame_id='right_fisheye_camera')

        point_cloud = np.fromfile(meta_dict["lidar"][frame_idx], dtype=np.float32).reshape(-1, 4)
        #point_cloud = point_cloud[point_cloud[:, 0] > np.abs(point_cloud[:, 1]) * 0.2 ]
        self.ros_interface.publish_point_cloud(point_cloud, "/kitti360/lidar", frame_id="lidar")
        
        self.sequence_index += 1


def main(args=None):
    rclpy.init(args=args)
    Kitti360VisualizeNode()