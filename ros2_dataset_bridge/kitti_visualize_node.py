#!/usr/bin/env python3
import os
import numpy as np
import rclpy
import cv2
from .utils import kitti_utils
from .utils.ros_util import ROSInterface
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Int32, Bool, Float32

class KittiVisualizeNode:
    """ Main node for data visualization. Core logic lies in publish_callback.
    """
    def __init__(self):
        self.ros_interface = ROSInterface("KITTIVisualizeNode")

        self.ros_interface.create_publisher(Image, "/kitti/left_camera/image", 1)
        self.ros_interface.create_publisher(Image, "/kitti/right_camera/image", 1)
        self.ros_interface.create_publisher(CameraInfo, "/kitti/left_camera/camera_info", 1)
        self.ros_interface.create_publisher(CameraInfo, "/kitti/right_camera/camera_info", 1)
        self.ros_interface.create_publisher(MarkerArray, "/kitti/bboxes", 1)
        self.ros_interface.create_publisher(PointCloud2, "/kitti/lidar", 1)
        self.ros_interface.create_publisher(PointCloud2, "/kitti/left_camera_pc", 1)

        self.ros_interface.declare_parameter("KITTI_OBJ_DIR", "/data/kitti_obj")
        self.ros_interface.declare_parameter("KITTI_RAW_DIR", "/data/kitti_raw")
        self.ros_interface.declare_parameter("KITTI_DEPTH_DIR", "/data/data_depth_annotated")
        self.ros_interface.declare_parameter("FLAG_KITTI_OBJ", False) # run kitti obj (True) or kitti raw (False)
        self.ros_interface.declare_parameter("UPDATE_FREQUENCY", 8.0)
        self.ros_interface.declare_parameter("Image_PointCloud_Depth", 5.0)

        self.kitti_obj_dir = self.ros_interface.get_parameter("KITTI_OBJ_DIR").get_parameter_value().string_value
        self.kitti_raw_dir = self.ros_interface.get_parameter("KITTI_RAW_DIR").get_parameter_value().string_value
        self.kitti_depth_dir = self.ros_interface.get_parameter("KITTI_DEPTH_DIR").get_parameter_value().string_value
        self.update_frequency = self.ros_interface.get_parameter("UPDATE_FREQUENCY").get_parameter_value().double_value
        self.image_pc_depth = self.ros_interface.get_parameter("Image_PointCloud_Depth").get_parameter_value().double_value
        self.flag_kitti_obj = self.ros_interface.get_parameter("FLAG_KITTI_OBJ").get_parameter_value().bool_value

        if os.path.isdir(self.kitti_depth_dir):
            self.ros_interface.create_publisher(PointCloud2, "/kitti/depth_image", 1)
        self.kitti_base_dir = [self.kitti_obj_dir, self.kitti_raw_dir]
        self.is_sequence = not self.flag_kitti_obj
        self.index = 0
        self.meta_dict_found = False
        self.sequence_index = 0
        self.select_folder()
        self.pause = False
        self.flag_is_published = False # save CPU usage for frame-based checking
        self.stop = True
        self.ros_interface.create_timer(1.0 / self.update_frequency, self.publish_callback)

        self.ros_interface.create_subscription(Int32, "/ui_controller/control/index", self.index_callback, 10)
        self.ros_interface.create_subscription(Bool, "/ui_controller/control/stop", self.stop_callback, 10)
        self.ros_interface.create_subscription(Bool, "/ui_controller/control/pause", self.pause_callback, 10)

        self.ros_interface.spin()

    def select_folder(self):
        self.meta_dict_found = False
        try:
            selected_folder = self.kitti_base_dir[int(self.is_sequence)]
            self.meta_dict = kitti_utils.get_files(selected_folder, self.index, self.is_sequence, self.kitti_depth_dir)
        except Exception as e:
            self.ros_interface.get_logger().warn(f"Exception in select_folder {selected_folder} index {self.index} is_sequence {self.is_sequence}: {e}")
            self.meta_dict = None
            return
        
        self.meta_dict_found = True

    def stop_callback(self, msg):
        self.stop = msg.data
        self.sequence_index = 0
        self.ros_interface.clear_all_bbox()
        self.publish_callback()

    def pause_callback(self, msg):
        self.pause = msg.data      

    def index_callback(self, msg):
        self.index = msg.data
        self.sequence_index = 0
        self.flag_is_published = False
        self.select_folder()
        self.publish_callback()

    def publish_kitti_tf(self, P2=None, P3=None, T=None, T_imu=None):
        """
        A Robust handler for KITTI tf
        """
        ## broadcast translation and rotation in velodyne T
        stamp = self.ros_interface.get_clock().now().to_msg()
        if T is not None:
            self.ros_interface.publish_transformation(T, "camera_base", "velodyne", stamp)

        ## broadcast the translation from world to base
        if T_imu is None:
            self.ros_interface.get_logger().info(f"updating tf")
            self.ros_interface.publish_transformation_quat([0.0, 0.0, 0.0], [0.5, -0.5, 0.5, -0.5], "base_link", "camera_base", stamp)
        else:
            T_cam2imu = np.linalg.inv(T_imu @ T)
            self.ros_interface.publish_transformation(T_cam2imu, "base_link", "camera_base", stamp)
        if P2 is not None:
            ## broadcast the translation in P2
            self.ros_interface.publish_transformation_quat((-P2[0, 3] / P2[0, 0], 0.0, 0.0),
                                                           [0.0, 0.0, 0.0, 1.0], "camera_base", "left_camera", stamp)
        if P3 is not None:
            ## broadcast translation in P3
            self.ros_interface.publish_transformation_quat((-P3[0, 3] / P3[0, 0], 0.0, 0.0),
                                                           [0.0, 0.0, 0.0, 1.0], "camera_base", "right_camera", stamp)

        
    def publish_callback(self):
        if self.stop: # if stopped, falls back to an empty loop
            return
        
        if not self.meta_dict_found:
            self.ros_interface.get_logger().warn("meta_dict not found, skipping this iteration")
            return

        P2 = self.meta_dict["calib"]["P2"]
        P3 = self.meta_dict["calib"]["P3"]
        T = self.meta_dict["calib"]["T_velo2cam"]
        T_imu = self.meta_dict["calib"]["T_imu2velo"]
        self.publish_kitti_tf(P2, P3, T, T_imu)

        if not self.is_sequence:

            if self.meta_dict["label"]:
                objects = kitti_utils.read_labels(self.meta_dict["label"])

                markers = MarkerArray()
                for i, box in enumerate(objects):
                    marker = self.ros_interface.object_to_marker(box, frame_id='camera_base', marker_id=i, duration= 1.2 / self.update_frequency)
                    markers.markers.append(marker)
                self.ros_interface.publish("/kitti/bboxes", markers)
            
            if self.flag_is_published:
                # if already published image, skip this iteration and only update markers
                return

            left_image = cv2.imread(self.meta_dict["left_image"])
            if left_image is None:
                self.ros_interface.get_logger().warn("No detection found at {} index {}".format(self.kitti_base_dir, self.index))
                return
            self.ros_interface.publish_image(left_image, P2, "/kitti/left_camera/image", frame_id="left_camera")
            right_image = cv2.imread(self.meta_dict["right_image"])
            self.ros_interface.publish_image(right_image, P3, "/kitti/right_camera/image", frame_id="right_camera")

            point_cloud = np.fromfile(self.meta_dict["point_cloud"], dtype=np.float32).reshape(-1, 4)
            point_cloud = point_cloud[point_cloud[:, 0] > np.abs(point_cloud[:, 1]) * 0.2]
            pitch = np.arctan2(point_cloud[:, 2], point_cloud[:, 0])
            point_cloud = point_cloud[ (point_cloud[:, 2] > -2.5) * (point_cloud[:, 2] < 1.5)]
            rgb_point_cloud = kitti_utils.color_pointcloud(point_cloud[:, :3], left_image, T, P2)
            #ros_util.publish_point_cloud(point_cloud, self.lidar_publisher, "velodyne")
            self.ros_interface.publish_point_cloud(rgb_point_cloud, "/kitti/lidar", frame_id="velodyne", field_names="xyzbgr")

            depth_image = np.zeros([left_image.shape[0], left_image.shape[1]])
            depth_image[:, :] = self.image_pc_depth * 256
            depth_point_cloud = self.ros_interface.depth_image_to_point_cloud_array(depth_image, P2[0:3, 0:3], rgb_image=left_image)
            self.ros_interface.publish_point_cloud(depth_point_cloud, "/kitti/left_camera_pc", frame_id="left_camera", field_names='xyzbgr')

            if "depth_image" in self.meta_dict and self.meta_dict["depth_image"] is not None:
                depth_image = cv2.imread(self.meta_dict["depth_image"], -1)#[H, W] uint16 /256.0=depth
                H,W,_ = left_image.shape
                print(H, W, depth_image.shape)
                depth_image = cv2.resize(depth_image, (W, H-100), interpolation=cv2.INTER_NEAREST)[50:]
                P2_depth = P2[0:3, 0:3].copy() #[3, 3]
                P2_depth[1, 2] -= 150
                depth_point_cloud = self.ros_interface.depth_image_to_point_cloud_array(depth_image, P2_depth)
                self.ros_interface.publish_point_cloud(depth_point_cloud, "/kitti/depth_image", frame_id="left_camera", field_names='xyz')

            self.flag_is_published = True
            
        else:
            if self.pause: # if paused, all data will freeze
                return

            length = min([len(self.meta_dict["left_image"]), len(self.meta_dict["right_image"]), len(self.meta_dict["point_cloud"])])

            if length == 0:
                self.ros_interface.get_logger().warn("No sequence found at {} index {}".format(self.kitti_base_dir, self.index))
                return
            self.sequence_index = (self.sequence_index) % length
            
            left_image = cv2.imread(self.meta_dict["left_image"][self.sequence_index])
            self.ros_interface.publish_image(left_image, P2, "/kitti/left_camera/image", frame_id="left_camera")
            right_image = cv2.imread(self.meta_dict["right_image"][self.sequence_index])
            self.ros_interface.publish_image(right_image, P3, "/kitti/right_camera/image", frame_id="right_camera")

            point_cloud = np.fromfile(self.meta_dict["point_cloud"][self.sequence_index], dtype=np.float32).reshape(-1, 4)
            point_cloud = point_cloud[point_cloud[:, 0] > np.abs(point_cloud[:, 1]) * 0.2 ]
            self.ros_interface.publish_point_cloud(point_cloud, "/kitti/lidar", frame_id="velodyne")

            depth_image = np.zeros([left_image.shape[0], left_image.shape[1]])
            depth_image[:, :] = self.image_pc_depth * 256
            depth_point_cloud = self.ros_interface.depth_image_to_point_cloud_array(depth_image, P2[0:3, 0:3], rgb_image=left_image)
            self.ros_interface.publish_point_cloud(depth_point_cloud, "/kitti/left_camera_pc", frame_id="left_camera", field_names='xyzbgr')

            if "depth_images" in self.meta_dict and self.meta_dict["depth_images"] is not None:
                image_name = self.meta_dict["left_image"][self.sequence_index].split("/")[-1]
                for depth_image_path in self.meta_dict["depth_images"]:
                    if image_name in depth_image_path:
                        depth_image = cv2.imread(depth_image_path, -1)#[H, W] uint16 /256.0=depth
                        depth_point_cloud = self.ros_interface.depth_image_to_point_cloud_array(depth_image, P2[0:3, 0:3], rgb_image=left_image)
                        self.ros_interface.publish_point_cloud(depth_point_cloud, "/kitti/depth_image", frame_id="left_camera", field_names='xyz')
            
            if "odom_array" in self.meta_dict and self.meta_dict["odom_array"] is not None:
                self.ros_interface.publish_transformation(self.meta_dict['odom_array'][self.sequence_index], "odom", "base_link")

            self.sequence_index += 1

def main(args=None):
    rclpy.init(args=args)
    KittiVisualizeNode()

if __name__ == "__main__":
    main()
