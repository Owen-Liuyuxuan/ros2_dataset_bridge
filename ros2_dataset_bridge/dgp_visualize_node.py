#!/usr/bin/env python3
import numpy as np
import json
import os
import rclpy
import cv2
from dgp.datasets.synchronized_dataset import SynchronizedScene
from .utils.ros_util import ROSInterface
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Int32, Bool

class DgpVisualizeNode(object):
    """ Main node for data visualization. Core logic lies in publish_callback.
    """
    def __init__(self):
        self.ros_interface = ROSInterface("DgpVisualizeNode")

        self.camera_names = ['CAMERA_01', 'CAMERA_05', 'CAMERA_06', 'CAMERA_07', 'CAMERA_08', 'CAMERA_09']
        self.lidar_name = 'LIDAR'
        
        for camera in self.camera_names:
            self.ros_interface.create_publisher(Image, f"/dgp/{camera}/image", 10)
            self.ros_interface.create_publisher(CameraInfo, f"/dgp/{camera}/camera_info", 10)
        self.ros_interface.create_publisher(PointCloud2, f"/dgp/{self.lidar_name.lower()}", 10)

        self.ros_interface.declare_parameter("DDAD_JSON_PATH", "/data/ddad_train_val/ddad.json")
        self.ros_interface.declare_parameter("UPDATE_FREQUENCY", 8.0)

        self.ddad_json_path = self.ros_interface.get_parameter("DDAD_JSON_PATH").get_parameter_value().string_value
        self.update_frequency = self.ros_interface.get_parameter("UPDATE_FREQUENCY").get_parameter_value().double_value

        print(f"update_frequency={self.update_frequency}")
        self.index = 0
        self.published = False
        self.sequence_index = 0
        self.pause = False
        self.stop = True

        scenes_files = json.load(open(self.ddad_json_path, 'r'))['scene_splits']['0']['filenames']
        base_dir_name = os.path.dirname(self.ddad_json_path)
        self.datasets = [
            SynchronizedScene(
                scene_json=os.path.join(base_dir_name, scene_file),
                datum_names = [self.lidar_name] + self.camera_names
            ) for scene_file in scenes_files
        ]
        self.ros_interface.get_logger().info(f"Loaded {len(self.datasets)} scenes from {self.ddad_json_path}")

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
        self.index = msg.data % len(self.datasets)
        self.sequence_index = 0
        print(f"Switch to scenes {self.index}")
        self.publish_callback()

    def _lidar_publish(self, lidar_data, is_publish_lidar=False):
        channel = lidar_data['datum_name']

        rotation = lidar_data['extrinsics'].quat.q #list, [4] r, x, y, z
        translation = lidar_data['extrinsics'].tvec #
        self.ros_interface.publish_transformation_quat(translation, rotation[[1,2,3,0]], "base_link", channel)

        ego_rotation = lidar_data['pose'].quat.q
        ego_translation = lidar_data['pose'].tvec
        self.ros_interface.publish_transformation_quat(ego_translation, ego_rotation[[1,2,3,0]], "world", "base_link")

        if self.sequence_index == 0:
            self.odom_pose = (ego_rotation, ego_translation)
        self.ros_interface.publish_transformation_quat(self.odom_pose[1], self.odom_pose[0][[1,2,3,0]], "world", "odom")

        if is_publish_lidar:
            point_cloud = np.concatenate([lidar_data['point_cloud'], lidar_data['extra_channels']], axis=-1) #[N, 4]
            self.ros_interface.publish_point_cloud(point_cloud, '/dgp/lidar', channel)

    
    def _camera_publish(self, camera_data, is_publish_camera=False):
        channel = camera_data['datum_name']
        
        # publish relative pose
        cam_intrinsic = np.copy(camera_data['intrinsics']) #[3 * 3]
        rotation = camera_data['extrinsics'].quat.q #list, [4] r, x, y, z
        translation = camera_data['extrinsics'].tvec #
        self.ros_interface.publish_transformation_quat(translation, rotation[[1,2,3,0]], "base_link", channel)

        if is_publish_camera:
            image = cv2.cvtColor(np.array(camera_data['rgb']), cv2.COLOR_RGB2BGR)
            ## resize to 1/3
            image = cv2.resize(image, (image.shape[1]//3, image.shape[0]//3))
            cam_intrinsic[0, :] /= 3
            cam_intrinsic[1, :] /= 3
            print(f"image.shape={image.shape}, cam_intrinsic={cam_intrinsic}")
            self.ros_interface.publish_image(image, cam_intrinsic,
                                            f"/dgp/{channel}/image", f"/dgp/{channel}/camera_info", channel)


    def publish_callback(self):
        if self.stop: # if stopped, falls back to an empty loop
            return
        
        self.sequence_index = (self.sequence_index) % len(self.datasets[self.index])
        self.ros_interface.get_logger().info(f"Publishing scene {self.index} frame {self.sequence_index}")

        data_collected = self.datasets[self.index][self.sequence_index][0]
        for data in data_collected:
            if data['datum_name'] == self.lidar_name:
                self._lidar_publish(data, is_publish_lidar=(not self.pause))
            else:
                self._camera_publish(data, is_publish_camera=(not self.pause))

        if self.pause: # if paused, all data will freeze
            return
        
        self.sequence_index += 1
        


def main(args=None):
    rclpy.init(args=args)
    DgpVisualizeNode()