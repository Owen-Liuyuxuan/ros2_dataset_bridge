#!/usr/bin/env python3
import numpy as np

import rclpy
import cv2
import os
from .utils import kitti360_utils
from .utils.ros_util import ROSInterface
from .utils.nuscenes_utils import NuscenesLoader
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Int32, Bool
from visualization_msgs.msg import MarkerArray


"""General Class conversion and color definition for Nuscenes

Ideas borrowed from CenterPoint and nuscenes-devkit.
"""
general_to_detection = {
    "human.pedestrian.adult": "pedestrian",
    "human.pedestrian.child": "pedestrian",
    "human.pedestrian.wheelchair": "ignore",
    "human.pedestrian.stroller": "ignore",
    "human.pedestrian.personal_mobility": "ignore",
    "human.pedestrian.police_officer": "pedestrian",
    "human.pedestrian.construction_worker": "pedestrian",
    "animal": "ignore",
    "vehicle.car": "car",
    "vehicle.motorcycle": "motorcycle",
    "vehicle.bicycle": "bicycle",
    "vehicle.bus.bendy": "bus",
    "vehicle.bus.rigid": "bus",
    "vehicle.truck": "truck",
    "vehicle.construction": "construction_vehicle",
    "vehicle.emergency.ambulance": "ignore",
    "vehicle.emergency.police": "ignore",
    "vehicle.trailer": "trailer",
    "movable_object.barrier": "barrier",
    "movable_object.trafficcone": "traffic_cone",
    "movable_object.pushable_pullable": "ignore",
    "movable_object.debris": "ignore",
    "static_object.bicycle_rack": "ignore",
}
nuscenes_names = ['car', "motorcycle", "bicycle", "bus", "truck", "construction_vehicle",  "trailer", "barrier", "traffic_cone", "pedestrian"]

colors = {'bicycle': (255, 61, 99),
          'motorcycle': (255, 61, 99),
          'car': (255, 158, 0),
          'bus': (255, 158, 0),
          'truck': (255, 158, 0),
          'construction_vehicle': (255, 158, 0),
          'trailer': (255, 158, 0),
          'barrier': (0, 0, 0),
          'traffic_cone': (0, 0, 0),
          'pedestrian': (0, 0, 230),
          'ignore': (255, 0, 255)
          }

class NuscenesVisualizeNode(object):
    """ Main node for data visualization. Core logic lies in publish_callback.
    """
    def __init__(self):
        self.ros_interface = ROSInterface("NuscenesVisualizeNode")

        self.ros_interface.declare_parameter("NUSCENES_DIR", "/data/nuscene")
        self.ros_interface.declare_parameter("NUSCENES_VER", "v1.0-trainval")
        self.ros_interface.declare_parameter("UPDATE_FREQUENCY", 8.0)

        self.nuscenes_dir = self.ros_interface.get_parameter("NUSCENES_DIR").get_parameter_value().string_value
        self.nuscenes_version   = self.ros_interface.get_parameter("NUSCENES_VER").get_parameter_value().string_value
        self.update_frequency = self.ros_interface.get_parameter("UPDATE_FREQUENCY").get_parameter_value().double_value


        self.ros_interface.create_publisher(MarkerArray, "/nuscenes/bboxes", 1)
        self.nusc_loader_helper = NuscenesLoader(version=self.nuscenes_version, dataroot=self.nuscenes_dir, verbose=True)
        self.nusc = self.nusc_loader_helper.get_nusc(logger=self.ros_interface.get_logger())
        
        print(f"update_frequency={self.update_frequency}")
        self.index = 0
        self.set_index(0)
        self.published = False
        self.sequence_index = 0
        self.publishing = True
        self.pause = False
        self.stop = True

        # self.meta_dict = kitti360_utils.get_files(self.KITTI360_raw_dir, self.index)

        self.ros_interface.create_timer(1.0 / self.update_frequency, self.publish_callback)

        self.ros_interface.create_subscription(Int32, "/ui_controller/control/index", self.index_callback, 10)
        self.ros_interface.create_subscription(Bool, "/ui_controller/control/stop", self.stop_callback, 10)
        self.ros_interface.create_subscription(Bool, "/ui_controller/control/pause", self.pause_callback, 10)

        self.ros_interface.spin()

    def stop_callback(self, msg):
        self.published=False
        self.stop = msg.data #Bool
        self.current_sample = self.nusc.get('sample', self.current_scene['first_sample_token']) #back to the first sample
        self.ros_interface.clear_all_bbox()
        self.publish_callback()

    def set_index(self, index):
        """Set current index -> select scenes -> print(scene description) -> set current sample as the first sample of the scene
        """
        self.index = index
        self.current_scene = self.nusc.scene[self.index]
        self.published = False
        des = self.current_scene["description"]
        print(f"Switch to scenes {self.index}: {des} ")
        self.current_sample = self.nusc.get('sample', self.current_scene['first_sample_token']) 

    def pause_callback(self, msg):
        self.pause = msg.data

    def index_callback(self, msg):
        self.set_index(msg.data)
        self.publish_callback()


    def _camera_publish(self, camera_data, is_publish_image=False):
        """Publish camera related data, first publish pose/tf information, then publish image if needed

        Args:
            camera_data (Dict): camera data from nuscenes' sample data
            is_publish_image (bool, optional): determine whether to read image data. Defaults to False.
        """        
        cs_record   = self.nusc.get("calibrated_sensor", camera_data['calibrated_sensor_token'])
        ego_record  = self.nusc.get("ego_pose", camera_data['ego_pose_token'])

        image_path = os.path.join(self.nuscenes_dir, camera_data['filename'])
        imsize    = (camera_data["width"], camera_data["height"])
        channel = camera_data['channel']
        
        # publish relative pose
        cam_intrinsic = np.array(cs_record["camera_intrinsic"]) #[3 * 3]
        rotation = cs_record["rotation"] #list, [4] r, x, y, z
        translation = cs_record['translation'] #
        rotation_xyzw = [rotation[i] for i in [1, 2, 3, 0]]

        self.ros_interface.publish_transformation_quat(translation, rotation_xyzw, "base_link", channel)
        

        if is_publish_image:
            image_pub_name = f"{channel}_image_pub"
            if image_pub_name not in self.ros_interface.__pub_registry__:
                self.ros_interface.create_publisher(Image, f"/nuscenes/{channel}/image", 1)
            info_pub_name  = f"{channel}_info_pub"
            if info_pub_name not in self.ros_interface.__pub_registry__:
                self.ros_interface.create_publisher(CameraInfo, f"/nuscenes/{channel}/camera_info", 1)

            image = cv2.imread(image_path)
            self.ros_interface.publish_image(image, cam_intrinsic, f"/nuscenes/{channel}/image", frame_id=channel)

    def _lidar_publish(self, lidar_data, is_publish_lidar=False):
        """Publish lidar related data, first publish pose/tf information and ego pose, then publish lidar if needed

        Args:
            lidar_data (Dict): lidar data from nuscenes' sample data
            is_publish_lidar (bool, optional): determine whether to read lidar data. Defaults to False.
        """
        cs_record   = self.nusc.get("calibrated_sensor", lidar_data['calibrated_sensor_token'])
        ego_record  = self.nusc.get("ego_pose", lidar_data['ego_pose_token'])

        lidar_path = os.path.join(self.nuscenes_dir, lidar_data['filename'])
        channel = 'LIDAR_TOP'
        
        # publish relative pose
        cam_intrinsic = np.array(cs_record["camera_intrinsic"]) #[3 * 3]
        rotation = cs_record["rotation"] #list, [4] r, x, y, z
        translation = cs_record['translation'] #
        rotation_xyzw = [rotation[i] for i in [1, 2, 3, 0]]
        self.ros_interface.publish_transformation_quat(translation, rotation_xyzw, "base_link", channel)

        # publish ego pose
        ego_rotation = ego_record["rotation"]
        ego_translation = ego_record["translation"]
        ego_rotation_xyzw = [ego_rotation[i] for i in [1, 2, 3, 0]]
        if not self.published:
            self.initial_pose = (ego_translation, ego_rotation_xyzw)
            self.published = True
        self.ros_interface.publish_transformation_quat(self.initial_pose[0], self.initial_pose[1], "world", "odom")
        self.ros_interface.publish_transformation_quat(ego_translation, ego_rotation_xyzw, "world", 'base_link')

        # publish lidar
        if is_publish_lidar:
            point_cloud = np.fromfile(os.path.join(self.nuscenes_dir, lidar_data['filename']), dtype=np.float32).reshape(-1, 5)[:, :4]
            lidar_pub_name = 'lidar_pub'
            if lidar_pub_name not in self.ros_interface.__pub_registry__:
                self.ros_interface.create_publisher(PointCloud2, "/nuscenes/LIDAR_TOP/data", 1)
            self.ros_interface.publish_point_cloud(point_cloud, "/nuscenes/LIDAR_TOP/data", frame_id="LIDAR_TOP")
              
    def publish_callback(self):
        if self.stop: # if stopped, falls back to an empty loop
            return

        # Publish cameras and camera info
        channels = ['CAM_BACK',  'CAM_FRONT', 'CAM_FRONT_LEFT', 'CAM_FRONT_RIGHT', 'CAM_BACK_RIGHT', 'CAM_BACK_LEFT']
        for channel in channels:
            data = self.nusc.get('sample_data', self.current_sample['data'][channel])
            self._camera_publish(data, is_publish_image=self.publishing)

        # Publish lidar and ego pose
        data = self.nusc.get('sample_data', self.current_sample['data']['LIDAR_TOP'])
        self._lidar_publish(data, self.publishing)

        # Publish 3D bounding boxes
        _, bboxes, _ = self.nusc.get_sample_data(self.current_sample['data']['LIDAR_TOP'])
        
        markers = MarkerArray()
        for i, box in enumerate(bboxes):
            detection_name = general_to_detection[box.name]
            obj_color = colors[detection_name]
            marker = self.ros_interface.object_to_marker(box, frame_id='LIDAR_TOP', marker_id=i, duration= 1.2 / self.update_frequency, color=obj_color)
            markers.markers.append(marker)
        self.ros_interface.publish("/nuscenes/bboxes", markers)

        self.publishing = not self.pause # if paused, the original images and lidar are latched (as defined in publishers) and we will not re-publish them to save memory access. But we need to re-publish tf and markers

        if not self.pause:
            if (self.current_sample['next'] == ''):
                # If end reached, loop back from the start
                self.current_sample = self.nusc.get('sample', self.current_scene['first_sample_token']) 
            else:
                self.current_sample = self.nusc.get('sample', self.current_sample['next'])

def main(args=None):
    rclpy.init(args=args)
    NuscenesVisualizeNode()