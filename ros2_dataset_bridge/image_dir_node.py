#!/usr/bin/env python3
import numpy as np
import json
import os
import rclpy
import cv2
from .utils.ros_util import ROSInterface
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Int32

camera_P = np.array([
 [500.168885, 0.0, 360.34088, 0.0,],
    [0.0, 505.752195, 276.8606, 0.0,],
    [0.0, 0.0, 1.0, 0.0]]
)

class ImageDirVisualizeNode(object):
    """ Main node for data visualization. Core logic lies in publish_callback.
    """
    def __init__(self):
        self.ros_interface = ROSInterface("ImageDirVisualizeNode")

        
        self.ros_interface.create_publisher(Image, f"/ImageDir/image", 10)
        self.ros_interface.create_publisher(CameraInfo, f"/ImageDir/camera_info", 10)

        self.ros_interface.declare_parameter("IMAGE_DIR", "/data/any_image_folder")
        self.ros_interface.declare_parameter("UPDATE_FREQUENCY", 8.0)
        self.ros_interface.declare_parameter("SUFFIX", "png")

        self.image_dir = self.ros_interface.get_parameter("IMAGE_DIR").get_parameter_value().string_value
        self.update_frequency = self.ros_interface.get_parameter("UPDATE_FREQUENCY").get_parameter_value().double_value
        self.suffix = self.ros_interface.get_parameter("SUFFIX").get_parameter_value().string_value

        print(f"update_frequency={self.update_frequency}")
        self.index = 0
        self.published = False
        self.sequence_index = 0
        self.pause = False
        self.stop = True

        image_list = os.listdir(self.image_dir)
        self.image_list = [os.path.join(self.image_dir, image) for image in image_list if image.endswith(self.suffix)]
        self.image_list.sort()

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
        self.sequence_index = msg.data % len(self.image_list)
        print(f"Switch to scenes {self.index}")
        self.publish_callback()

    def publish_callback(self):
        if self.stop: # if stopped, falls back to an empty loop
            return
        
        self.ros_interface.get_logger().info(f"Publishing scene {self.index} frame {self.sequence_index}")

        image = cv2.imread(self.image_list[self.sequence_index])
        image = cv2.resize(image, (image.shape[1], image.shape[0]))
        self.ros_interface.publish_image(image, camera_P,
                                        f"/ImageDir/image", f"/ImageDir/camera_info", "camera")

        if self.pause: # if paused, all data will freeze
            return
        
        self.sequence_index = (self.sequence_index + 1) % len(self.image_list)
        


def main(args=None):
    rclpy.init(args=args)
    ImageDirVisualizeNode()