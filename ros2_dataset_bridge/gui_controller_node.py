#!/usr/bin/env python3
"""
    This script launchs a control panel for GUI interaction with KITTI360 visualization.
    Currently support object dataset, sequential dataset and depth prediction dataset

    User manual:

    index: integer selection notice do not overflow the index number (especially for kitti360 object dataset)

    Stop: stop any data loading or processing of the visualization node.
    
    Pause: prevent pointer of the sequantial data stream from increasing, keep the current scene.

    Cancel: quit.
"""
import rclpy
from rclpy.node import Node
import sys
from PyQt5 import QtWidgets
from std_msgs.msg import Bool, String, Int32, Float32
from .qt_utils.gui import Ui_Dialog

class RosInterface(Node):
    def __init__(self):
        super().__init__("RosUserInterface")
        self.index_pub = self.create_publisher(Int32, "/ui_controller/control/index", 1)
        self.pause_pub = self.create_publisher(Bool, "/ui_controller/control/pause", 1)
        self.stop_pub = self.create_publisher(Bool, "/ui_controller/control/stop", 1)

    def publish_index(self, index):
        msg = Int32()
        msg.data = index
        self.index_pub.publish(msg)

    def publish_stop_bool(self, boolean):
        msg = Bool()
        msg.data = boolean
        self.stop_pub.publish(msg)

    def publish_pause_bool(self, boolean):
        msg = Bool()
        msg.data = boolean
        self.pause_pub.publish(msg)


class GUIControllerNode:
    def __init__(self):
        self.qt_processor = Ui_Dialog(RosInterface())

    def start(self):
        try: 
            while not self.qt_processor.quit:
                app = QtWidgets.QApplication(sys.argv)
                diag = QtWidgets.QDialog()
                self.qt_processor.setupUi(diag)
                diag.show()
                app.exec_()
                app = None
        except:
            print("GUI closed")
        finally:
            self.qt_processor.ros_interface.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    gui_controller = GUIControllerNode()
    gui_controller.start()
