# Nuscenes Visualization with ROS-Rviz

This package aims to visualize [nuscenes](https://www.nuscenes.org) dataset in rviz environment. 

We fully export nuscenes data to standard ROS topics.

## Visualizing for Nuscenes in ROS
```
# modify path in launch file as necessary
ros2 launch ros2_dataset_bridge nuscenes_launch.xml
```
**Notice/TODO**: Wait until the Logging INFO outputs nuscenes loaded, before toggling the GUI. The node is not responsive before nuscenes loaded.

![gif](nuscene_visualized.gif)

## Key features

- [x] Nuscenes keyframes support. 
- [x] Six cameras.
- [x] LiDARs.
- [x] Ground truth bounding boxes.
- [x] Pose and TF-trees (world, odom, base_link and sensors).
- [x] GUI control & ROS topic control.
- [ ] Nuscenes sweeps support. (the images / lidar not synchronized well)
- [ ] RADARs.

## Setup

Install nuscenes-devkit
```
pip3 install nuscenes-devkit
```

This repo runs with ROS2 python3 (humble), and we expect PyQt5 correctly setup with ROS installation.

Clone the repo under the {workspace}/src/ folder. Overwrite the folder names in the [launch file](../launch/nuscenes_launch.xml) to point to your data. 

Install and launch everything based on the [Install Steps](../readme.md)


## Default Topics

### Publish

| topic name                        | Description |
|-----------------------------------|---------------|
| /nuscenes/{channel}/image         | image.|
| /nuscenes/{channel}/camera_info   | intrinsic parameters.|
| /nuscenes/LIDAR_TOP/data          | Point cloud data.|
| /nuscenes/bboxes                  | MarkerArray visualizing bounding boxes.
