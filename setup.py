from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'ros2_dataset_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'rviz'), [os.path.join('rviz', 'kitti360.rviz'), 
                                                      os.path.join('rviz', 'nuscenes.rviz'),
                                                      os.path.join('rviz', 'kitti.rviz')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yliuhb',
    maintainer_email='yliuhb@connect.ust.hk',
    description='A visualization tool for KITTI360, NuScenes and KITTI dataset',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kitti360_visualize_node = ros2_dataset_bridge.kitti360_visualize_node:main',
            'kitti_visualize_node = ros2_dataset_bridge.kitti_visualize_node:main',
            'nuscenes_visualize_node = ros2_dataset_bridge.nuscenes_visualize_node:main',
            'gui_controller = ros2_dataset_bridge.gui_controller_node:main',
        ],
    },
)
