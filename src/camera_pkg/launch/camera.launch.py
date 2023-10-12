import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


ld = LaunchDescription()

use_rectify=LaunchConfiguration('use_rectify', default='true')

USB_CAM_DIR=Path(get_package_share_directory("camera_pkg"),"config","cam_params.yaml")

usb_cam_node=Node(
    package="usb_cam",
    executable="usb_cam_node_exe",
    namespace="usb_cam",
    parameters=[USB_CAM_DIR],
)

left_cam_info = LaunchConfiguration("left_cam_info", default="/config/left.yaml")
right_cam_info = LaunchConfiguration("right_cam_info", default="/config/right.yaml")

camera_split_node=Node(
    package="camera_pkg",
    executable="camera_split_node",
    parameters=[
        {"left_cam_info" : left_cam_info},
        {"right_cam_info" : right_cam_info}
    ]
)

img_proc_launch=IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("camera_pkg"),"launch","img_proc.launch.py")
    ),
    condition=IfCondition(use_rectify)
)

ld.add_action(usb_cam_node)
ld.add_action(camera_split_node)
ld.add_action(img_proc_launch)

def generate_launch_description():
    return ld
