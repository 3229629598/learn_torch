import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

ld = LaunchDescription()

USB_CAM_DIR=Path(get_package_share_directory("camera_pkg"),"config","cam_params.yaml")

camera_node=Node(
    package="usb_cam",
    executable="usb_cam_node_exe",
    namespace="usb_cam",
    parameters=[USB_CAM_DIR],
)

camera_split_node=Node(
    package="camera_pkg",
    executable="camera_split_node",
)

ld.add_action(camera_node)
ld.add_action(camera_split_node)

def generate_launch_description():
    return ld