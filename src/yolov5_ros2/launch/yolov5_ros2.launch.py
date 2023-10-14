import os
from launch_ros.actions import Node
from launch import LaunchDescription

ld = LaunchDescription()

yolov5_node=Node(
    package="yolov5_ros2",
    executable="yolo_detect_2d",
    parameters=[{"device":"cpu",
                 "image_topic":"/left_cam/image_rect",
                 "show_result":True,
                 }]
)

ld.add_action(yolov5_node)

def generate_launch_description():
    return ld
