import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    project_package = get_package_share_directory('yolov8_wrapper')

    return LaunchDescription([
        Node(
            package='yolov8_wrapper',  
            executable='image_detector',  
            output='screen',
            parameters=[{
                'model_file_path': os.path.join(project_package,'weights/yolov8s.torchscript'),  
                'data_path': os.path.join(project_package,'weights/coco.names'), 
                'confidence_threshold': 0.4,  
                'iou_threshold': 0.5, 
                'gpu': True,
                'half': False,  
                'pub_image': True
            }],
            remappings=[
                ("input_image", "camera1/image_raw"),
                ("output_image", "output_image")
            ]
        ),
    ])