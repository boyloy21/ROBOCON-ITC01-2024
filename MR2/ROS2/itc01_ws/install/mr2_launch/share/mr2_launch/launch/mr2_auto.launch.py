

from launch import LaunchDescription

from launch_ros.actions import Node



def generate_launch_description():

    return LaunchDescription([
   
        Node(package='mr2_realsense', executable='yolov8_rs07'),
        
        Node(package='buffalo_robot', executable='can_buffaloV3'),
    
        Node(package='buffalo_robot', executable='omni_pidv4'),
        
        # Node(package='cv_test', executable='realsense_depth'),
        
        # Node(package='rqt_image_view', executable='rqt_image_view'),
    ])